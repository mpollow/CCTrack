// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "pinmap.h"
#include "usb_descriptors.h"
#include "bno085_driver.h"
#include "cdc_io.h"
#include "command_processor.h"
#include "config_storage.h"
#include "config_schema.h"
#include "midi_mapper.h"
#include "midi_io.h"
#include "quat_math.h"
#include "button.h"
#include "led_status.h"

namespace {
Config           g_cfg;
MidiMapperState  g_mapper{};
ButtonState      g_btn{};
Quat             g_rezero_offset = {1, 0, 0, 0};
Quat             g_mount_quat    = {1, 0, 0, 0};   // mount correction = inverse of the physical mount displacement; derived from g_cfg.mount_ypr at boot + on set_config
Quat             g_last_rotated  = {1, 0, 0, 0};   // for hemisphere canonicalisation (see loop)
bool             g_rezeroed_this_session = false;  // sticky once any rezero has happened (see flags byte, docs/protocol.md §1.3)
bool             g_pending_rezero = false;
bool             g_pending_save = false;
bool             g_pending_usb_reattach = false;
uint8_t          g_last_accuracy = 0;
uint32_t         g_rezero_flash_until_ms = 0;
constexpr uint8_t kCalibThreshold = 2;   // BNO accuracy: 0=unrel, 1=low, 2=med, 3=high
}

// mount_ypr is the sensor's physical displacement from the user-aligned position
// (intrinsic ZYX yaw->pitch->roll). The correction that undoes it is the inverse,
// i.e. the conjugate. conj() negates the angles AND reverses the composition
// order; merely negating the three angles in ZYX is NOT the inverse once two or
// more axes are nonzero.
static Quat mount_correction_from_ypr(const float ypr_deg[3]) {
    constexpr float kDegToRad = (float)(M_PI / 180.0);
    Euler e{ ypr_deg[0] * kDegToRad,
             ypr_deg[1] * kDegToRad,
             ypr_deg[2] * kDegToRad };
    return quat_conj(euler_zyx_to_quat(e));
}

static Config get_cfg_cb()              { return g_cfg; }
static bool   set_cfg_cb(const Config* c){
    if (strncmp(g_cfg.device_name, c->device_name, DEVICE_NAME_MAX) != 0) {
        g_pending_usb_reattach = true;
    }
    // enable_midi controls whether the MIDI interface is present in the
    // USB descriptor. Toggling it requires re-running cctrack_usb::configure
    // and a reattach — but Adafruit_USBD_MIDI::begin() only takes effect at
    // boot, so the host will see the change after the next power-cycle.
    // The reattach below still helps for the CDC half (string descriptors).
    if (g_cfg.enable_midi != c->enable_midi) {
        g_pending_usb_reattach = true;
    }
    // Preserve the zero point if the mount rotation changed:
    // rezero_new = rezero_old * conj(q_old) * q_new.
    bool mount_changed = (g_cfg.mount_ypr[0] != c->mount_ypr[0])
                      || (g_cfg.mount_ypr[1] != c->mount_ypr[1])
                      || (g_cfg.mount_ypr[2] != c->mount_ypr[2]);
    Quat q_old = g_mount_quat;
    Quat q_new = mount_correction_from_ypr(c->mount_ypr);
    g_cfg = *c;
    g_mount_quat = q_new;
    if (mount_changed) {
        g_rezero_offset = quat_mul(g_rezero_offset, quat_mul(quat_conj(q_old), q_new));
    }
    midi_io::set_enabled(g_cfg.enable_midi);
    g_mapper = MidiMapperState{};
    g_pending_save = true;
    return true;
}

static void rezero_to(Quat current_raw) {
    // Full pipeline (loop): conj(offset) * (s.q * mount).
    // At zero pose this must be identity  =>  offset = s.q * mount.
    g_rezero_offset = quat_mul(current_raw, g_mount_quat);
    g_mapper = MidiMapperState{};
    g_last_rotated = {1, 0, 0, 0};   // hemisphere reference resets to identity
    g_rezero_flash_until_ms = millis() + 300;
    g_rezeroed_this_session = true;
    cdc_io::send_status(/*code=*/10, 0, "re-zeroed");
}

static void rezero_request_cb() { g_pending_rezero = true; }

static void save_cal_cb() {
    bool ok = bno085::cal_save();
    cdc_io::send_status(/*code=*/5, ok ? 1 : 0, ok ? "cal_saved" : "cal_save_failed");
}

// Edge-triggered button servicing. Called both inside the BNO drain loop (for
// full-rate debounce) and once per outer loop() iteration so the button still
// works when the IMU produces no samples. button_step only fires SHORT on
// release and gates LONG with long_already_emitted, so calling it from both
// sites never double-fires.
static void service_button() {
    bool p = (digitalRead(pin::BUTTON) == LOW);
    ButtonEvent bev = BUTTON_EVENT_NONE;
    if (button_step(&g_btn, p, millis(), &bev)) {
        if (bev == BUTTON_EVENT_SHORT) {
            g_pending_rezero = true;
            cdc_io::send_status(2, 0, "btn_short");
        } else if (bev == BUTTON_EVENT_LONG) {
            save_cal_cb();
            cdc_io::send_status(3, 0, "btn_long");
        }
    }
}

static void on_cdc_frame(uint8_t type,
                         const uint8_t* payload, size_t len, void*) {
    cmd::on_frame(type, payload, len);
}

void setup() {
    storage::begin();
    g_cfg = storage::load_or_defaults();
    g_mount_quat = mount_correction_from_ypr(g_cfg.mount_ypr);

    cctrack_usb::configure(g_cfg.device_name, g_cfg.enable_midi);
    midi_io::set_enabled(g_cfg.enable_midi);
    Serial.begin(115200);     // must register CDC before detach/attach on Adafruit core
    TinyUSBDevice.detach();   // unconditional: ensures host sees a clean reconnect
    delay(100);               // 100ms so host reliably detects disconnect
    TinyUSBDevice.attach();

    led::begin();
    pinMode(pin::BUTTON, INPUT_PULLUP);

    // Pump tud_task() during the CDC-ready wait so tud_midi_mounted() updates
    // before loop() begins — otherwise early MIDI writes are silently dropped.
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 2000) {
        TinyUSBDevice.task();
        delay(1);
    }

    if (!bno085::begin()) {
        cdc_io::send_status(21, 0, "bno085 init failed");
        led::set(led::STATE_ERROR);
        while (true) { led::update(millis()); delay(5); }
    }

    cmd::Hooks hooks{};
    hooks.rezero            = &rezero_request_cb;
    hooks.save_calibration  = &save_cal_cb;
    hooks.set_config        = &set_cfg_cb;
    hooks.get_config        = &get_cfg_cb;
    cmd::install(&hooks);

    cdc_io::set_rx_callback(&on_cdc_frame, nullptr);
    g_pending_rezero = true;
    cdc_io::send_status(1, 0, "boot");
}

void loop() {
    TinyUSBDevice.task();   // pump USB at top: updates tud_midi_mounted() before any write

    {
        bno085::Sample s;
        while (bno085::poll(&s)) {
            TinyUSBDevice.task();   // keep USB flowing; prevents MIDI/CDC FIFO starvation
            cdc_io::poll_rx();      // drain any commands queued while BNO was polled

            // A config command just queued a flash write/reattach. Break out so
            // it runs this loop() iteration instead of being starved by a
            // continuously-filled BNO FIFO.
            if (g_pending_save || g_pending_usb_reattach) break;

            // Sample button inside the BNO loop so debounce runs at BNO rate,
            // not at the (much slower) outer-loop rate.
            service_button();

            if (g_pending_rezero) {
                g_pending_rezero = false;
                rezero_to(s.q);
            }
            g_last_accuracy = s.accuracy;
            Quat rotated = quat_mul(quat_conj(g_rezero_offset),
                                quat_mul(s.q, g_mount_quat));
            if (g_cfg.conjugate_output) rotated = quat_conj(rotated);

            // BNO085 occasionally emits the −q representation of the same
            // rotation mid-motion (quaternion double cover). Without this
            // canonicalisation that flip turns into a 14000-unit jump on every
            // CC14 the moment it happens. Track the previous output so genuine
            // rotations past 180° also stay continuous. Reference resets to
            // identity on rezero.
            rotated = quat_canonical_to(rotated, g_last_rotated);
            g_last_rotated = rotated;

            uint8_t flags = g_rezeroed_this_session ? 0x01 : 0x00;
            uint32_t now_ms = millis();
            // CDC streams at BNO native rate (~400 Hz); cdc_io drops frames if
            // the TX FIFO can't keep up, so command ACKs are never starved.
            if (g_cfg.stream_quat_cdc) {
                cdc_io::send_quat(rotated.w, rotated.x, rotated.y, rotated.z,
                                  flags, s.accuracy, micros());
            }

            if (g_cfg.enable_midi) {
                MidiEvent evs[8];
                int n = midi_mapper_step(&g_mapper, &g_cfg, rotated,
                                         now_ms, evs, 8);
                for (int i = 0; i < n; i++) midi_io::emit(evs[i]);
            }
        }
    }

    // Service the button here too: the in-loop call above only runs while the
    // BNO is producing samples, so this keeps rezero / cal-save responsive when
    // the IMU stalls.
    service_button();

    cdc_io::poll_rx();

    {
        uint32_t now = millis();
        led::State next;
        // Subtraction-style comparison (wrap-safe across the ~49.7-day
        // millis() rollover), matching the rest of the codebase's timing
        // checks; a plain `now < g_rezero_flash_until_ms` would flash
        // wrongly (or not at all) once per rollover.
        if      ((int32_t)(g_rezero_flash_until_ms - now) > 0) next = led::STATE_REZERO_FLASH;
        else if (!TinyUSBDevice.mounted())             next = led::STATE_NO_HOST;
        else if (g_last_accuracy < kCalibThreshold)    next = led::STATE_CALIBRATING;
        else                                           next = led::STATE_OK;
        led::set(next);
        led::update(now);
    }

    if (g_pending_save) {
        g_pending_save = false;
        TinyUSBDevice.task();   // flush MIDI/CDC before the flash stall
        if (!storage::save(g_cfg)) {
            cdc_io::send_status(/*code=*/22, 0, "flash_save_failed");
        }
        TinyUSBDevice.task();   // resume USB immediately after stall
    }

    if (g_pending_usb_reattach) {
        g_pending_usb_reattach = false;
        // Drain the set_config ACK to the host before detaching. One task() only
        // loads the IN endpoint; the host still has to poll it and deliver the
        // bytes to userspace. Detaching immediately races that delivery, so
        // tools see "no response" / a disconnect mid-read. Pump the stack for a
        // short window so the ACK actually lands first.
        uint32_t drain_t0 = millis();
        while (millis() - drain_t0 < 30) {
            TinyUSBDevice.task();
            delay(1);
        }
        cctrack_usb::set_product_name(g_cfg.device_name);
        TinyUSBDevice.detach();
        delay(100);             // host needs ~100ms to register disconnect
        TinyUSBDevice.attach();
        // Note: a flipped enable_midi only takes full effect after a reboot
        // (the MIDI interface can't be added/removed on the fly). The set_config
        // ACK is enough to let the host know to power-cycle. Saved flash will
        // restore the new state on next boot.
    }
}
