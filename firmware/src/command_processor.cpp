// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "command_processor.h"
#include "cdc_io.h"
#include "config_schema.h"
#include "fw_version.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <string.h>

namespace cmd {

namespace { Hooks s_hooks{}; }

void install(const Hooks* hooks) { s_hooks = *hooks; }

static void send_ack(uint8_t cmd_id, uint8_t status,
                     const uint8_t* payload, size_t len) {
    uint8_t buf[CONFIG_SERIALIZED_MAX + 4];
    buf[0] = cmd_id;
    buf[1] = status;
    if (len) memcpy(&buf[2], payload, len);
    cdc_io::send_frame(0x11, buf, 2 + len);
}

void on_frame(uint8_t type, const uint8_t* payload, size_t len) {
    if (type != 0x10 || len < 1) return;
    const uint8_t cmd_id = payload[0];
    const uint8_t* arg   = payload + 1;
    const size_t   alen  = len - 1;

    switch (cmd_id) {
        case CMD_PING:
            send_ack(cmd_id, STATUS_OK, nullptr, 0);
            return;

        case CMD_REZERO_NOW:
            if (!s_hooks.rezero) { send_ack(cmd_id, STATUS_ERR_UNSUPPORTED, nullptr, 0); return; }
            s_hooks.rezero();
            send_ack(cmd_id, STATUS_OK, nullptr, 0);
            return;

        case CMD_SAVE_CALIBRATION:
            if (!s_hooks.save_calibration) { send_ack(cmd_id, STATUS_ERR_UNSUPPORTED, nullptr, 0); return; }
            s_hooks.save_calibration();
            send_ack(cmd_id, STATUS_OK, nullptr, 0);
            return;

        case CMD_GET_CONFIG: {
            if (!s_hooks.get_config) { send_ack(cmd_id, STATUS_ERR_UNSUPPORTED, nullptr, 0); return; }
            Config c = s_hooks.get_config();
            uint8_t buf[CONFIG_SERIALIZED_MAX];
            size_t n = config_serialize(&c, buf, sizeof(buf));
            send_ack(cmd_id, STATUS_OK, buf, n);
            return;
        }

        case CMD_SET_CONFIG: {
            if (!s_hooks.set_config) { send_ack(cmd_id, STATUS_ERR_UNSUPPORTED, nullptr, 0); return; }
            Config c{};
            if (!config_deserialize(arg, alen, &c)) { send_ack(cmd_id, STATUS_ERR_ARG, nullptr, 0); return; }
            bool ok = s_hooks.set_config(&c);
            send_ack(cmd_id, ok ? STATUS_OK : STATUS_ERR_ARG, nullptr, 0);
            return;
        }

        case CMD_RESET_TO_DEFAULTS: {
            if (!s_hooks.set_config) { send_ack(cmd_id, STATUS_ERR_UNSUPPORTED, nullptr, 0); return; }
            Config c = config_default();
            bool ok = s_hooks.set_config(&c);
            send_ack(cmd_id, ok ? STATUS_OK : STATUS_ERR_ARG, nullptr, 0);
            return;
        }

        case CMD_GET_VERSION: {
            uint8_t buf[FW_VERSION_PAYLOAD_MAX];
            size_t n = fw_version_payload(CCTRACK_FW_MAJOR, CCTRACK_FW_MINOR,
                                          CCTRACK_FW_PATCH, CCTRACK_FW_DESCRIBE,
                                          buf, sizeof(buf));
            send_ack(cmd_id, STATUS_OK, buf, n);
            return;
        }

        case CMD_REBOOT_TO_UF2: {
            send_ack(cmd_id, STATUS_OK, nullptr, 0);
            // Pump the USB stack so the ACK is actually delivered to the host
            // before we reset — one bare delay() doesn't poll the IN endpoint,
            // so the host can miss the ACK and just see a disconnect. Mirrors the
            // reattach drain in loop() (main.cpp).
            uint32_t drain_t0 = millis();
            while (millis() - drain_t0 < 30) {
                TinyUSBDevice.task();
                delay(1);
            }
            // SAMD21 UF2 bootloader double-tap magic: 0xF01669EF at the top word
            // of SRAM (0x20007FFC on the QT Py's 32 KB SAMD21E18) makes the bootloader enter
            // UF2 mode after reset instead of running the app.
            *((uint32_t*)0x20007FFC) = 0xF01669EFul;
            NVIC_SystemReset();
            return;
        }

        default:
            send_ack(cmd_id, STATUS_ERR_UNSUPPORTED, nullptr, 0);
            return;
    }
}

}  // namespace cmd
