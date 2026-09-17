// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "bno085_driver.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

namespace {

Adafruit_BNO08x s_imu(/*reset_pin=*/-1);   // BNO085's own RST line tied high externally
sh2_SensorValue_t s_value;
bool s_running = false;
uint32_t s_last_sample_ms = 0;
constexpr uint32_t kPollStallTimeoutMs = 500;  // re-enable GRV if no samples for this long

void enable_game_rotation_vector() {
    // 2500 µs interval = 400 Hz.
    s_imu.enableReport(SH2_GAME_ROTATION_VECTOR, 2500);
}

}  // namespace

namespace bno085 {

bool begin() {
    if (s_imu.begin_I2C(0x4A, &Wire)) {
        // ok
    } else if (s_imu.begin_I2C(0x4B, &Wire)) {
        // ok (alt addr; reserved for multi-sensor)
    } else {
        return false;
    }
    // Fast mode must be set *after* begin_I2C(): it calls Wire.begin(), which
    // resets the SAMD21 bus to 100 kHz. At 100 kHz each SHTP read takes ~3.8 ms,
    // longer than the 2.5 ms report period, so ~46% of the 400 Hz reports were
    // lost and the stream ran at ~217 Hz (measured via report sequence gaps).
    Wire.setClock(400000);

    enable_game_rotation_vector();
    sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO);   // always-on dynamic calibration
    s_running = true;
    s_last_sample_ms = millis();
    return true;
}

bool poll(Sample* out) {
    if (!s_running) return false;

    uint32_t now = millis();
    if (s_imu.wasReset()) {
        // Sensor came back — re-enable our report.
        enable_game_rotation_vector();
        s_last_sample_ms = now;
    }
    // Silent-stall watchdog: wasReset() catches detected resets, but SH2
    // subscriptions can also drop without a reset event (USB re-plug glitches,
    // bus contention). If we go too long without a sample, re-subscribe.
    if (now - s_last_sample_ms > kPollStallTimeoutMs) {
        enable_game_rotation_vector();
        s_last_sample_ms = now;
    }

    if (!s_imu.getSensorEvent(&s_value)) return false;
    if (s_value.sensorId != SH2_GAME_ROTATION_VECTOR) return false;

    out->q.w = s_value.un.gameRotationVector.real;
    out->q.x = s_value.un.gameRotationVector.i;
    out->q.y = s_value.un.gameRotationVector.j;
    out->q.z = s_value.un.gameRotationVector.k;
    out->accuracy = s_value.status & 0x03;
    s_last_sample_ms = now;
    return true;
}

}  // namespace bno085

namespace bno085 {

bool cal_save() {
    int rc = sh2_saveDcdNow();
    return rc == SH2_OK;   // DCD stays enabled; learning continues after save
}

}
