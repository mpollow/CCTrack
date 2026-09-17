// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <Arduino.h>

namespace pin {
#if defined(ARDUINO_QTPY_M0)
    // Adafruit QT Py SAMD21 — STEMMA QT connector provides I2C.
    // Wire.begin() uses PIN_WIRE_SDA/SCL automatically; these are documentation only.
    constexpr uint8_t SDA       = PIN_WIRE_SDA;
    constexpr uint8_t SCL       = PIN_WIRE_SCL;
    constexpr uint8_t BUTTON    = A1;     // optional button: A1 to GND (internal pull-up; unconnected = not pressed)
    // No LED_USER — led_status.cpp drives PIN_NEOPIXEL directly.
#else
    #error "Unsupported board: this build targets the Adafruit QT Py SAMD21 (ARDUINO_QTPY_M0)"
#endif
}
