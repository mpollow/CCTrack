// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "led_status.h"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "pinmap.h"   // rejects unsupported boards at compile time

namespace {
Adafruit_NeoPixel s_pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
led::State s_state = led::STATE_OK;
}

namespace led {

void begin() {
    s_pixel.begin();
    s_pixel.setBrightness(40);
    s_pixel.show();
}

void set(State s) { s_state = s; }

void update(uint32_t now_ms) {
    switch (s_state) {
        case STATE_OK: {
            bool on = (now_ms % 50) < 4;
            s_pixel.setPixelColor(0, on ? s_pixel.Color(0, 80, 0) : 0);
            break;
        }
        case STATE_CALIBRATING: {
            uint32_t tri = now_ms % 2000;
            uint8_t v = (uint8_t)((tri < 1000 ? tri : 2000 - tri) * 255 / 1000);
            s_pixel.setPixelColor(0, s_pixel.Color(0, 0, v));
            break;
        }
        case STATE_NO_HOST: {
            bool on = (now_ms % 1000) < 200;
            s_pixel.setPixelColor(0, on ? s_pixel.Color(200, 80, 0) : 0);
            break;
        }
        case STATE_ERROR: {
            bool on = (now_ms / 50) & 1;
            s_pixel.setPixelColor(0, on ? s_pixel.Color(200, 0, 0) : 0);
            break;
        }
        case STATE_REZERO_FLASH: {
            s_pixel.setPixelColor(0, s_pixel.Color(200, 200, 200));
            break;
        }
    }
    s_pixel.show();
}

}  // namespace led
