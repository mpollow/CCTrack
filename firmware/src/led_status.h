// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stdint.h>

namespace led {

enum State {
    STATE_OK            = 0,
    STATE_CALIBRATING   = 1,   // BNO accuracy below threshold
    STATE_NO_HOST       = 2,   // USB device not enumerated
    STATE_ERROR         = 3,   // fatal init error
    STATE_REZERO_FLASH  = 4,   // transient confirmation after rezero
};

void begin();
void set(State s);
void update(uint32_t now_ms);    // call once per loop iteration
}
