// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ButtonEvent {
    BUTTON_EVENT_NONE  = 0,
    BUTTON_EVENT_SHORT = 1,
    BUTTON_EVENT_LONG  = 2,
};

struct ButtonState {
    bool     last_raw;             // raw input as last seen
    bool     stable;               // debounced state
    uint32_t last_change_ms;       // time `last_raw` changed
    uint32_t press_started_ms;     // when `stable` last became true
    bool     long_already_emitted; // suppresses SHORT on release after LONG
};

bool button_step(ButtonState* st, bool pressed_raw,
                 uint32_t now_ms, ButtonEvent* out);

#ifdef __cplusplus
}
#endif
