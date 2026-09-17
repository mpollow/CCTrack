// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "button.h"

static constexpr uint32_t DEBOUNCE_MS  = 10;
static constexpr uint32_t LONG_HOLD_MS = 2000;

bool button_step(ButtonState* st, bool pressed_raw,
                 uint32_t now_ms, ButtonEvent* out) {
    const bool raw_changed = (pressed_raw != st->last_raw);
    if (raw_changed) {
        st->last_raw = pressed_raw;
        st->last_change_ms = now_ms;
    }

    // Asymmetric debounce: a falling edge while we were stably pressed
    // is acted on immediately. Spurious chatter never reaches stable=true,
    // so this can't fire a phantom SHORT.
    if (st->stable && !pressed_raw) {
        st->stable = false;
        if (st->long_already_emitted) return false;
        *out = BUTTON_EVENT_SHORT;
        return true;
    }

    // Press-side debounce: only update `stable` to true after the raw
    // input has been steadily HIGH for DEBOUNCE_MS.
    if (!raw_changed) {
        const bool stable_now = (now_ms - st->last_change_ms) >= DEBOUNCE_MS
                              ? pressed_raw : st->stable;
        if (stable_now != st->stable) {
            st->stable = stable_now;
            if (stable_now) {
                st->press_started_ms = now_ms;
                st->long_already_emitted = false;
            }
            return false;
        }
    }

    // Held — emit LONG once when threshold crossed.
    if (st->stable && !st->long_already_emitted
        && (now_ms - st->press_started_ms) >= LONG_HOLD_MS) {
        st->long_already_emitted = true;
        *out = BUTTON_EVENT_LONG;
        return true;
    }
    return false;
}
