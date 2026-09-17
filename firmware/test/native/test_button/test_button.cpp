// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include "button.h"

void setUp(void) {}
void tearDown(void) {}

void test_no_press_no_event(void) {
    ButtonState bs = {};
    ButtonEvent ev;
    bool any = button_step(&bs, /*pressed=*/false, /*now_ms=*/100, &ev);
    TEST_ASSERT_FALSE(any);
}

void test_short_press_emits_short_on_release(void) {
    ButtonState bs = {};
    ButtonEvent ev;
    // press
    button_step(&bs, true,  100, &ev);
    button_step(&bs, true,  150, &ev);     // past debounce
    // still pressed at 600
    button_step(&bs, true,  600, &ev);
    // release at 700
    bool any = button_step(&bs, false, 700, &ev);
    TEST_ASSERT_TRUE(any);
    TEST_ASSERT_EQUAL_INT(BUTTON_EVENT_SHORT, ev);
}

void test_long_press_emits_long_at_threshold_no_event_on_release(void) {
    ButtonState bs = {};
    ButtonEvent ev;
    button_step(&bs, true, 100, &ev);
    button_step(&bs, true, 150, &ev);
    // We expect a LONG event the first tick we observe held >= 2000 ms.
    bool got = false;
    for (uint32_t t = 200; t <= 2200; t += 100) {
        if (button_step(&bs, true, 100 + t, &ev)) {
            TEST_ASSERT_EQUAL_INT(BUTTON_EVENT_LONG, ev);
            got = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(got);
    // Subsequent release should NOT also emit a SHORT.
    bool released = button_step(&bs, false, 5000, &ev);
    TEST_ASSERT_FALSE(released);
}

void test_debounce_ignores_chatter(void) {
    ButtonState bs = {};
    ButtonEvent ev;
    // bounces faster than debounce window (10 ms)
    button_step(&bs, true,  100, &ev);
    button_step(&bs, false, 102, &ev);
    button_step(&bs, true,  104, &ev);
    button_step(&bs, false, 106, &ev);
    bool any = button_step(&bs, false, 200, &ev);
    TEST_ASSERT_FALSE(any);  // never crossed debounce as a real press
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_no_press_no_event);
    RUN_TEST(test_short_press_emits_short_on_release);
    RUN_TEST(test_long_press_emits_long_at_threshold_no_event_on_release);
    RUN_TEST(test_debounce_ignores_chatter);
    return UNITY_END();
}
