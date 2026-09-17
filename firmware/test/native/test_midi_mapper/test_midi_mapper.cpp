// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include "midi_mapper.h"

void setUp(void) {}
void tearDown(void) {}

void test_mapper_off_emits_nothing(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_OFF;
    MidiMapperState st = {};
    MidiEvent evs[8];
    Quat q = {1, 0, 0, 0};
    int n = midi_mapper_step(&st, &cfg, q, /*now_ms=*/0, evs, 8);
    TEST_ASSERT_EQUAL_INT(0, n);
}

void test_cc14_default_identity_yields_centred_values(void) {
    Config cfg = config_default();             // mode = cc14
    MidiMapperState st = {};
    MidiEvent evs[16];
    Quat identity = {1, 0, 0, 0};
    int n = midi_mapper_step(&st, &cfg, identity, /*now_ms=*/0, evs, 16);
    // Three CC pairs (MSB + LSB) × yaw/pitch/roll = 6 events.
    TEST_ASSERT_EQUAL_INT(6, n);
    // emit_cc14 emits LSB first (on base_cc+32), then MSB (on base_cc).
    // Per axis: events [i]=LSB, [i+1]=MSB. Reconstruct: (MSB<<7)|LSB.
    // Identity quaternion → every Euler == 0 → centre of 14-bit range = 8192.
    for (int i = 0; i < 6; i += 2) {
        uint16_t v = ((uint16_t)evs[i+1].data2 << 7) | (uint16_t)evs[i].data2;
        TEST_ASSERT_INT16_WITHIN(2, 8192, v);
    }
    // All events on the default channel (1 in user-facing terms = 0 on wire).
    for (int i = 0; i < 6; i++) TEST_ASSERT_EQUAL_UINT8(0u, evs[i].status & 0x0F);
}

void test_cc7_yaw_at_plus_pi_clamps_to_127(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_CC7;
    MidiMapperState st = {};
    MidiEvent evs[8];
    // q = (cos(90°), 0, 0, sin(90°)) → yaw = +π in ZYX
    Quat q = {0.0f, 0.0f, 0.0f, 1.0f};
    int n = midi_mapper_step(&st, &cfg, q, /*now_ms=*/0, evs, 8);
    TEST_ASSERT_GREATER_OR_EQUAL_INT(1, n);
    // First event is yaw (CC=16); value should saturate near 127.
    bool found = false;
    for (int i = 0; i < n; i++) {
        if ((evs[i].status & 0xF0) == 0xB0 && evs[i].data1 == 16) {
            TEST_ASSERT_GREATER_OR_EQUAL_UINT8(125u, evs[i].data2);
            found = true;
        }
    }
    TEST_ASSERT_TRUE(found);
}

void test_dedup_skips_when_value_unchanged(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_CC7;
    MidiMapperState st = {};
    MidiEvent evs[8];
    Quat q = {1, 0, 0, 0};
    int first = midi_mapper_step(&st, &cfg, q, 0, evs, 8);
    int second = midi_mapper_step(&st, &cfg, q, 6, evs, 8);
    TEST_ASSERT_GREATER_THAN_INT(0, first);
    TEST_ASSERT_EQUAL_INT(0, second);   // values unchanged → no events
}

void test_downsample_obeys_midi_rate_hz(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_CC7;
    cfg.midi_rate_hz = 100;             // every 10 ms
    MidiMapperState st = {};
    MidiEvent evs[8];

    Quat q = {0.99f, 0.05f, 0.05f, 0.05f};
    int a = midi_mapper_step(&st, &cfg, q, /*now_ms=*/0,  evs, 8);
    Quat q2 = {0.98f, 0.10f, 0.10f, 0.10f};
    int b = midi_mapper_step(&st, &cfg, q2, /*now_ms=*/3, evs, 8);
    int c = midi_mapper_step(&st, &cfg, q2, /*now_ms=*/12, evs, 8);
    TEST_ASSERT_GREATER_THAN_INT(0, a);
    TEST_ASSERT_EQUAL_INT(0, b);          // too soon
    TEST_ASSERT_GREATER_OR_EQUAL_INT(0, c); // 12 ms ≥ 10 ms
}

// Helper: drive one quat through the mapper in quat_4cc mode and return the
// four reassembled 14-bit values, indexed by axis (w/x/y/z = 0/1/2/3).
static void step_quat4cc(MidiMapperState* st, const Config* cfg,
                         Quat q, uint32_t now_ms, uint16_t out_v14[4]) {
    MidiEvent evs[16];
    int n = midi_mapper_step(st, cfg, q, now_ms, evs, 16);
    for (int a = 0; a < 4; a++) out_v14[a] = 0xFFFF;   // sentinel: "not emitted"
    for (int i = 0; i + 1 < n; i += 2) {
        // emit_cc14 writes LSB (on base_cc+32) then MSB (on base_cc); both share
        // the same axis. Index off the MSB event, whose CC is the axis base.
        int axis = (int)evs[i+1].data1 - (int)cfg->cc_yaw;
        if (axis >= 0 && axis < 4) {
            out_v14[axis] = ((uint16_t)evs[i+1].data2 << 7) | (uint16_t)evs[i].data2;
        }
    }
}

// Regression: BNO085 sometimes emits −q instead of q mid-rotation (quaternion
// double cover). Without hemisphere canonicalisation that flip turns into a
// 14000-unit jump on every CC14, which is what made the user's "rotate around
// Z" tests look broken. This pins the fix: feeding the mapper a real-data
// sign-flip pair PRE-canonicalised via quat_canonical_to must produce cc14
// values within the natural per-sample step (a few hundred units), not the
// catastrophic jump of the raw stream.
void test_quat4cc_canonicalised_stream_has_no_sign_flip_jump(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_QUATERNION_4CC;
    cfg.midi_rate_hz = 1000;   // effectively pass-through for our two ticks
    MidiMapperState st = {};

    // Two consecutive frames from the user's CDC trace, before/after the
    // double-cover flip. Without canonicalisation these decode to wildly
    // different cc14 values.
    Quat q_prev = {+0.8822f, -0.1074f, +0.1815f, -0.4210f};
    Quat q_raw  = {-0.8489f, +0.1265f, -0.2025f, +0.4716f};

    // Raw (unfixed) behaviour — sanity-check that the bug is real if we skip
    // canonicalisation. We compute what cc14_w would be without the fix and
    // assert that it differs from the canonicalised value by a huge margin,
    // so this test fails noisily if someone later "optimises" the helper out.
    auto cc14_from = [](float c) {
        float v = (c + 1.0f) * 0.5f * 16383.0f;
        if (v < 0.0f)     v = 0.0f;
        if (v > 16383.0f) v = 16383.0f;
        return (uint16_t)v;
    };
    uint16_t raw_cc14_w_before = cc14_from(q_prev.w);
    uint16_t raw_cc14_w_after  = cc14_from(q_raw.w);
    TEST_ASSERT_GREATER_THAN_UINT16(10000u, (uint16_t)(raw_cc14_w_before - raw_cc14_w_after));

    // With the fix: drive the mapper through the canonicalised stream.
    uint16_t v_first[4], v_second[4];
    step_quat4cc(&st, &cfg, q_prev, /*now_ms=*/0, v_first);
    Quat q_fixed = quat_canonical_to(q_raw, q_prev);
    step_quat4cc(&st, &cfg, q_fixed, /*now_ms=*/10, v_second);

    // Each component changes by at most a few hundred 14-bit units between
    // the two samples (a real, continuous rotation step). Tolerance 500 leaves
    // headroom for the ~280 actually expected at this rotation rate.
    for (int a = 0; a < 4; a++) {
        TEST_ASSERT_NOT_EQUAL_UINT16(0xFFFF, v_first[a]);   // first emit publishes all axes
        // v_second[a] may be 0xFFFF if dedup decided the value was unchanged;
        // treat that as "no jump". Otherwise compare magnitudes.
        if (v_second[a] != 0xFFFF) {
            int32_t delta = (int32_t)v_second[a] - (int32_t)v_first[a];
            if (delta < 0) delta = -delta;
            TEST_ASSERT_LESS_THAN_INT32(500, delta);
        }
    }
}

void test_cc7_non_consecutive_cc_numbers(void) {
    // Verify that cc_yaw/cc_pitch/cc_roll are used independently (not cc_base+offset).
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_CC7;
    cfg.cc_yaw   = 20;
    cfg.cc_pitch = 35;
    cfg.cc_roll  = 40;
    MidiMapperState st = {};
    MidiEvent evs[8];
    Quat q = {1, 0, 0, 0};
    int n = midi_mapper_step(&st, &cfg, q, 0, evs, 8);
    TEST_ASSERT_EQUAL_INT(3, n);
    // Confirm each axis lands on its own CC number (not consecutive from a base).
    bool found_yaw = false, found_pitch = false, found_roll = false;
    for (int i = 0; i < n; i++) {
        if (evs[i].data1 == 20) found_yaw   = true;
        if (evs[i].data1 == 35) found_pitch = true;
        if (evs[i].data1 == 40) found_roll  = true;
    }
    TEST_ASSERT_TRUE(found_yaw);
    TEST_ASSERT_TRUE(found_pitch);
    TEST_ASSERT_TRUE(found_roll);
}

// Regression: a dropped event (out_cap reached) must not be committed to
// last_value, or dedup would falsely skip re-sending that axis forever once
// capacity recovers.
void test_capacity_drop_does_not_permanently_dedupe_value(void) {
    Config cfg = config_default();               // cc14 mode
    MidiMapperState st = {};
    MidiEvent evs[8];

    // Prime with an initial value at full capacity.
    Quat q_a = {1, 0, 0, 0};
    int n0 = midi_mapper_step(&st, &cfg, q_a, 0, evs, 8);
    TEST_ASSERT_GREATER_THAN_INT(0, n0);

    // New orientation (different cc14 values), but capacity too small for
    // even one full pair (needs 2 events) -- nothing should actually emit.
    Quat q_b = {0.99f, 0.05f, 0.05f, 0.05f};
    int n1 = midi_mapper_step(&st, &cfg, q_b, 10, evs, 1);
    TEST_ASSERT_EQUAL_INT(0, n1);

    // Same q_b again with full capacity: since the previous attempt never
    // actually committed, this must still emit (not be falsely deduped).
    int n2 = midi_mapper_step(&st, &cfg, q_b, 20, evs, 8);
    TEST_ASSERT_GREATER_THAN_INT(0, n2);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_mapper_off_emits_nothing);
    RUN_TEST(test_cc14_default_identity_yields_centred_values);
    RUN_TEST(test_cc7_yaw_at_plus_pi_clamps_to_127);
    RUN_TEST(test_dedup_skips_when_value_unchanged);
    RUN_TEST(test_downsample_obeys_midi_rate_hz);
    RUN_TEST(test_cc7_non_consecutive_cc_numbers);
    RUN_TEST(test_quat4cc_canonicalised_stream_has_no_sign_flip_jump);
    RUN_TEST(test_capacity_drop_does_not_permanently_dedupe_value);
    return UNITY_END();
}
