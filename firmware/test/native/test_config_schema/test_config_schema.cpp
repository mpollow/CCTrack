// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include <math.h>
#include <string.h>
#include "config_schema.h"

void setUp(void) {}
void tearDown(void) {}

void test_default_config_matches_spec(void) {
    Config cfg = config_default();
    TEST_ASSERT_EQUAL_UINT8(CONFIG_SCHEMA_VERSION, cfg.schema_ver);
    TEST_ASSERT_EQUAL_UINT8(1, cfg.midi_channel);
    TEST_ASSERT_EQUAL_UINT8(OUTPUT_MODE_CC14, cfg.output_mode);
    TEST_ASSERT_EQUAL_UINT8(16, cfg.cc_yaw);
    TEST_ASSERT_EQUAL_UINT8(17, cfg.cc_pitch);
    TEST_ASSERT_EQUAL_UINT8(18, cfg.cc_roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -90.0f, cfg.mount_ypr[0]);  // sensor physically yawed -90° (correction applies +90°)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  cfg.mount_ypr[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  cfg.mount_ypr[2]);
    TEST_ASSERT_EQUAL_UINT16(100, cfg.midi_rate_hz);
    TEST_ASSERT_TRUE(cfg.conjugate_output);
    TEST_ASSERT_EQUAL_UINT8('\0', cfg.device_name[0]);
    TEST_ASSERT_TRUE(cfg.enable_midi);
    TEST_ASSERT_TRUE(cfg.stream_quat_cdc);
}

void test_serialize_then_deserialize_is_identity(void) {
    Config cfg = config_default();
    cfg.midi_channel = 7;
    cfg.cc_yaw   = 20;
    cfg.cc_pitch = 35;
    cfg.cc_roll  = 40;
    cfg.midi_rate_hz = 400;
    // 90° yaw mounting correction (degrees, ZYX).
    cfg.mount_ypr[0] = 90.0f;
    cfg.mount_ypr[1] = 0.0f;
    cfg.mount_ypr[2] = 0.0f;

    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_size_t(0u, n);

    Config back;
    bool ok = config_deserialize(buf, n, &back);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(cfg.midi_channel, back.midi_channel);
    TEST_ASSERT_EQUAL_UINT8(cfg.cc_yaw,   back.cc_yaw);
    TEST_ASSERT_EQUAL_UINT8(cfg.cc_pitch, back.cc_pitch);
    TEST_ASSERT_EQUAL_UINT8(cfg.cc_roll,  back.cc_roll);
    TEST_ASSERT_EQUAL_UINT16(cfg.midi_rate_hz, back.midi_rate_hz);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, cfg.mount_ypr[0], back.mount_ypr[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, cfg.mount_ypr[1], back.mount_ypr[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, cfg.mount_ypr[2], back.mount_ypr[2]);
    TEST_ASSERT_EQUAL(cfg.conjugate_output, back.conjugate_output);
    TEST_ASSERT_EQUAL_STRING(cfg.device_name, back.device_name);
    TEST_ASSERT_EQUAL(cfg.enable_midi,     back.enable_midi);
    TEST_ASSERT_EQUAL(cfg.stream_quat_cdc, back.stream_quat_cdc);
}

void test_flags_roundtrip_when_false(void) {
    // Both flags individually settable; not both implicitly tied to "true".
    Config cfg = config_default();
    cfg.enable_midi     = false;
    cfg.stream_quat_cdc = false;

    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(CONFIG_SERIALIZED_MAX, n);

    Config back;
    bool ok = config_deserialize(buf, n, &back);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FALSE(back.enable_midi);
    TEST_ASSERT_FALSE(back.stream_quat_cdc);
}

void test_deserialize_rejects_blob_shorter_than_v1(void) {
    // v1 is the baseline layout: a blob shorter than CONFIG_SERIALIZED_MIN is
    // malformed, not an older version — there is no version older than 1.
    Config cfg = config_default();
    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(CONFIG_SERIALIZED_MAX, n);

    Config out;
    TEST_ASSERT_FALSE(config_deserialize(buf, n - 1, &out));   // one byte short
    TEST_ASSERT_FALSE(config_deserialize(buf, 51, &out));      // pre-v1 dev-era length
}

void test_deserialize_rejects_version_zero(void) {
    // 0 is not a valid schema version (v1 is the first); a zeroed flash page
    // or all-zero blob must not parse as config.
    Config cfg = config_default();
    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    buf[0] = 0;

    Config out;
    TEST_ASSERT_FALSE(config_deserialize(buf, n, &out));
}

void test_serialize_is_little_endian(void) {
    Config cfg = config_default();
    cfg.midi_rate_hz = 0x0190;  // 400

    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_size_t(0u, n);

    // Layout: [schema_ver][midi_ch][mode][reserved][mount_ypr 12 bytes]
    //         [midi_rate_hz LE u16] ...
    const size_t off_rate = 1+1+1+1+12;
    TEST_ASSERT_EQUAL_HEX8(0x90, buf[off_rate]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[off_rate + 1]);
}

void test_deserialize_rejects_future_schema_version(void) {
    // Anything strictly newer than what we know about is rejected; older
    // versions are accepted (append-only migration).
    Config cfg = config_default();
    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    (void)n;
    buf[0] = CONFIG_SCHEMA_VERSION + 1;

    Config out;
    bool ok = config_deserialize(buf, n, &out);
    TEST_ASSERT_FALSE(ok);
}

void test_deserialize_rejects_truncated(void) {
    Config out;
    uint8_t tiny[3] = {0x01, 0x01, 0x02};
    bool ok = config_deserialize(tiny, sizeof(tiny), &out);
    TEST_ASSERT_FALSE(ok);
}

// Helper: serialize a config and assert deserialize rejects it.
static void assert_rejected(const Config& cfg) {
    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_size_t(0u, n);
    Config out;
    TEST_ASSERT_FALSE(config_deserialize(buf, n, &out));
}

void test_deserialize_rejects_out_of_range_fields(void) {
    Config cfg = config_default();
    cfg.midi_channel = 0;                 // valid range is 1..16
    assert_rejected(cfg);

    cfg = config_default();
    cfg.midi_channel = 17;
    assert_rejected(cfg);

    cfg = config_default();
    cfg.output_mode = 9;                  // > OUTPUT_MODE_QUATERNION_4CC
    assert_rejected(cfg);

    cfg = config_default();
    cfg.cc_yaw = 200;                     // > 127, would wrap when masked
    assert_rejected(cfg);

    cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_QUATERNION_4CC;
    cfg.cc_yaw = 126;                     // 126+3 = 129 > 127 in quat mode
    assert_rejected(cfg);
}

// Regression: cc14's LSB lands at base+32 (emit_cc14 in midi_mapper.cpp); a
// base > 95 makes base+32 exceed 127 and wrap when masked with &0x7F —
// cc_yaw=96 in cc14 mode would have put the yaw LSB on CC 0 (Bank Select).
void test_deserialize_rejects_cc14_lsb_wraparound(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_CC14;
    cfg.cc_yaw = 96;
    assert_rejected(cfg);
}

// Regression: quat_4cc's last pair is cc_yaw+3 (MSB) / cc_yaw+3+32 (LSB), so
// the base must stay <= 92, not the old (MSB-only) bound of 124.
void test_deserialize_rejects_quat4cc_lsb_wraparound(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_QUATERNION_4CC;
    cfg.cc_yaw = 93;
    assert_rejected(cfg);
}

void test_deserialize_accepts_cc14_lsb_boundary(void) {
    Config cfg = config_default();
    cfg.output_mode = OUTPUT_MODE_CC14;
    cfg.cc_yaw = cfg.cc_pitch = cfg.cc_roll = 95;   // 95+32 = 127, the boundary
    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    Config out;
    TEST_ASSERT_TRUE(config_deserialize(buf, n, &out));
}

void test_deserialize_accepts_in_range_boundaries(void) {
    Config cfg = config_default();
    cfg.midi_channel = 16;
    cfg.output_mode  = OUTPUT_MODE_QUATERNION_4CC;
    cfg.cc_yaw = 92;                      // 92+3+32 = 127, the highest legal base
    cfg.cc_pitch = 127;
    cfg.cc_roll  = 0;
    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    Config out;
    TEST_ASSERT_TRUE(config_deserialize(buf, n, &out));
}

// Regression: a NaN/Inf mount angle used to be accepted and persisted,
// poisoning the mount correction and rezero offset across reboots.
void test_deserialize_rejects_non_finite_or_out_of_range_mount_ypr(void) {
    const float bad[] = { NAN, INFINITY, -INFINITY, 360.5f, -360.5f };
    for (int axis = 0; axis < 3; axis++) {
        for (size_t k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
            Config cfg = config_default();
            cfg.mount_ypr[axis] = bad[k];
            assert_rejected(cfg);
        }
    }
}

void test_deserialize_accepts_mount_ypr_boundaries(void) {
    const float good[] = { 360.0f, -360.0f, 0.0f };
    for (int axis = 0; axis < 3; axis++) {
        for (size_t k = 0; k < sizeof(good) / sizeof(good[0]); k++) {
            Config cfg = config_default();
            cfg.mount_ypr[axis] = good[k];
            uint8_t buf[CONFIG_SERIALIZED_MAX];
            size_t n = config_serialize(&cfg, buf, sizeof(buf));
            Config out;
            TEST_ASSERT_TRUE(config_deserialize(buf, n, &out));
            TEST_ASSERT_EQUAL_FLOAT(good[k], out.mount_ypr[axis]);
        }
    }
}

void test_deserialize_sanitizes_control_characters_in_device_name(void) {
    // device_name feeds directly into USB string descriptors and the MIDI
    // port name; control bytes (0x01, DEL) must be scrubbed on ingestion.
    Config cfg = config_default();
    const char raw[] = "Cc\x01Track\x7Fx";
    memset(cfg.device_name, 0, DEVICE_NAME_MAX);
    memcpy(cfg.device_name, raw, sizeof(raw) - 1);  // exclude the literal's own NUL

    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    Config out;
    TEST_ASSERT_TRUE(config_deserialize(buf, n, &out));
    TEST_ASSERT_EQUAL_STRING("Cc_Track_x", out.device_name);
}

void test_serialize_stamps_running_schema_version(void) {
    // A client that round-trips an old blob must not pin the stored version:
    // serialize always emits CONFIG_SCHEMA_VERSION regardless of the field.
    Config cfg = config_default();
    cfg.schema_ver = 99;
    uint8_t buf[CONFIG_SERIALIZED_MAX];
    size_t n = config_serialize(&cfg, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_size_t(0u, n);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_SCHEMA_VERSION, buf[0]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_default_config_matches_spec);
    RUN_TEST(test_serialize_then_deserialize_is_identity);
    RUN_TEST(test_flags_roundtrip_when_false);
    RUN_TEST(test_deserialize_rejects_blob_shorter_than_v1);
    RUN_TEST(test_deserialize_rejects_version_zero);
    RUN_TEST(test_serialize_is_little_endian);
    RUN_TEST(test_deserialize_rejects_future_schema_version);
    RUN_TEST(test_deserialize_rejects_truncated);
    RUN_TEST(test_deserialize_rejects_out_of_range_fields);
    RUN_TEST(test_deserialize_rejects_cc14_lsb_wraparound);
    RUN_TEST(test_deserialize_rejects_quat4cc_lsb_wraparound);
    RUN_TEST(test_deserialize_accepts_cc14_lsb_boundary);
    RUN_TEST(test_deserialize_accepts_in_range_boundaries);
    RUN_TEST(test_deserialize_rejects_non_finite_or_out_of_range_mount_ypr);
    RUN_TEST(test_deserialize_accepts_mount_ypr_boundaries);
    RUN_TEST(test_deserialize_sanitizes_control_characters_in_device_name);
    RUN_TEST(test_serialize_stamps_running_schema_version);
    return UNITY_END();
}
