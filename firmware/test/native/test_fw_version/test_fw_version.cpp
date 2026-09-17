// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include <string.h>
#include "fw_version.h"

void setUp(void) {}
void tearDown(void) {}

void test_bcd_encodes_major_minor_patch(void) {
    TEST_ASSERT_EQUAL_HEX16(0x0010, fw_version_bcd(0, 1, 0));
    TEST_ASSERT_EQUAL_HEX16(0x1234, fw_version_bcd(12, 3, 4));
}

void test_bcd_clamps_out_of_range_fields(void) {
    TEST_ASSERT_EQUAL_HEX16(0x9999, fw_version_bcd(150, 12, 10));
}

void test_payload_layout(void) {
    uint8_t buf[FW_VERSION_PAYLOAD_MAX];
    size_t n = fw_version_payload(0, 1, 2, "v0.1.2-dirty", buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(4 + 12, n);
    TEST_ASSERT_EQUAL_UINT8(0, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(1, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(2, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(12, buf[3]);
    TEST_ASSERT_EQUAL_MEMORY("v0.1.2-dirty", &buf[4], 12);
}

void test_payload_truncates_long_describe(void) {
    char longdesc[64];
    memset(longdesc, 'x', sizeof(longdesc) - 1);
    longdesc[sizeof(longdesc) - 1] = '\0';
    uint8_t buf[FW_VERSION_PAYLOAD_MAX];
    size_t n = fw_version_payload(1, 0, 0, longdesc, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(FW_VERSION_PAYLOAD_MAX, n);
    TEST_ASSERT_EQUAL_UINT8(FW_VERSION_DESCRIBE_MAX, buf[3]);
}

void test_payload_null_describe_and_small_buffer(void) {
    uint8_t buf[8];
    TEST_ASSERT_EQUAL_size_t(4, fw_version_payload(1, 2, 3, nullptr, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_UINT8(0, buf[3]);
    TEST_ASSERT_EQUAL_size_t(0, fw_version_payload(1, 2, 3, "too-long", buf, sizeof(buf)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_bcd_encodes_major_minor_patch);
    RUN_TEST(test_bcd_clamps_out_of_range_fields);
    RUN_TEST(test_payload_layout);
    RUN_TEST(test_payload_truncates_long_describe);
    RUN_TEST(test_payload_null_describe_and_small_buffer);
    return UNITY_END();
}
