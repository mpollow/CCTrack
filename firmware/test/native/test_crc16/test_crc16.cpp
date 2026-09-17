// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include "crc16.h"

void setUp(void) {}
void tearDown(void) {}

// CRC-16/CCITT-FALSE reference vector from crccalc.com
void test_crc16_check_vector_123456789(void) {
    const uint8_t data[] = "123456789";
    uint16_t crc = crc16_ccitt_false(data, 9);
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc);
}

void test_crc16_empty_input_returns_init(void) {
    uint16_t crc = crc16_ccitt_false(nullptr, 0);
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc);
}

void test_crc16_single_zero_byte(void) {
    const uint8_t data[] = {0x00};
    uint16_t crc = crc16_ccitt_false(data, 1);
    // Verified externally: CRC of single 0x00 byte under CCITT-FALSE = 0xE1F0
    TEST_ASSERT_EQUAL_HEX16(0xE1F0, crc);
}

void test_crc16_streaming_matches_oneshot(void) {
    const uint8_t data[] = "Hello, world!";
    uint16_t oneshot = crc16_ccitt_false(data, 13);

    uint16_t streamed = crc16_init();
    for (size_t i = 0; i < 13; i++) {
        streamed = crc16_update(streamed, data[i]);
    }
    TEST_ASSERT_EQUAL_HEX16(oneshot, streamed);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_check_vector_123456789);
    RUN_TEST(test_crc16_empty_input_returns_init);
    RUN_TEST(test_crc16_single_zero_byte);
    RUN_TEST(test_crc16_streaming_matches_oneshot);
    return UNITY_END();
}
