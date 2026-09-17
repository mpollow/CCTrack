// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include <string.h>
#include "cobs.h"

void setUp(void) {}
void tearDown(void) {}

// Wikipedia reference vectors (decoded → encoded, sans trailing 0x00):
// {}            -> {0x01}
// {0x00}        -> {0x01, 0x01}
// {0x11,0x22}   -> {0x03, 0x11, 0x22}
// {0x00,0x00}   -> {0x01, 0x01, 0x01}
// {0x11,0x00,0x33} -> {0x02, 0x11, 0x02, 0x33}

void test_cobs_encode_empty(void) {
    uint8_t out[8];
    size_t n = cobs_encode(nullptr, 0, out);
    TEST_ASSERT_EQUAL_size_t(1u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
}

void test_cobs_encode_single_zero(void) {
    const uint8_t in[] = {0x00};
    uint8_t out[8];
    size_t n = cobs_encode(in, 1, out);
    TEST_ASSERT_EQUAL_size_t(2u, n);
    const uint8_t expect[] = {0x01, 0x01};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, out, 2);
}

void test_cobs_encode_two_nonzero(void) {
    const uint8_t in[] = {0x11, 0x22};
    uint8_t out[8];
    size_t n = cobs_encode(in, 2, out);
    TEST_ASSERT_EQUAL_size_t(3u, n);
    const uint8_t expect[] = {0x03, 0x11, 0x22};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, out, 3);
}

void test_cobs_encode_zero_in_middle(void) {
    const uint8_t in[] = {0x11, 0x00, 0x33};
    uint8_t out[8];
    size_t n = cobs_encode(in, 3, out);
    TEST_ASSERT_EQUAL_size_t(4u, n);
    const uint8_t expect[] = {0x02, 0x11, 0x02, 0x33};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, out, 4);
}

void test_cobs_decode_round_trip_arbitrary(void) {
    uint8_t in[300];
    for (size_t i = 0; i < sizeof(in); i++) in[i] = (uint8_t)i; // many 0x00s
    uint8_t encoded[400];
    size_t enc_n = cobs_encode(in, sizeof(in), encoded);
    // Encoded output must contain no zeros.
    for (size_t i = 0; i < enc_n; i++) {
        TEST_ASSERT_NOT_EQUAL_UINT8(0x00, encoded[i]);
    }
    uint8_t decoded[400];
    int dec_n = cobs_decode(encoded, enc_n, decoded, sizeof(decoded));
    TEST_ASSERT_EQUAL_INT((int)sizeof(in), dec_n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, decoded, sizeof(in));
}

void test_cobs_decode_rejects_zero_in_input(void) {
    const uint8_t bad[] = {0x02, 0x11, 0x00};
    uint8_t out[8];
    int n = cobs_decode(bad, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(-1, n);
}

void test_cobs_decode_rejects_truncated_run(void) {
    // First byte says "next zero is 4 bytes away" but buffer is only 2 bytes.
    const uint8_t bad[] = {0x04, 0x11};
    uint8_t out[8];
    int n = cobs_decode(bad, 2, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(-1, n);
}

void test_cobs_decode_rejects_output_overflow(void) {
    // {0x11,0x22,0x33} -> {0x04, 0x11, 0x22, 0x33} (from cobs_encode).
    const uint8_t in[] = {0x11, 0x22, 0x33};
    uint8_t encoded[8];
    size_t enc_n = cobs_encode(in, sizeof(in), encoded);

    uint8_t out[2];  // deliberately smaller than the 3-byte decode
    int n = cobs_decode(encoded, enc_n, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(-1, n);
}

void test_cobs_decode_exact_capacity_succeeds(void) {
    const uint8_t in[] = {0x11, 0x22, 0x33};
    uint8_t encoded[8];
    size_t enc_n = cobs_encode(in, sizeof(in), encoded);

    uint8_t out[3];  // exactly the decoded length — must not be rejected
    int n = cobs_decode(encoded, enc_n, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT((int)sizeof(in), n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, sizeof(in));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_cobs_encode_empty);
    RUN_TEST(test_cobs_encode_single_zero);
    RUN_TEST(test_cobs_encode_two_nonzero);
    RUN_TEST(test_cobs_encode_zero_in_middle);
    RUN_TEST(test_cobs_decode_round_trip_arbitrary);
    RUN_TEST(test_cobs_decode_rejects_zero_in_input);
    RUN_TEST(test_cobs_decode_rejects_truncated_run);
    RUN_TEST(test_cobs_decode_rejects_output_overflow);
    RUN_TEST(test_cobs_decode_exact_capacity_succeeds);
    return UNITY_END();
}
