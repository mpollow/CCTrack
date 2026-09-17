// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include <string.h>
#include "frame_codec.h"

void setUp(void) {}
void tearDown(void) {}

void test_frame_encode_round_trip(void) {
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t out[64];
    size_t n = frame_encode(/*type=*/0x01, payload, 4, out, sizeof(out));

    TEST_ASSERT_GREATER_THAN_size_t(0u, n);
    // Output must not contain 0x00 (delimiter is added by the writer).
    for (size_t i = 0; i < n; i++) {
        TEST_ASSERT_NOT_EQUAL_UINT8(0x00, out[i]);
    }

    uint8_t type;
    uint8_t pl_buf[64];
    int pl_len = frame_decode(out, n, &type, pl_buf, sizeof(pl_buf));
    TEST_ASSERT_EQUAL_INT(4, pl_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, type);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, pl_buf, 4);
}

void test_frame_encode_empty_payload(void) {
    uint8_t out[16];
    size_t n = frame_encode(0x02, nullptr, 0, out, sizeof(out));
    TEST_ASSERT_GREATER_THAN_size_t(0u, n);

    uint8_t type;
    uint8_t pl[16];
    int pl_len = frame_decode(out, n, &type, pl, sizeof(pl));
    TEST_ASSERT_EQUAL_INT(0, pl_len);
    TEST_ASSERT_EQUAL_HEX8(0x02, type);
}

void test_frame_decode_rejects_corrupted_crc(void) {
    const uint8_t payload[] = {1, 2, 3};
    uint8_t buf[32];
    size_t n = frame_encode(0x10, payload, 3, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_size_t(2u, n);
    buf[n - 1] ^= 0x01;  // flip a bit in the CRC region

    uint8_t type;
    uint8_t pl[32];
    int pl_len = frame_decode(buf, n, &type, pl, sizeof(pl));
    TEST_ASSERT_EQUAL_INT(FRAME_ERR_CRC, pl_len);
}

void test_frame_decode_rejects_undersized_input(void) {
    const uint8_t too_short[] = {0x01};
    uint8_t type, pl[16];
    int n = frame_decode(too_short, 1, &type, pl, sizeof(pl));
    TEST_ASSERT_EQUAL_INT(FRAME_ERR_SHORT, n);
}

void test_frame_decode_rejects_payload_too_large(void) {
    uint8_t payload[200];
    memset(payload, 0xAB, sizeof(payload));
    uint8_t buf[256];
    size_t n = frame_encode(0x01, payload, sizeof(payload), buf, sizeof(buf));

    uint8_t type;
    uint8_t small[8];
    int pl_len = frame_decode(buf, n, &type, small, sizeof(small));
    TEST_ASSERT_EQUAL_INT(FRAME_ERR_OVERFLOW, pl_len);
}

// Regression test for the confirmed stack-buffer-overflow blocker
// (PROJECT_REVIEW.md): cdc_io::poll_rx accumulates up to RX_BUF_MAX=263
// delimiter-free bytes before calling frame_decode. A 263-byte COBS block
// built from maximal 0xFF runs decodes to ~261 bytes — more than
// FRAME_MAX_BODY(256) — which used to overflow frame_decode's stack-local
// `body` buffer before the length was ever checked. This builds exactly that
// shape (255-byte block of code=0xFF + 254 non-zero data bytes, then an
// 8-byte tail block of code=0x08 + 7 non-zero data bytes; no 0x00 anywhere)
// and asserts frame_decode rejects it cleanly instead of overflowing.
void test_frame_decode_rejects_max_length_delimiter_free_block(void) {
    uint8_t in[263];
    size_t idx = 0;
    in[idx++] = 0xFF;
    for (int i = 0; i < 254; i++) in[idx++] = 0x41;  // non-zero payload bytes
    in[idx++] = 0x08;
    for (int i = 0; i < 7; i++) in[idx++] = 0x42;    // non-zero payload bytes
    TEST_ASSERT_EQUAL_size_t(sizeof(in), idx);

    uint8_t type;
    uint8_t payload_out[512];
    int n = frame_decode(in, sizeof(in), &type, payload_out, sizeof(payload_out));
    TEST_ASSERT_LESS_THAN_INT(0, n);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_frame_encode_round_trip);
    RUN_TEST(test_frame_encode_empty_payload);
    RUN_TEST(test_frame_decode_rejects_corrupted_crc);
    RUN_TEST(test_frame_decode_rejects_undersized_input);
    RUN_TEST(test_frame_decode_rejects_payload_too_large);
    RUN_TEST(test_frame_decode_rejects_max_length_delimiter_free_block);
    return UNITY_END();
}
