// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Largest decoded body (type + payload + crc16) the codec will accept.
// Update this in lockstep with cdc_io and the host scripts if the wire
// protocol ever grows a bigger payload.
#define FRAME_MAX_BODY 256

// frame_decode return codes (negative).
enum {
    FRAME_ERR_SHORT    = -1,    // input shorter than minimum frame
    FRAME_ERR_COBS     = -2,    // COBS decode failed
    FRAME_ERR_CRC      = -3,    // CRC mismatch
    FRAME_ERR_OVERFLOW = -4,    // payload larger than caller's buffer
};

// Encode a frame into `out`. Returns bytes written (no trailing 0x00).
// `out_cap` must be at least  payload_len + 3 + COBS overhead (worst case
// payload_len + 3 + ceil((payload_len + 3) / 254) + 1).
size_t frame_encode(uint8_t type,
                    const uint8_t* payload, size_t payload_len,
                    uint8_t* out, size_t out_cap);

// Decode a frame (already stripped of 0x00 delimiters) into payload buffer.
// Returns payload length on success, negative on error.
int frame_decode(const uint8_t* in, size_t in_len,
                 uint8_t* type,
                 uint8_t* payload_out, size_t payload_out_cap);

#ifdef __cplusplus
}
#endif
