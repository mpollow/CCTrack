// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "cobs.h"

size_t cobs_encode(const uint8_t* in, size_t in_len, uint8_t* out) {
    size_t out_idx = 1;          // reserve out[0] for first overhead byte
    size_t code_idx = 0;         // index of the most recent overhead byte
    uint8_t code = 0x01;         // distance to next zero (incl. self) so far

    for (size_t i = 0; i < in_len; i++) {
        if (in[i] == 0x00) {
            out[code_idx] = code;
            code_idx = out_idx++;
            code = 0x01;
        } else {
            out[out_idx++] = in[i];
            code++;
            if (code == 0xFF) {
                out[code_idx] = code;
                code_idx = out_idx++;
                code = 0x01;
            }
        }
    }
    out[code_idx] = code;
    return out_idx;
}

int cobs_decode(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap) {
    if (in_len == 0) return -1;
    size_t i = 0;
    size_t out_idx = 0;
    while (i < in_len) {
        uint8_t code = in[i];
        if (code == 0x00) return -1;            // never legal mid-stream
        if (i + code > in_len) return -1;       // truncated run
        for (size_t j = 1; j < code; j++) {
            if (out_idx >= out_cap) return -1;  // would overflow caller's buffer
            out[out_idx++] = in[i + j];
        }
        i += code;
        if (code != 0xFF && i < in_len) {
            if (out_idx >= out_cap) return -1;  // would overflow caller's buffer
            out[out_idx++] = 0x00;
        }
    }
    return (int)out_idx;
}
