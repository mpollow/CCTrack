// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Encode `in[0..in_len)` into `out`. Returns the number of bytes written.
// `out` must have room for at least `in_len + 1 + (in_len / 254)` bytes.
// The output never contains 0x00 — the caller appends the delimiter.
size_t cobs_encode(const uint8_t* in, size_t in_len, uint8_t* out);

// Decode `in[0..in_len)` (which must NOT contain 0x00) into `out`, which has
// room for `out_cap` bytes. Returns decoded byte count, or -1 if the input
// is malformed OR would decode to more than `out_cap` bytes — callers whose
// output buffer is smaller than the worst-case decode MUST pass its real
// capacity, not `in_len - 1`, or this cap check does nothing to protect them.
int cobs_decode(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap);

#ifdef __cplusplus
}
#endif
