// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflection, xorout=0).
// One-shot: returns final CRC over `data[0..len)`.
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

// Streaming API. Use crc16_init() to start, then crc16_update() per byte.
static inline uint16_t crc16_init(void) { return 0xFFFF; }
uint16_t crc16_update(uint16_t crc, uint8_t byte);

#ifdef __cplusplus
}
#endif
