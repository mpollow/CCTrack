// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stddef.h>
#include <stdint.h>

// Injected by tools/firmware_version.py from `git describe` at build time.
// The defaults only apply to builds that bypass that script (e.g. native).
#ifndef CCTRACK_FW_MAJOR
#define CCTRACK_FW_MAJOR 0
#endif
#ifndef CCTRACK_FW_MINOR
#define CCTRACK_FW_MINOR 0
#endif
#ifndef CCTRACK_FW_PATCH
#define CCTRACK_FW_PATCH 0
#endif
#ifndef CCTRACK_FW_DESCRIBE
#define CCTRACK_FW_DESCRIBE "unknown"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Longest describe string carried in the GET_VERSION payload.
#define FW_VERSION_DESCRIBE_MAX 40
// [major][minor][patch][describe_len][describe...]
#define FW_VERSION_PAYLOAD_MAX  (4 + FW_VERSION_DESCRIBE_MAX)

// USB bcdDevice encoding 0xJJMN: major as two BCD digits (clamped to 99),
// minor and patch as one BCD digit each (clamped to 9).
uint16_t fw_version_bcd(uint8_t major, uint8_t minor, uint8_t patch);

// Writes the GET_VERSION ACK payload. `describe` may be NULL and is truncated
// to FW_VERSION_DESCRIBE_MAX bytes. Returns bytes written, or 0 if `cap` is
// too small for the result.
size_t fw_version_payload(uint8_t major, uint8_t minor, uint8_t patch,
                          const char* describe, uint8_t* out, size_t cap);

#ifdef __cplusplus
}
#endif
