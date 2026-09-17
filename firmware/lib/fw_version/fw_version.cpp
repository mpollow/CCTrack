// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "fw_version.h"
#include <string.h>

uint16_t fw_version_bcd(uint8_t major, uint8_t minor, uint8_t patch) {
    if (major > 99) major = 99;
    if (minor > 9)  minor = 9;
    if (patch > 9)  patch = 9;
    return (uint16_t)(((major / 10) << 12) | ((major % 10) << 8)
                      | (minor << 4) | patch);
}

size_t fw_version_payload(uint8_t major, uint8_t minor, uint8_t patch,
                          const char* describe, uint8_t* out, size_t cap) {
    size_t dlen = describe ? strnlen(describe, FW_VERSION_DESCRIBE_MAX) : 0;
    if (cap < 4 + dlen) return 0;
    out[0] = major;
    out[1] = minor;
    out[2] = patch;
    out[3] = (uint8_t)dlen;
    if (dlen) memcpy(&out[4], describe, dlen);
    return 4 + dlen;
}
