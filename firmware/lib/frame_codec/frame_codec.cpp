// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "frame_codec.h"
#include "cobs.h"
#include "crc16.h"
#include <string.h>

size_t frame_encode(uint8_t type,
                    const uint8_t* payload, size_t payload_len,
                    uint8_t* out, size_t out_cap) {
    // Layout: [type][payload...][crc_lo][crc_hi]
    const size_t body_len = 1 + payload_len + 2;
    if (body_len > FRAME_MAX_BODY) return 0;
    // COBS worst case is body_len + 1 + (body_len / 254).
    if (out_cap < body_len + 2 + (body_len / 254)) return 0;

    uint8_t body[FRAME_MAX_BODY];
    body[0] = type;
    if (payload_len) memcpy(&body[1], payload, payload_len);
    uint16_t crc = crc16_ccitt_false(body, 1 + payload_len);
    body[1 + payload_len + 0] = (uint8_t)(crc & 0xFF);
    body[1 + payload_len + 1] = (uint8_t)(crc >> 8);

    return cobs_encode(body, body_len, out);
}

int frame_decode(const uint8_t* in, size_t in_len,
                 uint8_t* type,
                 uint8_t* payload_out, size_t payload_out_cap) {
    if (in_len < 3) return FRAME_ERR_SHORT;     // need at least type+crc

    uint8_t body[FRAME_MAX_BODY];
    int body_len = cobs_decode(in, in_len, body, FRAME_MAX_BODY);
    if (body_len < 3) return FRAME_ERR_COBS;
    if ((size_t)body_len > FRAME_MAX_BODY) return FRAME_ERR_COBS;

    const size_t payload_len = (size_t)body_len - 3;
    uint16_t crc_calc = crc16_ccitt_false(body, 1 + payload_len);
    uint16_t crc_seen = (uint16_t)body[1 + payload_len]
                      | ((uint16_t)body[1 + payload_len + 1] << 8);
    if (crc_calc != crc_seen) return FRAME_ERR_CRC;

    if (payload_len > payload_out_cap) return FRAME_ERR_OVERFLOW;

    *type = body[0];
    if (payload_len) memcpy(payload_out, &body[1], payload_len);
    return (int)payload_len;
}
