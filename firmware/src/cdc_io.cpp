// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "cdc_io.h"
#include "frame_codec.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <string.h>

namespace cdc_io {

namespace {
// Worst-case COBS-encoded body plus a delimiter byte plus slack — derived
// from FRAME_MAX_BODY so a protocol bump only changes one place.
constexpr size_t TX_BUF_MAX = FRAME_MAX_BODY + 2 + (FRAME_MAX_BODY / 254) + 4;
constexpr size_t RX_BUF_MAX = TX_BUF_MAX;
uint8_t s_rx[RX_BUF_MAX];
size_t  s_rx_len = 0;
bool    s_rx_skip = false;   // dropping bytes until the next 0x00 delimiter
FrameRxCallback s_cb = nullptr;
void* s_cb_user = nullptr;
}

bool send_frame(uint8_t type, const uint8_t* payload, size_t payload_len) {
    uint8_t out[TX_BUF_MAX];
    size_t n = frame_encode(type, payload, payload_len, out, TX_BUF_MAX - 1);
    if (n == 0) return false;
    out[n++] = 0x00;
    // Use tud_cdc_write directly: Serial.write() checks DTR (tud_cdc_n_connected)
    // but we want to stream regardless of whether the host has opened the port.
    // Guard against partial writes: a truncated COBS frame (missing the 0x00
    // delimiter) merges with the next frame and causes CRC errors on the host.
    // If no host is ever connected, TinyUSB's TX FIFO fills, write_available
    // returns < n, and frames silently drop — intentional, the device runs
    // as a standalone MIDI source in that case.
    if (tud_cdc_write_available() < n) return false;
    tud_cdc_write(out, n);
    tud_cdc_write_flush();
    return true;
}

bool send_quat(float qw, float qx, float qy, float qz,
               uint8_t flags, uint8_t cal_accuracy, uint32_t host_us) {
    // Drop QUAT early if FIFO is running low: reserves headroom for the largest
    // non-QUAT frames on the wire (a STATUS with a 60-char message is ~71 bytes,
    // the get_config ACK ~63). 128 = half the 256-byte TinyUSB CDC TX FIFO
    // (CFG_TUD_CDC_TX_BUFSIZE), so QUAT is dropped before ACKs are at risk.
    if (tud_cdc_write_available() < 128) return false;
    uint8_t pl[24];
    pl[0] = flags;
    pl[1] = cal_accuracy;
    memcpy(&pl[2 + 0],  &qw, 4);
    memcpy(&pl[2 + 4],  &qx, 4);
    memcpy(&pl[2 + 8],  &qy, 4);
    memcpy(&pl[2 + 12], &qz, 4);
    pl[18] = (uint8_t)( host_us        & 0xFF);
    pl[19] = (uint8_t)((host_us >> 8)  & 0xFF);
    pl[20] = (uint8_t)((host_us >> 16) & 0xFF);
    pl[21] = (uint8_t)((host_us >> 24) & 0xFF);
    pl[22] = 0;     // pad to 24 (kept for forward-compat)
    pl[23] = 0;
    return send_frame(0x01, pl, 24);
}

bool send_status(uint8_t code, int32_t arg, const char* msg) {
    uint8_t pl[6 + 60];
    pl[0] = code;
    pl[1] = (uint8_t)( (uint32_t)arg        & 0xFF);
    pl[2] = (uint8_t)(((uint32_t)arg >> 8)  & 0xFF);
    pl[3] = (uint8_t)(((uint32_t)arg >> 16) & 0xFF);
    pl[4] = (uint8_t)(((uint32_t)arg >> 24) & 0xFF);
    size_t mlen = msg ? strnlen(msg, 60) : 0;
    pl[5] = (uint8_t)mlen;
    if (mlen) memcpy(&pl[6], msg, mlen);
    return send_frame(0x02, pl, 6 + mlen);
}

void set_rx_callback(FrameRxCallback cb, void* user) {
    s_cb = cb;
    s_cb_user = user;
}

void poll_rx() {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        uint8_t b = (uint8_t)c;
        if (b == 0x00) {
            // Delimiter: if we weren't skipping a too-long frame, decode.
            if (!s_rx_skip && s_rx_len >= 3 && s_cb) {
                uint8_t type;
                uint8_t pl[FRAME_MAX_BODY];
                int n = frame_decode(s_rx, s_rx_len, &type, pl, sizeof(pl));
                if (n >= 0) s_cb(type, pl, (size_t)n, s_cb_user);
                // bad frames are silently dropped — host will retry / log.
            }
            s_rx_len = 0;
            s_rx_skip = false;
        } else if (s_rx_skip) {
            // already past buffer capacity, wait for delimiter to resync
        } else if (s_rx_len < RX_BUF_MAX) {
            s_rx[s_rx_len++] = b;
        } else {
            // Overflow: drop the partial frame and skip everything until the
            // next 0x00. Starting accumulation mid-frame would just waste
            // bytes that can never form a valid frame anyway.
            s_rx_len = 0;
            s_rx_skip = true;
        }
    }
}

}  // namespace cdc_io
