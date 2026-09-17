// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace cdc_io {

// Encode and write a frame on the CDC port. Returns false if encoding failed
// or USB write blocked (caller can drop and try next tick).
bool send_frame(uint8_t type, const uint8_t* payload, size_t payload_len);

// Convenience: build a QUAT payload from quaternion + flags + accuracy +
// timestamp and send it.
bool send_quat(float qw, float qx, float qy, float qz,
               uint8_t flags, uint8_t cal_accuracy, uint32_t host_us);

// Send a STATUS frame.
bool send_status(uint8_t code, int32_t arg, const char* msg);

// Pump RX: reads up to N bytes from CDC, accumulates them, and on each `0x00`
// delimiter calls the user-provided callback with (type, payload, len).
// Callback runs in the main loop context.
typedef void (*FrameRxCallback)(uint8_t type,
                                const uint8_t* payload, size_t len, void* user);
void set_rx_callback(FrameRxCallback cb, void* user);
void poll_rx();

}
