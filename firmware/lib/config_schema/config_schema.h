// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

constexpr uint8_t CONFIG_SCHEMA_VERSION = 1;

enum OutputMode : uint8_t {
    OUTPUT_MODE_OFF            = 0,
    OUTPUT_MODE_CC7            = 1,
    OUTPUT_MODE_CC14           = 2,
    OUTPUT_MODE_QUATERNION_4CC = 3,
};

constexpr size_t DEVICE_NAME_MAX    = 32;  // includes NUL; 31 usable chars

struct Config {
    uint8_t   schema_ver;
    uint8_t   midi_channel;          // 1..16
    uint8_t   output_mode;           // OutputMode
    float     mount_ypr[3];          // physical sensor displacement from the user-aligned pose, degrees, ZYX intrinsic (yaw, pitch, roll); firmware applies the inverse. identity = {0,0,0}
    uint16_t  midi_rate_hz;          // MIDI emit rate (0 = emit on every sample); CDC is unthrottled
    bool      conjugate_output;      // true=conjugate quaternion before mapping (reverses rotation direction)
    char      device_name[DEVICE_NAME_MAX]; // empty = use auto-generated USB product name
    bool      enable_midi;           // false = MIDI interface hidden from host on next reattach
    bool      stream_quat_cdc;       // false = suppress QUAT frames on CDC (commands still served)
    uint8_t   cc_yaw;               // CC number for yaw axis (default 16)
    uint8_t   cc_pitch;             // CC number for pitch axis (default 17)
    uint8_t   cc_roll;              // CC number for roll axis (default 18)
};

// v1 wire layout: 56 bytes, including one reserved byte at offset 3. The
// schema is append-only: a future version may add trailing fields (with sane
// defaults for blobs that lack them) and bump MAX, but MIN stays the v1 size
// so v1 blobs keep round-tripping.
constexpr size_t CONFIG_SERIALIZED_MIN =
      1 /*schema*/ + 1 /*midi_channel*/ + 1 /*output_mode*/ + 1 /*reserved*/
    + 12 /*mount_ypr*/
    + 2 /*rate*/
    + 1 /*conjugate_output*/
    + DEVICE_NAME_MAX
    + 1 /*enable_midi*/ + 1 /*stream_quat_cdc*/
    + 3 /*cc_yaw + cc_pitch + cc_roll*/;

constexpr size_t CONFIG_SERIALIZED_MAX = CONFIG_SERIALIZED_MIN;

Config config_default(void);

// Returns bytes written, or 0 if `out_cap` is too small.
size_t config_serialize(const Config* cfg, uint8_t* out, size_t out_cap);

// Returns true on success. Accepts schema versions 1..CONFIG_SCHEMA_VERSION;
// fields appended in versions newer than the blob's default to their
// config_default() values (append-only migration).
bool config_deserialize(const uint8_t* in, size_t in_len, Config* out);

#ifdef __cplusplus
}
#endif
