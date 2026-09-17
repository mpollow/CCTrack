// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "config_schema.h"
#include <cmath>
#include <string.h>

static void put_u8 (uint8_t** p, uint8_t  v) { *(*p)++ = v; }
static void put_u16(uint8_t** p, uint16_t v) { (*p)[0] = (uint8_t)v; (*p)[1] = (uint8_t)(v >> 8); *p += 2; }
static void put_f32(uint8_t** p, float    v) { memcpy(*p, &v, 4); *p += 4; }

static uint8_t  get_u8 (const uint8_t** p) { return *(*p)++; }
static uint16_t get_u16(const uint8_t** p) { uint16_t v = (uint16_t)((*p)[0]) | ((uint16_t)((*p)[1]) << 8); *p += 2; return v; }
static float    get_f32(const uint8_t** p) { float v; memcpy(&v, *p, 4); *p += 4; return v; }

Config config_default(void) {
    Config c = {};
    c.schema_ver = CONFIG_SCHEMA_VERSION;
    c.midi_channel = 1;
    c.output_mode = OUTPUT_MODE_CC14;
    c.cc_yaw   = 16;
    c.cc_pitch = 17;
    c.cc_roll  = 18;
    // mount_ypr = the sensor's physical displacement from the user-aligned pose;
    // firmware applies the inverse. The sensor is mounted yawed -90°, so the
    // applied correction is +90° — restoring "front" to the user's "forward".
    c.mount_ypr[0] = -90.0f;
    c.mount_ypr[1] = 0.0f;
    c.mount_ypr[2] = 0.0f;
    c.midi_rate_hz = 100;
    c.conjugate_output = true;
    memset(c.device_name, 0, DEVICE_NAME_MAX);
    c.enable_midi      = true;
    c.stream_quat_cdc  = true;
    return c;
}

size_t config_serialize(const Config* cfg, uint8_t* out, size_t out_cap) {
    if (out_cap < CONFIG_SERIALIZED_MAX) return 0;
    uint8_t* p = out;
    // Always stamp the running schema version, never the caller's field: a
    // read-modify-write client (cctrack-cmd.py) round-trips whatever it read,
    // so trusting cfg->schema_ver could pin a stored blob below the firmware's
    // actual version and make get_config misreport it.
    put_u8 (&p, CONFIG_SCHEMA_VERSION);
    put_u8 (&p, cfg->midi_channel);
    put_u8 (&p, cfg->output_mode);
    put_u8 (&p, 0u);  // offset 3: reserved, always 0
    put_f32(&p, cfg->mount_ypr[0]);
    put_f32(&p, cfg->mount_ypr[1]);
    put_f32(&p, cfg->mount_ypr[2]);
    put_u16(&p, cfg->midi_rate_hz);
    put_u8 (&p, cfg->conjugate_output ? 1u : 0u);
    for (size_t i = 0; i < DEVICE_NAME_MAX; i++) put_u8(&p, (uint8_t)cfg->device_name[i]);
    put_u8 (&p, cfg->enable_midi     ? 1u : 0u);
    put_u8 (&p, cfg->stream_quat_cdc ? 1u : 0u);
    put_u8 (&p, cfg->cc_yaw);
    put_u8 (&p, cfg->cc_pitch);
    put_u8 (&p, cfg->cc_roll);
    return (size_t)(p - out);
}

bool config_deserialize(const uint8_t* in, size_t in_len, Config* out) {
    // Accept any schema version we recognise (1..ours) as long as the blob is
    // at least the v1 size. The schema is append-only, so if a future version
    // has added trailing fields, a v1 blob still parses and the new fields
    // default — but a blob *shorter* than v1 is malformed, not old.
    if (in_len < CONFIG_SERIALIZED_MIN)               return false;
    if (in[0] < 1 || in[0] > CONFIG_SCHEMA_VERSION)   return false;

    const uint8_t* p = in;
    out->schema_ver        = get_u8 (&p);
    out->midi_channel      = get_u8 (&p);
    out->output_mode       = get_u8 (&p);
    get_u8(&p);  // offset 3: reserved byte, consumed and ignored
    out->mount_ypr[0]      = get_f32(&p);
    out->mount_ypr[1]      = get_f32(&p);
    out->mount_ypr[2]      = get_f32(&p);
    out->midi_rate_hz      = get_u16(&p);
    out->conjugate_output  = (get_u8(&p) != 0);
    for (size_t i = 0; i < DEVICE_NAME_MAX; i++) out->device_name[i] = (char)get_u8(&p);
    out->device_name[DEVICE_NAME_MAX - 1] = '\0';  // guarantee null termination
    // device_name feeds directly into USB string descriptors and the MIDI
    // port name; replace any control character (host-supplied, untrusted)
    // with '_' so it can't confuse or crash a host's USB/MIDI stack.
    for (size_t i = 0; i < DEVICE_NAME_MAX; i++) {
        char c = out->device_name[i];
        if (c == '\0') break;
        if ((uint8_t)c < 0x20 || (uint8_t)c == 0x7F) out->device_name[i] = '_';
    }

    out->enable_midi     = (get_u8(&p) != 0);
    out->stream_quat_cdc = (get_u8(&p) != 0);
    out->cc_yaw   = get_u8(&p);
    out->cc_pitch = get_u8(&p);
    out->cc_roll  = get_u8(&p);

    // Range-check fields that would otherwise fail silently downstream: a bad
    // midi_channel wraps via (ch-1)&0x0F to the wrong channel, an out-of-range
    // output_mode emits nothing, and CC numbers >127 wrap when masked &0x7F.
    // Reject rather than clamp so the caller (set_config -> STATUS_ERR_ARG, or
    // load_or_defaults -> fall back to defaults) sees a clean failure.
    if (out->midi_channel < 1 || out->midi_channel > 16)        return false;
    // A NaN/Inf mount angle would poison the mount correction and the rezero
    // offset (and, once persisted, survive reboots). |angle| > 360 is never
    // needed to express a rotation, so treat it as a malformed blob too.
    for (int i = 0; i < 3; i++) {
        float a = out->mount_ypr[i];
        if (!std::isfinite(a) || std::fabs(a) > 360.0f)         return false;
    }
    if (out->output_mode > OUTPUT_MODE_QUATERNION_4CC)          return false;
    if (out->cc_yaw > 127 || out->cc_pitch > 127 || out->cc_roll > 127) return false;
    // CC14 pairs put the LSB at base+32 (emit_cc14 in midi_mapper.cpp); a base
    // above 95 makes base+32 exceed 127 and wrap when masked with &0x7F,
    // colliding with an unrelated low-numbered CC.
    if (out->output_mode == OUTPUT_MODE_CC14) {
        if (out->cc_yaw > 95 || out->cc_pitch > 95 || out->cc_roll > 95) return false;
    }
    // quat_4cc lays four consecutive CCs starting at cc_yaw, each with its own
    // LSB at (cc_yaw+i)+32; the last pair's LSB is cc_yaw+3+32, so the base
    // must leave room for that to stay within 0..127.
    if (out->output_mode == OUTPUT_MODE_QUATERNION_4CC && out->cc_yaw > 92) return false;

    return true;
}
