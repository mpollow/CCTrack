// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "midi_mapper.h"
#include <math.h>

static int16_t map_radians_to_cc7(float rad) {
    // ±π → 0..127, centre 64.
    float v = (rad / (float)M_PI) * 64.0f + 64.0f;
    if (v < 0.0f)   v = 0.0f;
    if (v > 127.0f) v = 127.0f;
    return (int16_t)v;
}

static int16_t map_radians_to_cc14(float rad) {
    // ±π → 0..16383, centre 8192.
    float v = (rad / (float)M_PI) * 8192.0f + 8192.0f;
    if (v < 0.0f)     v = 0.0f;
    if (v > 16383.0f) v = 16383.0f;
    return (int16_t)v;
}

static int16_t map_quat_component_to_cc14(float c) {
    // [-1,+1] → 0..16383
    float v = (c + 1.0f) * 0.5f * 16383.0f;
    if (v < 0.0f)     v = 0.0f;
    if (v > 16383.0f) v = 16383.0f;
    return (int16_t)v;
}

static int emit_cc7(MidiEvent* out, int cap, int* n,
                    uint8_t channel, uint8_t cc, uint8_t value) {
    if (*n >= cap) return -1;
    out[*n].status = (uint8_t)(0xB0 | ((channel - 1) & 0x0F));
    out[*n].data1  = cc & 0x7F;
    out[*n].data2  = value & 0x7F;
    (*n)++;
    return 0;
}

static int emit_cc14(MidiEvent* out, int cap, int* n,
                     uint8_t channel, uint8_t base_cc, uint16_t value14) {
    if (*n + 1 >= cap) return -1;
    uint8_t msb = (uint8_t)((value14 >> 7) & 0x7F);
    uint8_t lsb = (uint8_t)(value14 & 0x7F);
    if (emit_cc7(out, cap, n, channel, base_cc + 32u,  lsb)) return -1;
    if (emit_cc7(out, cap, n, channel, base_cc,        msb)) return -1;
    return 0;
}

int midi_mapper_step(MidiMapperState* st, const Config* cfg,
                     Quat q, uint32_t now_ms,
                     MidiEvent* out, int out_cap) {
    if (cfg->output_mode == OUTPUT_MODE_OFF) return 0;

    // Down-sample. Integer division quantizes the period to whole
    // milliseconds, so the effective rate can differ from the configured one
    // (e.g. 150 Hz -> 6 ms period -> ~166 Hz effective). midi_rate_hz itself
    // isn't range-checked in config_schema.cpp; harmless — period_ms just
    // saturates toward 0 (no throttling) as the configured rate grows.
    const uint32_t period_ms = (cfg->midi_rate_hz == 0) ? 0
                             : (1000u / cfg->midi_rate_hz);
    if (st->primed && (now_ms - st->last_emit_ms) < period_ms) return 0;

    int n = 0;
    int16_t v[4] = {0, 0, 0, 0};
    bool changed[4] = {true, true, true, true};

    // Input quaternion is already mount-corrected, re-zeroed, and conjugated by the caller.
    if (cfg->output_mode == OUTPUT_MODE_CC7
     || cfg->output_mode == OUTPUT_MODE_CC14) {
        Euler e = quat_to_euler(q);
        if (cfg->output_mode == OUTPUT_MODE_CC7) {
            v[0] = map_radians_to_cc7(e.yaw);
            v[1] = map_radians_to_cc7(e.pitch);
            v[2] = map_radians_to_cc7(e.roll);
        } else {
            v[0] = map_radians_to_cc14(e.yaw);
            v[1] = map_radians_to_cc14(e.pitch);
            v[2] = map_radians_to_cc14(e.roll);
        }
    } else if (cfg->output_mode == OUTPUT_MODE_QUATERNION_4CC) {
        v[0] = map_quat_component_to_cc14(q.w);
        v[1] = map_quat_component_to_cc14(q.x);
        v[2] = map_quat_component_to_cc14(q.y);
        v[3] = map_quat_component_to_cc14(q.z);
    }

    // Dedup vs last emission.
    if (st->primed) {
        for (int i = 0; i < 4; i++) changed[i] = (v[i] != st->last_value[i]);
    }

    bool any = false;
    for (int i = 0; i < 4; i++) if (changed[i]) { any = true; break; }
    if (!any && st->primed) return 0;

    // Tracks which axes actually made it into `out` this call, so a dropped
    // event (out_cap reached) doesn't get committed to last_value below —
    // otherwise dedup would falsely skip re-sending it forever once capacity
    // recovers. Unreachable with the current caller's evs[8] sizing (always
    // ample), but a latent trap if that buffer ever shrinks.
    bool emitted[4] = {false, false, false, false};

    if (cfg->output_mode == OUTPUT_MODE_CC7) {
        if (changed[0]) emitted[0] = (emit_cc7 (out, out_cap, &n, cfg->midi_channel, cfg->cc_yaw,   (uint8_t)v[0]) == 0);
        if (changed[1]) emitted[1] = (emit_cc7 (out, out_cap, &n, cfg->midi_channel, cfg->cc_pitch, (uint8_t)v[1]) == 0);
        if (changed[2]) emitted[2] = (emit_cc7 (out, out_cap, &n, cfg->midi_channel, cfg->cc_roll,  (uint8_t)v[2]) == 0);
    } else if (cfg->output_mode == OUTPUT_MODE_CC14) {
        if (changed[0]) emitted[0] = (emit_cc14(out, out_cap, &n, cfg->midi_channel, cfg->cc_yaw,   (uint16_t)v[0]) == 0);
        if (changed[1]) emitted[1] = (emit_cc14(out, out_cap, &n, cfg->midi_channel, cfg->cc_pitch, (uint16_t)v[1]) == 0);
        if (changed[2]) emitted[2] = (emit_cc14(out, out_cap, &n, cfg->midi_channel, cfg->cc_roll,  (uint16_t)v[2]) == 0);
    } else if (cfg->output_mode == OUTPUT_MODE_QUATERNION_4CC) {
        // Quat mode uses cc_yaw as the consecutive base (qw=cc_yaw+0 .. qz=cc_yaw+3)
        for (int i = 0; i < 4; i++) {
            if (changed[i])
                emitted[i] = (emit_cc14(out, out_cap, &n, cfg->midi_channel,
                                        (uint8_t)(cfg->cc_yaw + i), (uint16_t)v[i]) == 0);
        }
    }

    for (int i = 0; i < 4; i++) {
        if (changed[i] && !emitted[i]) continue;  // dropped: keep old value so it's retried
        st->last_value[i] = v[i];
    }
    st->last_emit_ms = now_ms;
    st->primed = true;
    return n;
}
