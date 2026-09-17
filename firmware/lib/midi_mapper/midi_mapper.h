// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include "config_schema.h"
#include "quat_math.h"

#ifdef __cplusplus
extern "C" {
#endif

struct MidiEvent {
    uint8_t status;     // includes channel in low nibble
    uint8_t data1;
    uint8_t data2;
};

struct MidiMapperState {
    uint32_t last_emit_ms;
    int16_t  last_value[4];   // last emitted integer value per axis (for dedup)
    bool     primed;          // false until first emission
};

// Steps the mapper: takes the current quaternion and a wall-clock-ish ms
// counter; appends MIDI events into `out`. Returns event count (≥0).
int midi_mapper_step(MidiMapperState* st, const Config* cfg,
                     Quat q, uint32_t now_ms,
                     MidiEvent* out, int out_cap);

#ifdef __cplusplus
}
#endif
