// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include "midi_mapper.h"

namespace midi_io {
    // Belt-and-suspenders: even if the MIDI USB interface was hidden at boot
    // (Config::enable_midi=false), short-circuit emits so a stale mapper
    // call can't push bytes into an unattached endpoint.
    void set_enabled(bool enabled);
    void emit(const MidiEvent& ev);
}
