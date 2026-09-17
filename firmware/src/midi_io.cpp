// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "midi_io.h"
#include "usb_descriptors.h"

namespace midi_io {

namespace { bool s_enabled = true; }

void set_enabled(bool enabled) { s_enabled = enabled; }

void emit(const MidiEvent& ev) {
    if (!s_enabled) return;
    uint8_t pkt[3] = { ev.status, ev.data1, ev.data2 };
    g_midi_dev.write(pkt, 3);
}

}
