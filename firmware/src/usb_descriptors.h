// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <Adafruit_TinyUSB.h>

extern Adafruit_USBD_MIDI g_midi_dev;     // declared in usb_descriptors.cpp

namespace cctrack_usb {
    // Sets USB VID/PID/strings and instantiates CDC + (optionally) MIDI.
    // Call once from setup() before TinyUSB.begin().
    // name_override:  if non-empty, used as USB product and MIDI port name;
    //                 if null or empty, auto-generates "CCTrack #XXXX".
    // enable_midi:    if false, the MIDI interface is omitted from the
    //                 descriptor list and the host sees a CDC-only device.
    //                 Note: Adafruit_USBD_MIDI::begin() can only be called
    //                 once, at boot; toggling this at runtime requires a
    //                 reboot to take effect.
    void configure(const char* name_override, bool enable_midi);

    // Updates only the product / MIDI-port name descriptors. Safe to call
    // repeatedly. Caller is responsible for detach/delay/attach to re-enumerate.
    void set_product_name(const char* name_override);
}
