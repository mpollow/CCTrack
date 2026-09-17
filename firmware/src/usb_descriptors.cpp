// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "usb_descriptors.h"
#include <Adafruit_TinyUSB.h>
#include "fw_version.h"

// pid.codes open-source VID; PID 0xCC3D reserved for CCTrack via pid.codes
// (allocation pending, see README).
static constexpr uint16_t CCTRACK_USB_VID = 0x1209;
static constexpr uint16_t CCTRACK_USB_PID = 0xCC3D;

// One MIDI cable (in + out).
Adafruit_USBD_MIDI g_midi_dev(1);

namespace cctrack_usb {

// Persistent string storage: descriptor pointers reference these buffers, so
// they must outlive every call. Updating their content updates what the host
// sees on the next descriptor request.
static char s_auto_name[16];   // "CCTrack #XXXX\0"
static char s_serial[33];      // 32 hex digits + NUL
static char s_midi_port[37];   // up to 31-char name + " MIDI\0"

void set_product_name(const char* name_override) {
    const char* product_name = (name_override && name_override[0]) ? name_override : s_auto_name;
    snprintf(s_midi_port, sizeof(s_midi_port), "%s MIDI", product_name);
    TinyUSBDevice.setProductDescriptor(product_name);
    g_midi_dev.setStringDescriptor(s_midi_port);
}

void configure(const char* name_override, bool enable_midi) {
    // SAMD21 128-bit factory serial: four words at non-contiguous NVM addresses
    // (datasheet §9.3.3).
    uint32_t a = *reinterpret_cast<const uint32_t*>(0x0080A00Cu);
    uint32_t b = *reinterpret_cast<const uint32_t*>(0x0080A014u);
    uint32_t c = *reinterpret_cast<const uint32_t*>(0x0080A018u);
    uint32_t d = *reinterpret_cast<const uint32_t*>(0x0080A01Cu);
    uint16_t uid16 = static_cast<uint16_t>((a ^ b ^ c ^ d) & 0xFFFFu);

    snprintf(s_auto_name, sizeof(s_auto_name), "CCTrack #%04X", uid16);
    snprintf(s_serial,    sizeof(s_serial),    "%08lX%08lX%08lX%08lX",
             static_cast<unsigned long>(a), static_cast<unsigned long>(b),
             static_cast<unsigned long>(c), static_cast<unsigned long>(d));

    TinyUSBDevice.setID(CCTRACK_USB_VID, CCTRACK_USB_PID);
    TinyUSBDevice.setDeviceVersion(fw_version_bcd(CCTRACK_FW_MAJOR, CCTRACK_FW_MINOR, CCTRACK_FW_PATCH));
    TinyUSBDevice.setManufacturerDescriptor("Martin Pollow");
    TinyUSBDevice.setSerialDescriptor(s_serial);
    set_product_name(name_override);
    if (enable_midi) {
        g_midi_dev.begin();   // adds the MIDI interface to the descriptor list
    }
    // CDC is included automatically by Adafruit_TinyUSB when USE_TINYUSB is set
    // and `Serial` is referenced — independent of the MIDI gate.
}

} // namespace cctrack_usb
