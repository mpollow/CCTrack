// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "config_storage.h"
#include <FlashStorage.h>
#include <string.h>

namespace {
struct StoredBlob {
    uint16_t magic;
    uint16_t length;
    uint8_t  bytes[CONFIG_SERIALIZED_MAX];
};

constexpr uint16_t MAGIC = 0xABCD;

FlashStorage(s_blob, StoredBlob);

#ifdef ARDUINO_ARCH_SAMD
// On SAMD21 the NVM address space shadows the page buffer after a write, so a
// read-back would return the just-written buffer rather than the committed
// flash cells. A Page Buffer Clear makes the verify read true flash.
static void flush_page_buffer() {
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
    while (!NVMCTRL->INTFLAG.bit.READY) { }
}
#endif
}

namespace storage {

void begin() { /* FlashStorage init is implicit */ }

Config load_or_defaults() {
    StoredBlob b = s_blob.read();
    if (b.magic != MAGIC) return config_default();
    if (b.length == 0 || b.length > CONFIG_SERIALIZED_MAX) return config_default();
    Config c{};
    if (!config_deserialize(b.bytes, b.length, &c)) return config_default();
    return c;
}

bool save(const Config& cfg) {
    StoredBlob b{};
    b.magic = MAGIC;
    b.length = (uint16_t)config_serialize(&cfg, b.bytes, sizeof(b.bytes));
    if (b.length == 0) return false;

    // Skip identical rewrites: SAMD21 NVM rows have finite erase endurance, and
    // hosts may resend an unchanged config (e.g. a scripted set_config loop).
    StoredBlob current = s_blob.read();
    if (current.magic == b.magic && current.length == b.length
        && memcmp(current.bytes, b.bytes, b.length) == 0) {
        return true;
    }

    s_blob.write(b);

#ifdef ARDUINO_ARCH_SAMD
    flush_page_buffer();
#endif
    StoredBlob check = s_blob.read();
    return check.magic == b.magic
        && check.length == b.length
        && memcmp(check.bytes, b.bytes, b.length) == 0;
}

}  // namespace storage
