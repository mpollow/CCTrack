// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include "config_schema.h"

namespace cmd {

constexpr uint8_t CMD_REZERO_NOW          = 1;
// CMD IDs 2 and 3 were CMD_CAL_START / CMD_CAL_CANCEL before DCD became
// always-on; left as gaps to keep wire compatibility for the remaining commands.
constexpr uint8_t CMD_SAVE_CALIBRATION    = 4;
constexpr uint8_t CMD_GET_CONFIG          = 5;
constexpr uint8_t CMD_SET_CONFIG          = 6;
constexpr uint8_t CMD_RESET_TO_DEFAULTS   = 7;
constexpr uint8_t CMD_PING                = 8;
constexpr uint8_t CMD_REBOOT_TO_UF2       = 9;
constexpr uint8_t CMD_GET_VERSION         = 10;

constexpr uint8_t STATUS_OK               = 0;
constexpr uint8_t STATUS_ERR_ARG          = 1;
constexpr uint8_t STATUS_ERR_STATE        = 2;
constexpr uint8_t STATUS_ERR_UNSUPPORTED  = 3;

typedef void (*RezeroFn)();
typedef void (*SaveCalFn)();
typedef bool (*ConfigSetFn)(const Config* new_cfg);
typedef Config (*ConfigGetFn)();

struct Hooks {
    RezeroFn      rezero;
    SaveCalFn     save_calibration;
    ConfigSetFn   set_config;
    ConfigGetFn   get_config;
};

void install(const Hooks* hooks);
void on_frame(uint8_t type, const uint8_t* payload, size_t len);

}
