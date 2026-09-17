// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include "config_schema.h"

namespace storage {

void   begin();              // call once in setup() before any load/save.
Config load_or_defaults();   // schema-mismatch → defaults; never returns blank.
bool   save(const Config& cfg);  // writes + reads back; false if flash didn't take.

}
