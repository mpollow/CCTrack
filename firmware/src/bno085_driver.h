// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once
#include <stdint.h>
#include "quat_math.h"

namespace bno085 {

struct Sample {
    Quat     q;            // raw fused quaternion
    uint8_t  accuracy;     // 0..3
};

bool begin();              // initialise BNO085 over I2C (probes 0x4A and 0x4B).
bool poll(Sample* out);    // reads next report. Returns true if a quaternion was decoded.

// Persist the current dynamic calibration profile to BNO085 internal flash.
// DCD remains enabled afterward so calibration continues to refine during use.
bool cal_save();

}
