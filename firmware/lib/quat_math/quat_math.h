// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct Quat  { float w, x, y, z; };
struct Euler { float yaw, pitch, roll; };  // radians

Quat  quat_mul(Quat a, Quat b);
Quat  quat_conj(Quat q);

// q and -q describe the same rotation (quaternion double cover). Returns q if
// it lies in the same hemisphere as ref (dot >= 0), else -q. Use this to keep
// a stream of orientation samples continuous when the source (e.g. the BNO085
// game rotation vector) occasionally emits the opposite-sign representation
// mid-motion.
Quat  quat_canonical_to(Quat q, Quat ref);

// ZYX decomposition: yaw=Z, pitch=Y, roll=X.
Euler quat_to_euler(Quat q);
Quat  euler_zyx_to_quat(Euler e);

#ifdef __cplusplus
}
#endif
