// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include "quat_math.h"
#include <math.h>

Quat quat_mul(Quat a, Quat b) {
    return Quat{
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    };
}

Quat quat_conj(Quat q) {
    return Quat{ q.w, -q.x, -q.y, -q.z };
}

Quat quat_canonical_to(Quat q, Quat ref) {
    float dot = q.w*ref.w + q.x*ref.x + q.y*ref.y + q.z*ref.z;
    if (dot < 0.0f) return Quat{ -q.w, -q.x, -q.y, -q.z };
    return q;
}

// ZYX (yaw-Z, pitch-Y, roll-X) — standard aviation/audio.
// References: Diebel "Representing Attitude" eq. 290.
static Euler euler_zyx(Quat q) {
    Euler e;
    const float w=q.w, x=q.x, y=q.y, z=q.z;
    // Yaw  (Z)
    e.yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    // Pitch (Y) — clamp to avoid NaN at the singularity.
    float sinp = 2.0f*(w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    e.pitch = asinf(sinp);
    // Roll (X)
    e.roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    return e;
}

Euler quat_to_euler(Quat q) {
    return euler_zyx(q);
}

Quat euler_zyx_to_quat(Euler e) {
    float cy = cosf(e.yaw   * 0.5f), sy = sinf(e.yaw   * 0.5f);
    float cp = cosf(e.pitch * 0.5f), sp = sinf(e.pitch * 0.5f);
    float cr = cosf(e.roll  * 0.5f), sr = sinf(e.roll  * 0.5f);
    return Quat{
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    };
}

