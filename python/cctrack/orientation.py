# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Convert quaternions to Euler angles.

The convention matches the firmware's ``mount_ypr``: ZYX intrinsic Tait-Bryan
angles (yaw about Z, then pitch about Y, then roll about X), in degrees by
default. The pitch term is clamped so values near the gimbal-lock poles stay
finite instead of raising on a slightly out-of-range ``asin`` argument.
"""

from __future__ import annotations

import math


def quat_to_ypr(qw: float, qx: float, qy: float, qz: float, degrees: bool = True):
    """Return ``(yaw, pitch, roll)`` for quaternion ``(qw, qx, qy, qz)``.

    Angles are ZYX intrinsic (Tait-Bryan). With ``degrees=True`` the result is
    in degrees; otherwise radians.
    """
    # roll (X axis)
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    # pitch (Y axis), clamped against rounding past ±1 at the poles
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (Z axis)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    if degrees:
        return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)
    return yaw, pitch, roll
