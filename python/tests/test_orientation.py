# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Quaternion -> yaw/pitch/roll tests (ZYX intrinsic Tait-Bryan)."""

import math

from cctrack import quat_to_ypr


def _axis_quat(axis, deg):
    """Unit quaternion for a rotation of ``deg`` about a single axis."""
    half = math.radians(deg) / 2.0
    s = math.sin(half)
    w = math.cos(half)
    x = s if axis == "x" else 0.0
    y = s if axis == "y" else 0.0
    z = s if axis == "z" else 0.0
    return w, x, y, z


def test_identity():
    yaw, pitch, roll = quat_to_ypr(1.0, 0.0, 0.0, 0.0)
    assert abs(yaw) < 1e-6
    assert abs(pitch) < 1e-6
    assert abs(roll) < 1e-6


def test_yaw_90():
    yaw, pitch, roll = quat_to_ypr(*_axis_quat("z", 90.0))
    assert abs(yaw - 90.0) < 1e-4
    assert abs(pitch) < 1e-4
    assert abs(roll) < 1e-4


def test_roll_45():
    yaw, pitch, roll = quat_to_ypr(*_axis_quat("x", 45.0))
    assert abs(roll - 45.0) < 1e-4
    assert abs(pitch) < 1e-4
    assert abs(yaw) < 1e-4


def test_pitch_clamped_at_pole():
    # +90° pitch is the gimbal-lock pole; must stay finite and ~+90.
    yaw, pitch, roll = quat_to_ypr(*_axis_quat("y", 90.0))
    assert abs(pitch - 90.0) < 1e-3


def test_radians_mode():
    yaw, pitch, roll = quat_to_ypr(*_axis_quat("z", 90.0), degrees=False)
    assert abs(yaw - math.pi / 2) < 1e-4
