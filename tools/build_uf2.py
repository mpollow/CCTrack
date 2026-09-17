# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""PlatformIO post-build script: convert firmware.bin to firmware.uf2.

Wired in via platformio.ini:
    extra_scripts = post:../tools/build_uf2.py

The QT Py M0 ships with the UF2 bootloader, which exposes a USB mass-storage
volume (QTPY_BOOT) when reset is double-tapped. Dragging firmware.uf2 onto
that volume flashes the board — no toolchain, no 1200-bps touch, no bossac.

The board's application offset is 0x2000 (matches upload.offset_address in
the platform's adafruit_qt_py_m0.json). SAMD21 UF2 family id: 0x68ed2b88.
"""

import os
import subprocess
import sys

Import("env")  # noqa: F821 — PlatformIO SCons env


# PlatformIO loads extra_scripts via exec() with no __file__ in scope, so the
# script directory has to be derived from PROJECT_DIR (the firmware/ dir).
_TOOLS_DIR = os.path.normpath(os.path.join(env["PROJECT_DIR"], "..", "tools"))  # noqa: F821
_UF2CONV = os.path.join(_TOOLS_DIR, "uf2conv.py")


def _make_uf2(source, target, env):
    bin_path = str(target[0])
    uf2_path = os.path.splitext(bin_path)[0] + ".uf2"

    cmd = [
        sys.executable, _UF2CONV,
        "--convert",
        "--base", "0x2000",
        "--family", "SAMD21",
        "--output", uf2_path,
        bin_path,
    ]
    print(f"  [build_uf2] {' '.join(cmd)}")
    subprocess.check_call(cmd, env={**os.environ, "PYTHONPATH": _TOOLS_DIR})


env.AddPostAction("$BUILD_DIR/firmware.bin", _make_uf2)  # noqa: F821
