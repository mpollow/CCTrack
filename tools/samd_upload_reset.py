#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""PlatformIO pre-upload script: 1200-bps touch → SAM-BA port detection.

Wired in via platformio.ini:
    extra_scripts = pre:../tools/samd_upload_reset.py

Flow:
  1. Find the running firmware's CDC port: UPLOAD_PORT if given, else the
     port with CCTrack's USB ID (1209:CC3D), else PlatformIO autodetect.
  2. Open it at 1200 baud, close immediately → SAMD21 resets to SAM-BA bootloader.
  3. Scan for the bootloader's CDC port by diffing /dev/ttyACM* before/after.
  4. Inject it as UPLOAD_PORT so bossac gets the right port.
"""

import glob
import os
import time

Import("env")  # noqa: F821 — PlatformIO SCons env


# CCTrack's own USB ID (firmware/src/usb_descriptors.cpp). PlatformIO's
# autodetect only knows the board's stock Adafruit IDs (239A:80CB/00CB), so
# without this it falls back to an unrelated port such as /dev/ttyS0.
_CCTRACK_VID = 0x1209
_CCTRACK_PID = 0xCC3D


def _find_cctrack_port():
    """Return the CDC port of the one attached CCTrack, or None."""
    from serial.tools import list_ports
    matches = sorted(p.device for p in list_ports.comports()
                     if p.vid == _CCTRACK_VID and p.pid == _CCTRACK_PID)
    if len(matches) > 1:
        print(f"  [samd_upload_reset] several CCTrack devices found ({', '.join(matches)}); "
              "choose one with --upload-port")
        return None
    return matches[0] if matches else None


def _serial_ports():
    """Return a set of existing /dev/ttyACM* port paths."""
    return set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def _find_samd_bootloader_port(before: set, timeout: float = 10.0) -> str | None:
    """Wait for bootloader port to appear after reset.

    Handles two cases:
    - Bootloader appears at a new address (e.g., ttyACM0 → ttyACM1)
    - Bootloader re-uses the same address (disappears briefly then reappears)
    """
    deadline = time.monotonic() + timeout

    # Phase 1: wait for the reset to happen (all before-ports disappear).
    gone = False
    while time.monotonic() < deadline:
        if not (_serial_ports() & before):
            gone = True
            break
        time.sleep(0.05)

    # Phase 2: wait for any port to come back.
    while time.monotonic() < deadline:
        current = _serial_ports()
        if current:
            new = current - before
            if new:
                return sorted(new)[0]           # genuinely new port
            if gone:
                return sorted(current)[0]       # same port re-appeared after reset
        time.sleep(0.05)

    return None


def _do_1200_touch(port: str) -> None:
    import serial
    try:
        s = serial.Serial(port, 1200, timeout=1)
        time.sleep(0.1)
        s.close()
    except Exception as e:
        print(f"  [samd_upload_reset] 1200-bps touch failed: {e}")


def before_upload(source, target, env):
    upload_port = env.get("UPLOAD_PORT", None)
    if not upload_port:
        upload_port = _find_cctrack_port()
        if upload_port:
            print(f"  [samd_upload_reset] found CCTrack on {upload_port}")
            env.Replace(UPLOAD_PORT=upload_port)
    if not upload_port:
        env.AutodetectUploadPort()
        upload_port = env.get("UPLOAD_PORT", None)

    if not upload_port:
        print("  [samd_upload_reset] no upload port found, skipping reset")
        return

    print(f"  [samd_upload_reset] 1200-bps touch on {upload_port}")

    before = _serial_ports()
    _do_1200_touch(upload_port)

    # Wait up to 10 s for a new ttyACM* to appear (the SAM-BA bootloader).
    # If none appears, fall back to checking whether the existing port changed.
    time.sleep(0.5)   # give the SAMD a moment to reset

    new_port = _find_samd_bootloader_port(before, timeout=8.0)
    if new_port:
        print(f"  [samd_upload_reset] bootloader port: {new_port}")
        env.Replace(UPLOAD_PORT=new_port)
    else:
        # Port may have reappeared at the same address.
        current = _serial_ports()
        if upload_port in current:
            print(f"  [samd_upload_reset] port unchanged at {upload_port}, continuing")
        else:
            print("  [samd_upload_reset] no bootloader port found; manual reset may be needed")


env.AddPreAction("upload", before_upload)
