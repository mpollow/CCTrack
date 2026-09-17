#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Print the live yaw/pitch/roll stream from an attached CCTrack.

    python examples/print_ypr.py [PORT]

PORT is optional; without it the device is auto-detected by USB VID/PID.
"""

import sys
import time

from cctrack import Tracker

port = sys.argv[1] if len(sys.argv) > 1 else None

with Tracker(port=port) as t:
    if not t.wait(timeout=3.0):
        sys.exit("No samples received — is the device streaming (stream_quat_cdc)?")
    while True:
        yaw, pitch, roll = t.get_ypr()
        age_ms = t.age() * 1e3
        print(f"\ryaw={yaw:+7.2f}  pitch={pitch:+7.2f}  roll={roll:+7.2f}  "
              f"age={age_ms:4.1f}ms  ok={t.frames_ok} bad={t.frames_bad}", end="")
        time.sleep(1 / 30)  # 30 Hz print; the reader thread stays at floor latency
