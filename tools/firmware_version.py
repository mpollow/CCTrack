# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""PlatformIO pre-build script: stamp the firmware version from git.

Wired in via platformio.ini:
    extra_scripts = pre:../tools/firmware_version.py

Runs `git describe --tags --match "v[0-9]*" --dirty --always` and passes the
result to the compiler as CCTRACK_FW_MAJOR / _MINOR / _PATCH and
CCTRACK_FW_DESCRIBE (see firmware/lib/fw_version/fw_version.h). A release tag
v0.1.0 yields 0.1.0; builds with no version tag, or without git, yield 0.0.0
with the describe string (commit hash or "unknown") for identification.
"""

import re
import subprocess

Import("env")  # noqa: F821 — PlatformIO SCons env


def _describe(cwd):
    try:
        out = subprocess.run(
            ["git", "describe", "--tags", "--match", "v[0-9]*", "--dirty", "--always"],
            cwd=cwd, capture_output=True, text=True, check=True,
        )
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


_desc = _describe(env["PROJECT_DIR"])  # noqa: F821
_m = re.match(r"^v(\d+)\.(\d+)\.(\d+)(?:-|$)", _desc)
_major, _minor, _patch = (int(g) for g in _m.groups()) if _m else (0, 0, 0)

print(f"  [firmware_version] {_major}.{_minor}.{_patch} ({_desc})")
env.Append(CPPDEFINES=[  # noqa: F821
    ("CCTRACK_FW_MAJOR", _major),
    ("CCTRACK_FW_MINOR", _minor),
    ("CCTRACK_FW_PATCH", _patch),
    ("CCTRACK_FW_DESCRIBE", env.StringifyMacro(_desc)),  # noqa: F821
])
