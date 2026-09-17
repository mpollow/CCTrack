<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->

# CCTrack

A USB-C head-tracker for spatial-audio production, built from an Adafruit
QT Py SAMD21 and a BNO085 IMU. Plug it in and it shows up as two things at
once, no drivers on any OS:

- **USB-MIDI** (class-compliant): head orientation as MIDI CC — 7-bit,
  14-bit, or raw-quaternion mapping, fully configurable.
- **USB-CDC** (virtual COM port): a ~400 Hz stream of COBS-framed,
  CRC-checked quaternions plus a binary command channel for configuration.

## Quick start

**Hardware:** an Adafruit QT Py SAMD21 and an Adafruit BNO085 breakout,
joined by one STEMMA QT cable — no soldering. A re-zero button is optional
(see [`docs/hardware.md`](docs/hardware.md)).

**Flash:** download `cctrack-vX.Y.Z.uf2` from the
[latest release](https://github.com/mpollow/CCTrack/releases/latest) (or
build it with `pio run` in `firmware/`), double-tap the QT Py's
reset button, and drag the file onto the `QTPY_BOOT` drive. The device
re-enumerates as `CCTrack #XXXX`; `tools/cctrack-cmd.py <port> version`
prints the firmware version. **Flashing (including updates) resets all
settings to factory defaults** — re-apply your preset afterwards.

**Use as MIDI:** it already sends 14-bit CC pairs for yaw/pitch/roll on
CC 16/17/18 (channel 1) out of the box. Map them in your DAW like any
controller.

The device re-zeroes on power-up, so plug it in facing
forward. To re-zero later, run `tools/cctrack-cmd.py <port> rezero` (or
replug, or press the optional button on `A1`; the QT Py's on-board button
is reset, not re-zero).

*Example application:* head-tracked binaural rendering with
[IEM SceneRotator](https://plugins.iem.at/docs/plugindescriptions/#scenerotator)
in REAPER — select CCTrack as the plugin's MIDI input and set its MIDI
scheme to **"MrHT YPR Direct"**, which matches CCTrack's factory default
(14-bit CC pairs on CC 16/17/18). Keep `midi_rate_hz` at its default of
100 unless you have a reason to raise it; see
[`docs/protocol.md`](docs/protocol.md) §3.3.

**Read it from Python:**

```python
from cctrack import Tracker

with Tracker() as t:                  # auto-detects the device, waits for data
    yaw, pitch, roll = t.get_ypr()    # newest sample, non-blocking, degrees
```

See [`python/README.md`](python/README.md) for the library
(`pip install "git+https://github.com/mpollow/CCTrack#subdirectory=python"`).

## Repo layout

| Directory | Contents | Licence |
|---|---|---|
| `firmware/` | QT Py M0 firmware (PlatformIO) | MIT |
| `python/` | `cctrack` host library (quaternion / yaw-pitch-roll consumer) | MIT |
| `tools/` | Config CLI, stream monitor, UF2 build helpers (Python) | MIT |
| `docs/` | Protocol reference; hardware wiring and BOM (`docs/hardware.md`) | CC BY 4.0; `docs/hardware.md` CERN-OHL-P v2 |

## Documentation

[`docs/protocol.md`](docs/protocol.md) is the **source of truth** for
everything on the wire: CDC frame format, command set, config schema,
MIDI mapping, and a walkthrough of `tools/cctrack-cmd.py`. A third party
can implement a host from it.

## Configuration

All configuration happens over the CDC port with
[`tools/cctrack-cmd.py`](tools/cctrack-cmd.py); settings persist in
flash. Apply a preset, then re-zero (`rezero` command, replug, or the
optional button) if needed.

**Headtracker, yaw/pitch/roll** — 14-bit CC pairs on CC 16/17/18 (the
factory default):

```sh
tools/cctrack-cmd.py /dev/ttyACM0 set_config \
  output_mode=cc14 midi_rate_hz=100 \
  mount_ypr=-90,0,0 \
  conjugate_output=1 cc_base=16
```

**Pointer, yaw/pitch/roll** — same, but rotation direction not reversed
(device as pointer rather than head-compensation):

```sh
tools/cctrack-cmd.py /dev/ttyACM0 set_config \
  output_mode=cc14 midi_rate_hz=100 \
  mount_ypr=-90,0,0 \
  conjugate_output=0 cc_base=16
```

**Raw quaternion** — w/x/y/z as four 14-bit CC pairs starting at CC 16:

```sh
tools/cctrack-cmd.py /dev/ttyACM0 set_config \
  output_mode=quat_4cc midi_rate_hz=100 \
  mount_ypr=-90,0,0 \
  conjugate_output=1 cc_base=16
```

`cc_base=N` is shorthand for `cc_yaw=N cc_pitch=N+1 cc_roll=N+2`; each
axis can also be set independently.

`mount_ypr` (yaw, pitch, roll in degrees) tells the firmware how the
sensor is rotated inside your build; the presets use `-90,0,0` for the
reference build. [`docs/hardware.md`](docs/hardware.md#orientation)
explains the reference orientation and how to find the right values for
yours. See [`docs/protocol.md`](docs/protocol.md) §2 for the full config
schema.

## Building and testing

```sh
cd firmware
pio run                 # firmware build (also emits firmware.uf2)
pio test -e native      # unit tests, host-compiled, no hardware needed
```

```sh
cd python
pip install -e ".[dev]"
pytest                  # host-library tests, no hardware needed
cd ..
pytest tools/tests      # config CLI tests
```

CI runs all of these plus the firmware build on every push.

**Releases:** tag the commit locally (`git tag vX.Y.Z` — the firmware
version is stamped from the tag at build time), run the on-hardware
checklist in [`firmware/test/README.md`](firmware/test/README.md), then
push the tag. The release workflow builds the firmware and publishes it,
with licence texts and checksums, as a GitHub release.

## Licensing

CCTrack's own files are permissively licensed: **MIT** for code ([`LICENSE`](LICENSE)),
**CC BY 4.0** for documentation
([`LICENSE-CC-BY-4.0`](LICENSE-CC-BY-4.0)), **CERN-OHL-P v2** for the
hardware design — the wiring and BOM in `docs/hardware.md`
([`LICENSE-CERN-OHL-P-v2`](LICENSE-CERN-OHL-P-v2)).
Per-file SPDX headers identify each file's licence. The firmware binary
also contains third-party libraries, some under the LGPL (Arduino core,
NeoPixel, FlashStorage); [`NOTICE`](NOTICE) lists them all.

USB identity: VID `0x1209`, PID `0xCC3D` ([pid.codes](https://pid.codes),
allocation pending).

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) — DCO sign-off, no CLA.
