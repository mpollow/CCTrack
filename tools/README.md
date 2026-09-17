<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->

# CCTrack tooling

## UF2 build artifact

`pio run` (in `firmware/`) writes `firmware.uf2` alongside `firmware.bin`
in the PlatformIO build directory (`firmware/.pio/build/…/`). Drag the
`.uf2` onto the `QTPY_BOOT` USB drive (double-tap reset to mount) to flash
without a toolchain.

`build_uf2.py` is a PlatformIO post-build hook that calls vendored
`uf2conv.py` and `uf2families.json` (upstream
[microsoft/uf2](https://github.com/microsoft/uf2), MIT-licensed) with
`-b 0x2000 -f SAMD21`. The vendored files are unmodified — to refresh:

```sh
curl -fsSL -o tools/uf2conv.py https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2conv.py
curl -fsSL -o tools/uf2families.json https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2families.json
```

## Release licence texts

`license-texts/` holds the full LGPL-2.1, GPL-3.0 and Apache-2.0 texts
(SPDX list copies). The release workflow bundles them with the licence
files of the pinned firmware dependencies, as `NOTICE` describes.

## cctrack-cmd

Sends one command to the device over the CDC port and prints the reply;
the exit status is 0 only if the device acknowledged with status OK.

```sh
python tools/cctrack-cmd.py /dev/ttyACM0 version
python tools/cctrack-cmd.py /dev/ttyACM0 get_config
python tools/cctrack-cmd.py /dev/ttyACM0 set_config output_mode=cc14 cc_base=16
```

`set_config` reads the current config, applies the `KEY=VALUE` overrides
(each range-checked before anything is sent), and writes it back. The
full command and field list is in
[`docs/protocol.md`](../docs/protocol.md). Both tools import the wire
helpers from the in-repo `python/cctrack` package, so run them from a
checkout.

## cctrack-monitor

Decodes COBS+CRC frames from the CCTrack CDC port and prints them.

```sh
python -m venv .venv && source .venv/bin/activate
pip install -r tools/requirements.txt
python tools/cctrack-monitor.py /dev/ttyACM0
```

The MCU streams QUAT frames at the BNO's native ~400 Hz over CDC (only
the MIDI side is throttled by `midi_rate_hz`). The monitor's
`--quat-rate` flag (default 10 Hz) decimates *printed* lines so the
terminal stays readable — it's a print throttle, not a stream throttle.
A consumer that wants to record at full fidelity should drop the flag
or replace the print path with file/socket I/O.
