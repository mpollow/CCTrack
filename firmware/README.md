<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->

# CCTrack firmware

PlatformIO project for the Adafruit QT Py SAMD21 + BNO085 head-tracker.
[`../docs/protocol.md`](../docs/protocol.md) documents everything on the
wire (frames, commands, config schema, MIDI mapping);
[`../docs/hardware.md`](../docs/hardware.md) covers wiring and parts.

## Build & flash

```sh
pio run                    # build (also writes firmware.uf2)
pio run -t upload          # flash over USB (normally no button press)
python ../tools/cctrack-monitor.py /dev/ttyACM0   # decode the CDC stream
```

`upload` opens the running firmware's serial port at 1200 baud, which
resets the board into its bootloader, then flashes it. If that fails
(e.g. the firmware is not running), double-tap reset and drag
`firmware.uf2` onto the `QTPY_BOOT` drive instead. The CDC port carries
binary frames, so use `cctrack-monitor.py` rather than a text terminal.

**Flashing resets the config.** Settings are stored in a zero-filled
region inside the firmware image (`FlashStorage` in
`src/config_storage.cpp`), so every flash — `upload` or UF2 — overwrites
them and the device boots with `config_default()`. Save your settings
first (`tools/cctrack-cmd.py <port> get_config`) and re-apply them with
`set_config`.

## Native unit tests

```sh
pio test -e native         # runs all host-target Unity tests
```

## LSP for vim/neovim

```sh
pio run -t compiledb       # emits compile_commands.json next to platformio.ini
```
