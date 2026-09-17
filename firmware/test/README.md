<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->

# Hardware smoke test

Run before publishing a release. Requires a QT Py M0 + BNO085 over
STEMMA QT. The re-zero button on `A1` is optional: the steps marked
*(button)* only apply if one is fitted. Don't use the QT Py's on-board
button for them — it is **RESET** and reboots the board (twice quickly
enters the UF2 bootloader). Commands run from the repo root; replace
`/dev/ttyACM0` with the device's port. Stop the monitor before running
`cctrack-cmd.py` — two programs on one port steal each other's data.

1. Tag locally (don't push yet) and flash:
   ```sh
   git tag vX.Y.Z
   (cd firmware && pio run -t upload)
   ```
   The version is stamped from the tag at build time, so tag first. Or
   double-tap reset and drag the `firmware.uf2` from `pio run` (under
   `firmware/.pio/build/`) onto the `QTPY_BOOT` USB volume.
   Expected: build + upload succeed, device re-enumerates within 5 seconds.
   Flashing resets the config to factory defaults, which the later steps
   assume (e.g. `output_mode=cc14`, `midi_rate_hz=100`).
   If any later step fails, drop the tag (`git tag -d vX.Y.Z`), fix, and
   start over.

2. Verify USB enumeration and version:
   ```sh
   lsusb -v -d 1209:cc3d | grep -E 'bcdDevice|iProduct|bInterfaceClass'
   python tools/cctrack-cmd.py /dev/ttyACM0 version
   ```
   Expected: `CCTrack #XXXX`, with both an Audio (MIDI) and a Communications
   (CDC) interface. `version` prints the tag (e.g. `0.1.0 (v0.1.0)`, no
   `-dirty`), and `bcdDevice` matches it (0.1.0 → `0.10`).

3. Verify CDC streaming:
   ```sh
   python tools/cctrack-monitor.py /dev/ttyACM0
   ```
   Expected: `QUAT … acc=N …` lines changing as you tilt the IMU. The
   device streams ~400 frames/s; the monitor prints only ~10 lines/s by
   default (`--quat-rate`). The boot status lines (`STATUS code=1 … 'boot'`,
   `STATUS code=10 … 're-zeroed'`) appear only if still buffered when the
   port opens — don't treat their absence as a failure.

4. Verify USB-MIDI (in a DAW, or on Linux with `aseqdump`):
   ```sh
   aseqdump -p $(aconnect -l | grep -i CCTrack | head -n1 | grep -oP 'client \K[0-9]+'):0
   ```
   Expected (default `output_mode=cc14`, `cc_base=16`): 14-bit CC pairs on
   CC 16/48, 17/49, 18/50 changing as you tilt.

5. Re-zero:
   - Tilt the device, then run
     `python tools/cctrack-cmd.py /dev/ttyACM0 rezero` — prints `ok` and
     `STATUS code=10 're-zeroed'`. Restart the monitor without moving the
     device: quaternions ≈ identity.
   - *(button)* With the monitor running, press the button briefly:
     `STATUS code=2 arg=0 msg='btn_short'`, then
     `STATUS code=10 arg=0 msg='re-zeroed'`, and quaternions ≈ identity.
     With no button fitted, confirm instead that the monitor never shows
     `btn_short`/`btn_long` on its own.

6. Auto-rezero on boot:
   - Tilt the device to a non-default orientation.
   - Unplug + replug, then restart the monitor.
   - Quaternions are ≈ identity for that orientation: the rezero offset is
     not persisted, so every boot re-zeroes on the first sample.

7. Calibration save:
   - DCD (dynamic calibration) is always on; `acc=` in the QUAT lines rises
     (0→3) as the BNO085 sees enough motion.
   - Move the device through slow figure-8s + tilts for ~30 s until
     `acc=3`.
   - Persist it: `python tools/cctrack-cmd.py /dev/ttyACM0 save_calibration`
     — prints `STATUS code=5 'cal_saved'` and `ok`.
   - *(button)* Holding the button ≥ 2 s does the same and emits
     `STATUS code=5 arg=1 msg='cal_saved'` then
     `STATUS code=3 arg=0 msg='btn_long'`.

8. Config validation:
   ```sh
   python tools/cctrack-cmd.py /dev/ttyACM0 set_config mount_ypr=nan,0,0; echo "exit=$?"
   python tools/cctrack-cmd.py /dev/ttyACM0 set_config midi_rate_hz=100; echo "exit=$?"
   ```
   Expected: the first is refused by the tool before anything is sent
   (`error: mount_ypr angles must be finite…`, `exit=1`); the second (the
   default, so an unchanged value) prints `ok` with `exit=0`. The
   firmware's own rejection of such blobs is covered by the native unit
   tests.

If all eight pass, push the tag (`git push origin vX.Y.Z`); the release
workflow builds and publishes the UF2.
