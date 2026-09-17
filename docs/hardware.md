<!-- SPDX-License-Identifier: CERN-OHL-P-2.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->

# CCTrack hardware wiring

Reference build with the Adafruit QT Py SAMD21 and the Adafruit BNO085
breakout (#4754). Cabling is a single STEMMA QT lead — no soldering on the
I²C path.

## Schematic

```
   ┌──────────────────┐
   │  Host PC / USB   │
   │   power source   │
   └────────┬─────────┘
            │
            │  USB-C cable (data-capable): VBUS, D+, D−, GND
            ▼
 ┌────────────────────────────────────────────┐
 │            Adafruit QT Py SAMD21           │
 │  ┌──────────────────────────────────────┐  │
 │  │  USB-C jack                          │  │
 │  │   ├─ VBUS → 3V3 LDO → 3V3 rail       │  │
 │  │   └─ D+/D− → SAMD21 native USB       │  │
 │  │                                      │  │
 │  │  SAMD21E18A                          │  │
 │  │   ├─ PA16  →  SDA  (I²C)             │  │
 │  │   ├─ PA17  →  SCL  (I²C)             │  │
 │  │   ├─ NeoPixel  →  status LED         │  │
 │  │   └─ A1 (PA03) ─ opt. button ─ GND   │  │
 │  └──────────────────────────────────────┘  │
 │                                            │
 │  STEMMA QT connector  (JST-SH 1.0 mm, 4P)  │
 │     ┌───┬───┬───┬───┐                      │
 │     │ 1 │ 2 │ 3 │ 4 │   1: GND             │
 │     └─┬─┴─┬─┴─┬─┴─┬─┘   2: 3V3             │
 │       │   │   │   │     3: SDA (PA16)      │
 │       │   │   │   │     4: SCL (PA17)      │
 └───────┼───┼───┼───┼────────────────────────┘
         │   │   │   │
         │   │   │   │   STEMMA QT cable  (4-conductor, 1.0 mm pitch)
         │   │   │   │
         │   │   │   │     pin 1  ── BLACK   GND
         │   │   │   │     pin 2  ── RED     3V3
         │   │   │   │     pin 3  ── BLUE    SDA
         │   │   │   │     pin 4  ── YELLOW  SCL
         │   │   │   │
         ▼   ▼   ▼   ▼
 ┌────────────────────────────────────────────┐
 │       Adafruit BNO085 breakout (#4754)     │
 │                                            │
 │  STEMMA QT input  (JST-SH 1.0 mm, 4P)      │
 │     ┌───┬───┬───┬───┐                      │
 │     │ 1 │ 2 │ 3 │ 4 │   1: GND             │
 │     └─┬─┴─┬─┴─┬─┴─┬─┘   2: 3V3 (VIN)       │
 │       │   │   │   │     3: SDA             │
 │       │   │   │   │     4: SCL             │
 │       │   │   │   │                        │
 │       │   │   ├───┼── pull-up → 3V3        │
 │       │   │   │   ├── pull-up → 3V3        │
 │       │   │   ▼   ▼                        │
 │       │   │   SDA SCL ──► BNO085 IMU       │
 │       │   │           (SH2 protocol,       │
 │       │   │            I²C addr 0x4A)      │
 │       │   └────► 3V3 LDO → BNO085 VDD      │
 │       └────────► GND plane                 │
 └────────────────────────────────────────────┘
```

## Notes

- I²C is 3.3 V logic; the BNO085 breakout has on-board pull-ups, so the QT Py
  side needs no external resistors.
- **The button is optional.** Without it the device works the same: it
  re-zeroes on power-up, `tools/cctrack-cmd.py <port> rezero` re-zeroes on
  demand, and `save_calibration` stores the IMU calibration. If fitted, a
  momentary button between `A1` (PA03) and `GND` adds a short press =
  re-zero and a ≥ 2 s hold = save calibration. `A1` uses the SAMD21's
  internal pull-up (`pinMode(BUTTON, INPUT_PULLUP)` in
  `firmware/src/main.cpp`), so an unconnected pin simply reads "not
  pressed" and no external components are needed.
- The QT Py's own on-board button is **RESET**, not a user button: it
  reboots the board (which also re-zeroes), and a quick double press
  enters the UF2 bootloader.

## Orientation

The firmware's reference pose is the breakout lying **flat, component side
up (Z up), with the silkscreen X arrow pointing forward** — the direction
the listener faces. Angles follow a right-handed frame with X forward,
Y left, Z up: positive yaw turns left, positive pitch tips the nose down,
positive roll drops the right ear.

`mount_ypr` (config, degrees, intrinsic ZYX yaw → pitch → roll) describes
how the sensor is physically rotated away from that reference pose; the
firmware applies the inverse, so the enclosure can hold the IMU however it
fits.

**Reference build:** the breakout lies flat on top of the head (Z up) with
its X arrow pointing to the listener's **right** — a −90° yaw from the
reference pose. That is why the factory default is `mount_ypr=-90,0,0`
(set in `firmware/lib/config_schema/config_schema.cpp`).

**Other builds** — find `mount_ypr` empirically:

1. Work out the rotation from the reference pose to your mounting: first
   yaw about Z, then pitch about the new Y, then roll about the new X.
   For example, X arrow pointing left → `90,0,0`; X arrow pointing
   backward → `180,0,0`.
2. Apply it and re-zero while facing forward:
   ```sh
   tools/cctrack-cmd.py /dev/ttyACM0 set_config mount_ypr=YAW,PITCH,ROLL
   tools/cctrack-cmd.py /dev/ttyACM0 rezero
   ```
3. Check with a MIDI monitor (e.g. `aseqdump`, see
   [`firmware/test/README.md`](../firmware/test/README.md)): turning your
   head should move only the yaw CC (default 16/48), nodding only the
   pitch CC (17/49), and tilting an ear to the shoulder only the roll CC
   (18/50). If a motion shows up on the wrong CC, revisit step 1. If every
   axis moves the opposite way to what you want, flip `conjugate_output`
   instead of changing the angles.

## Bill of materials

| Part                                    | Notes                          |
|-----------------------------------------|--------------------------------|
| Adafruit QT Py SAMD21 (#4600)           | USB-C, on-board NeoPixel       |
| Adafruit BNO085 breakout (#4754)        | STEMMA QT, on-board I²C pull-ups |
| STEMMA QT / Qwiic cable, JST-SH 4-pin   | Length to suit enclosure       |
| *Optional:* push-button, SPST momentary | Wired across `A1` and `GND`    |
| USB-C cable, data-capable               | Host ↔ QT Py                   |
