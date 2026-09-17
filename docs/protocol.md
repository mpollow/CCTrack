<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->

# CCTrack data IO

The MCU is a composite USB device with two independent interfaces. Reference
points for everything below: `firmware/src/usb_descriptors.cpp`,
`firmware/src/cdc_io.cpp`, `firmware/src/command_processor.cpp`,
`firmware/lib/frame_codec/`, `firmware/lib/midi_mapper/`,
`firmware/lib/config_schema/`, `tools/cctrack-cmd.py`,
`tools/cctrack-monitor.py`.

| USB interface | Direction | Carries |
|---|---|---|
| **USB-MIDI** (class-compliant, one cable) | MCU → host | Head orientation as MIDI CC, parameterised by config |
| **USB-CDC** (virtual COM, e.g. `/dev/ttyACM0`) | bidirectional | COBS-framed binary protocol: quaternion stream, status events, host commands |

USB IDs: VID `0x1209`, PID `0xCC3D` ([pid.codes](https://pid.codes), allocation pending). Product name and MIDI port name are
derived from `Config.device_name` (or auto-generated `CCTrack #XXXX` when
the name field is empty).

---

## 1. CDC wire protocol

Binary, COBS-framed, with **CRC-16/CCITT-FALSE** (poly `0x1021`, init
`0xFFFF`, no reflection, no xor-out). All multi-byte fields are
**little-endian**.

### 1.1 Framing

```
  ┌──────────────────── body (COBS-encoded) ────────────────────┐
  │                                                             │
  cobs( type:u8  payload[…]  crc16_lo:u8  crc16_hi:u8 ) 0x00
                             └── CRC over (type, payload) ──┘
```

* `0x00` is the inter-frame delimiter. COBS guarantees the body never
  contains `0x00`, so a reader resyncs to frame boundaries by reading
  until the next `0x00`.
* CRC is computed over the **un-encoded** body excluding the CRC bytes
  themselves (`crc16_ccitt_false(body, 1 + payload_len)` in
  `firmware/lib/frame_codec/frame_codec.cpp`; mirrored in
  `python/cctrack/_protocol.py`, which the tools import).
* Max decoded body is 256 bytes (`FRAME_MAX_BODY` in `frame_codec.h`;
  `cdc_io` derives its RX buffer from this constant).
* There is no sequence/correlation byte. Commands are single-shot
  request/response: the MCU echoes `cmd_id` (and reports `status`) in
  the ACK, which is what the host correlates against. If you ever need
  pipelined commands or out-of-order responses, this is the place that
  would have to grow.

### 1.2 Frame types

| Type | Name | Direction | Purpose |
|---|---|---|---|
| `0x01` | `QUAT` | MCU → host | Orientation sample, ~400 Hz (full BNO rate; host decimates as needed) |
| `0x02` | `STATUS` | MCU → host | Discrete events (boot, button, errors) |
| `0x10` | `CMD` | host → MCU | Request: ping, rezero, get/set config, reboot, … |
| `0x11` | `CMD_ACK` | MCU → host | Response to `0x10`: status + optional payload |

### 1.3 `0x01` QUAT payload (24 bytes)

```
offset  size  field         notes
  0     u8    flags         bit0=rezeroed_this_session (sticky true once any rezero has happened, incl. the automatic boot rezero); bits1-7 reserved, always 0
  1     u8    cal_accuracy  0..3 (BNO085 accuracy estimate)
  2     f32   qw            already mount-corrected, re-zeroed, and (optionally) conjugated on the MCU
  6     f32   qx
 10     f32   qy
 14     f32   qz
 18     u32   ts_us         MCU `micros()` timestamp (wraps; host can detect)
 22     u8×2  reserved      zero-filled, reserved for forward-compatibility
```

Bits 1-7 of `flags` are reserved and always 0.

CDC streams quaternions at the sensor's native ~400 Hz; the firmware
does not throttle this channel (CDC is the high-fidelity stream — MIDI
is the downstream that gets rate-limited via `midi_rate_hz`). Hosts that
need fewer samples should decimate on their side.

QUAT frames are dropped on the MCU when the TinyUSB CDC TX FIFO drops
below 128 bytes free (`send_quat` in `firmware/src/cdc_io.cpp`) — this reserves
headroom so a host command ACK is never starved by the high-rate
quaternion stream. The host treats dropped frames as benign; the next
frame just arrives.

**Stale frames after opening the port.** The MCU keeps writing frames
into the TinyUSB CDC TX FIFO while no host reads the port. When a host
opens it, the first few frames delivered are left over from before — in
testing about six frames, with a jump of several hundred milliseconds
in `ts_us` before the live stream. Hosts that care about freshness
should discard frames until `ts_us` advances continuously (≈2.5 ms
steps), or simply ignore the first ~50 ms of data. `reset_input_buffer()`
on the host does not help: the bytes are still on the device side.

Streaming can be suppressed entirely with `Config.stream_quat_cdc=false`
(see §2). With it off, the command channel and `STATUS` events still
work — only the high-rate QUAT firehose is silenced.

#### Host decoder example (Python)

The [`cctrack`](../python/README.md) library does all of this for you
(`Tracker().get_quat()`); the sketch below shows what it takes to decode
the stream from scratch.

Read the CDC port, resync on the `0x00` delimiter, COBS-decode, verify
the CRC, and unpack the QUAT payload. The COBS/CRC helpers are generic
to every frame type; only the final `struct.unpack_from` is
QUAT-specific. A full runnable version (with STATUS/ACK decoding and
print throttling) lives in `tools/cctrack-monitor.py`.

```python
import struct
import serial   # pyserial

def cobs_decode(data: bytes):
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            return None                       # malformed frame
        out.extend(data[i + 1:i + code])
        i += code
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)

def crc16_ccitt_false(data: bytes) -> int:    # poly 0x1021, init 0xFFFF
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def quat_frames(port: str, baud: int = 115200):
    """Yield (flags, accuracy, (qw,qx,qy,qz), ts_us) for each QUAT frame."""
    ser = serial.Serial(port, baud, timeout=0.1)
    buf = bytearray()
    while True:
        for b in ser.read(256):
            if b != 0:                        # accumulate until delimiter
                buf.append(b)
                continue
            frame, buf = bytes(buf), bytearray()
            if not frame:
                continue
            body = cobs_decode(frame)         # body = type ‖ payload ‖ crc16(LE)
            if body is None or len(body) < 3:
                continue
            crc_seen = body[-2] | (body[-1] << 8)
            if crc_seen != crc16_ccitt_false(body[:-2]):
                continue                      # drop corrupt frame
            type_, payload = body[0], body[1:-2]
            if type_ != 0x01 or len(payload) < 22:
                continue                      # not a QUAT frame
            flags, acc, qw, qx, qy, qz, ts_us = \
                struct.unpack_from('<BBffffI', payload, 0)
            yield flags, acc, (qw, qx, qy, qz), ts_us

if __name__ == '__main__':
    for flags, acc, (qw, qx, qy, qz), ts_us in quat_frames('/dev/ttyACM0'):
        print(f'q=({qw:+.4f}, {qx:+.4f}, {qy:+.4f}, {qz:+.4f}) '
              f'acc={acc} flags={flags:02x} ts={ts_us}us')
```

### 1.4 `0x02` STATUS payload (variable, ≤66 bytes)

```
offset  size       field      notes
  0     u8         code       status code (firmware-specific)
  1     i32 LE     arg        numeric context (e.g. error code, axis index)
  5     u8         msg_len    0..60
  6     u8×N       msg        UTF-8 text, no NUL terminator on the wire
```

### 1.5 `0x10` CMD payload (variable, host → MCU)

```
offset  size       field         notes
  0     u8         cmd_id        see table below
  1     u8×N       cmd_payload   command-specific (empty for most)
```

| `cmd_id` | Name | Payload | Notes |
|---|---|---|---|
| 1 | `rezero` | — | Snapshot current orientation as new zero |
| 4 | `save_calibration` | — | Persist BNO085 DCD calibration to its NVM |
| 5 | `get_config` | — | ACK payload = serialised `Config` blob (56 bytes, §2) |
| 6 | `set_config` | full `Config` blob (56 bytes; see §2 on back-compat) | Validates + persists; ACK has no payload |
| 7 | `factory_reset` | — | Equivalent to `set_config(config_default())` |
| 8 | `ping` | — | Liveness; ACK has no payload |
| 9 | `bootloader` | — | ACKs, then jumps to the UF2 bootloader |
| 10 | `version` | — | ACK payload = firmware version (see below) |

IDs 2 and 3 are gaps — they were `cal_start` / `cal_cancel` before BNO085
DCD (Dynamic Calibration during use) became always-on. They are
preserved so that older hosts get a clean `STATUS_ERR_UNSUPPORTED` from
the default switch case rather than triggering an unrelated command.

### 1.6 `0x11` CMD_ACK payload (variable, MCU → host)

```
offset  size  field        notes
  0     u8    cmd_id       echo of the originating command
  1     u8    status       see table below
  2     u8×N  ack_payload  `get_config`: the serialised Config blob; `version`: see below; otherwise empty
```

| `status` | Meaning |
|---|---|
| 0 | `STATUS_OK` |
| 1 | `STATUS_ERR_ARG` (malformed payload, schema mismatch) |
| 2 | `STATUS_ERR_STATE` |
| 3 | `STATUS_ERR_UNSUPPORTED` (unknown `cmd_id` or no hook installed) |

`version` ACK payload:

```
offset  size  field         notes
  0     u8    major         from the release tag vMAJOR.MINOR.PATCH (0.0.0 for untagged builds)
  1     u8    minor
  2     u8    patch
  3     u8    describe_len  0..40
  4     u8×N  describe      ASCII `git describe` string, e.g. "v0.1.0" or "v0.1.0-3-gabc1234-dirty"
```

The same version appears as the USB `bcdDevice` (BCD `0xJJMN`: major as two
digits, minor and patch as one digit each, clamped to 9).

---

## 2. `Config` blob (schema version 1, 56 bytes)

Same layout for `get_config` ACK and `set_config` payload. Defined in
`firmware/lib/config_schema/config_schema.h`.

```
offset  size       field              notes
  0     u8         schema_ver         currently 1; accepted on set_config if in 1..CONFIG_SCHEMA_VERSION
  1     u8         midi_channel       1..16
  2     u8         output_mode        0=off, 1=cc7, 2=cc14, 3=quat_4cc
  3     u8         (reserved)         always 0 on serialize; ignored on deserialize
  4     f32×3      mount_ypr          physical sensor displacement from the user-aligned pose, degrees, ZYX intrinsic (yaw,pitch,roll); firmware applies the inverse. Each angle must be finite and within ±360, else the blob is rejected
 16     u16        midi_rate_hz       MIDI emit rate; 0 = emit every sample. CDC is unthrottled — see §1.3.
 18     u8         conjugate_output   1 = conjugate the quaternion before mapping (reverses rotation direction)
 19     char[32]   device_name        NUL-padded ASCII; empty = auto-generated "CCTrack #XXXX"
 51     u8         enable_midi        1 = MIDI interface enumerated on next boot, 0 = CDC-only device
 52     u8         stream_quat_cdc    1 = stream QUAT frames over CDC, 0 = suppress (commands still served)
 53     u8         cc_yaw             CC number for yaw axis (default 16)
 54     u8         cc_pitch           CC number for pitch axis (default 17)
 55     u8         cc_roll            CC number for roll axis (default 18)
```

The config is persisted in the MCU's program flash, inside the firmware
image, so **flashing new firmware (upload or UF2) resets it to defaults**.

`set_config` accepts blobs whose `schema_ver` byte is in
`1..CONFIG_SCHEMA_VERSION` and whose length is at least 56 bytes (the v1
size). The firmware always **emits** the full layout of its running schema
version on `get_config`. Migration rule: **append-only**. A future version
must extend the trailing layout with new fields and define defaults that
preserve existing behaviour, so v1 blobs keep round-tripping and clients
that don't know the new fields keep working. The `CONFIG_SCHEMA_VERSION`
constant moves with each addition but a version mismatch alone is not a
reject — only an unknown (future) version or a too-short blob is.

---

## 3. USB-MIDI output

Class-compliant USB-MIDI, one cable, no driver needed on any OS. The MCU
emits MIDI as a side-effect of running the mapper on every sensor
sample. Implementation: `firmware/lib/midi_mapper/midi_mapper.cpp`,
emitted via `firmware/src/midi_io.cpp` → `g_midi_dev.write()`.

### 3.1 Output modes

| `output_mode` | Emits | Channel use |
|---|---|---|
| `0` `off` | nothing | (mapper short-circuits — the MIDI interface still enumerates; the host just sees a silent port) |
| `1` `cc7` | three 7-bit CCs: yaw on `cc_yaw`, pitch on `cc_pitch`, roll on `cc_roll` | 1 message per changed axis |
| `2` `cc14` | three 14-bit CC pairs: MSB on `cc_yaw/cc_pitch/cc_roll`, LSB on `cc_axis+32` (standard MIDI 14-bit convention) | 2 messages per changed axis |
| `3` `quat_4cc` | four 14-bit CC pairs for `qw, qx, qy, qz` on `cc_yaw+0..3` (consecutive from cc_yaw) | 2 messages per changed component |

For both `cc14` and `quat_4cc`, each pair is transmitted **LSB first**
(the `cc_axis+32` message), then MSB (the base-CC message). The CC
*assignment* is the standard convention; the *order* is reversed
deliberately so the MSB — the message a host typically latches the
combined 14-bit value on — arrives last, after the LSB is already in
place.

`midi_channel` is encoded into the status byte as
`0xB0 | ((midi_channel - 1) & 0x0F)`.

### 3.2 Value mapping

* **CC7 (Euler):** `value = clamp((rad / π) · 64 + 64, 0, 127)`. ±π → 0..127, centre 64.
* **CC14 (Euler):** `value = clamp((rad / π) · 8192 + 8192, 0, 16383)`. ±π → 0..16383, centre 8192.
* **CC14 (quaternion):** `value = clamp((component + 1) / 2 · 16383, 0, 16383)`. [-1,+1] → 0..16383.

The Euler triplet is computed from the already-rotated quaternion using
ZYX intrinsic Tait-Bryan angles (yaw=Z, pitch=Y, roll=X), so yaw lives
in `e.yaw` regardless of mounting.

> **Known limitation (Euler CC7/CC14 modes):** near pitch ±90° (head tipped
> fully up or down) the ZYX extraction hits gimbal lock — yaw and roll become
> ill-conditioned and the corresponding CC values jump erratically. This is
> inherent to any Euler→CC mapping, not a firmware bug. For full-range head
> motion (e.g. looking straight up/down) use `output_mode = quat_4cc`, which
> maps the raw quaternion components and has no singularity.

### 3.3 Rate-limiting and dedup

* **Down-sample:** the mapper enforces `period_ms = 1000 / midi_rate_hz`
  between successive emits. The very first emit after boot always passes
  so the host sees a known initial value. This throttle only affects the
  MIDI interface — the CDC QUAT stream runs at the full sensor rate.
* **Dedup:** the mapper stores the last emitted *integer* value per axis
  and skips axes that didn't change. This typically cuts traffic ~10×
  when the head is still.
* `midi_rate_hz = 0` disables the down-sample (emit on every sample).
* **Stale messages on connect:** like the CDC stream (§1.3), MIDI written
  while no host application had the port open can be delivered first
  when one opens it — e.g. a burst of CCs from the boot-time re-zero.
  The next movement sends current values.
* **Choosing a rate:** the default `100` is a good fit for DAWs, which
  process MIDI once per audio buffer (typically every 5–10 ms). Higher
  rates cut the time a value is held (10 ms → 5 ms at 200 Hz) but
  multiply the parameter updates the receiving host or plugin has to
  process; not all of them cope well with that. Raise it only if your
  setup benefits measurably.

The mapper sees an already-processed quaternion — mount correction,
re-zero, and (optionally) conjugation are applied upstream in `main.cpp`
before the mapper runs. The mapper itself is pure math with no IO.

### 3.4 Enabling / disabling the MIDI interface

Two knobs interact:

* `Config.output_mode = OUTPUT_MODE_OFF` — MIDI interface still enumerates
  on the host, mapper short-circuits, no MIDI bytes go out. Useful when a
  DAW expects the device to be present but you want it silent.
* `Config.enable_midi = false` — MIDI interface is omitted from the USB
  descriptor entirely; the device enumerates as CDC-only. Useful when a
  DAW auto-binds every controller it sees and you don't want CCTrack
  showing up there at all.

`enable_midi=false` implies "no MIDI" regardless of `output_mode`. The
flag only takes effect on the next boot — toggling it at runtime via
`set_config` triggers a USB reattach (so CDC reconnects cleanly) but
adding/removing the whole MIDI interface is a boot-time decision in
Adafruit_TinyUSB. Power-cycle the device after flipping it.

---

## 4. How `cctrack-cmd.py` talks to the MCU

`cctrack-cmd.py` is a thin client over the CDC frame protocol from §1.
It uses the COBS/CRC/frame helpers from `python/cctrack/_protocol.py`
(imported from the checkout, no install needed); the Config codec lives
in the script itself.

### 4.1 Connect

```python
ser = serial.Serial(port, 115200, timeout=0.1)
ser.reset_input_buffer()   # flush before sending; avoids CDC stalls
time.sleep(0.05)           # let USB IN transfers establish
```

Baud is nominal — TinyUSB CDC ignores it — but the host's `pyserial`
needs a value. `reset_input_buffer()` before the first write is
important: any stale QUAT bytes already buffered would otherwise be
parsed as garbage frames.

### 4.2 Round-trip a simple command (`ping`, `rezero`, …)

```
build_frame(type=0x10, payload=[cmd_id])
   = cobs([0x10, cmd_id, crc_lo, crc_hi]) + 0x00
send → MCU
read_frames() until timeout, scanning for type=0x11 with status=0
```

The script retries up to `--retries` times (default 3) with a 500 ms
gap. It accepts the first valid ACK and ignores the QUAT/STATUS frames
that arrive interleaved in the buffer.

Example (`tools/cctrack-cmd.py /dev/ttyACM0 ping`):

```
host → MCU :  0x10 0x08         (CMD, cmd_id=ping)             — wrapped in COBS + CRC + 0x00
MCU  → host:  0x11 0x08 0x00    (CMD_ACK, cmd_id=ping, status=OK) — wrapped likewise
```

### 4.3 `set_config` is a read-modify-write

`set_config` requires sending the full Config blob, so the script first
calls `get_config` to fetch the current config, applies CLI overrides
to the Python dict, re-serialises, and sends `set_config`:

```
ping?            no — directly:
get_config       → ACK carries 56-byte Config blob
decode_config_raw()  → dict
for KEY=VALUE in argv: apply_override(cfg, ...)
encode_config()      → 56 bytes
set_config       → ACK status=0 (or 1 if schema/length/range wrong); tool exits 1 on any error
```

Field keys recognised by `apply_override` in `tools/cctrack-cmd.py` (each
range-checked before sending; out-of-range values are an error, never wrapped):
`output_mode` (accepts `off`/`cc7`/`cc14`/`quat_4cc` or 0..3),
`conjugate_output` (0/1), `midi_channel` (1..16), `midi_rate_hz` (0..65535),
`cc_yaw`, `cc_pitch`, `cc_roll` (0..127; set each axis CC independently),
`cc_base` (0..125; shorthand: sets `cc_yaw=N cc_pitch=N+1 cc_roll=N+2`),
`mount_ypr` (`yaw,pitch,roll` in degrees, each finite and within ±360),
`device_name` (≤31 ASCII chars),
`enable_midi` (0/1), `stream_quat_cdc` (0/1).

### 4.4 `bootloader`

ACKs, pumps USB for 30 ms so the ACK is delivered, writes the bootloader magic value (`0xF01669EF`) at
`0x20007FFC`, and triggers `NVIC_SystemReset()`
(`command_processor.cpp`, `CMD_REBOOT_TO_UF2`). The board re-enumerates as the UF2
bootloader, exposing `QTPY_BOOT` as a USB mass-storage drive. The host
script just sends the command and exits — the next reconnect is on a
different USB device.

### 4.5 Reading the live stream

For passive observation (no commands), use `tools/cctrack-monitor.py`,
which decodes every frame type and pretty-prints. It also throttles
QUAT prints to `--quat-rate` Hz (default 10) so terminals stay readable
under the ~400 Hz sensor cadence.

---

## 5. Invariants worth knowing

* **CRC scope is body-minus-CRC**, not the COBS-encoded bytes. Encode
  body → CRC → append → COBS. Decode reverses the order.
* **Frame boundary is `0x00`**, not a length prefix. A truncated USB
  write (no trailing `0x00`) merges two frames into a CRC error on the
  host. The firmware guards against this by checking
  `tud_cdc_write_available()` before every `send_frame` and dropping
  the frame if there isn't room (`send_frame` in `cdc_io.cpp`).
* **Schema bumps are append-only.** `config_deserialize` accepts any
  payload whose `schema_ver` byte is in `1..CONFIG_SCHEMA_VERSION` and
  whose length is `≥ CONFIG_SERIALIZED_MIN` (the v1 size). Trailing
  fields added after the blob's version default to `config_default()`.
  New fields must keep this rule — bumping the constant alone is never
  enough; the deserializer must handle the shorter, older layout.
* **The MCU does not buffer commands**. Each `0x10` frame is processed
  to completion in the main loop tick that drains it; there is no
  command queue. The ACK is sent in that same tick. If the TX FIFO has
  no room for it at that moment, `send_frame` **drops** it — there is
  no retry on the MCU — so hosts should time out and resend, as
  `cctrack-cmd.py` does for simple commands (`--retries`; `set_config`
  is sent once, and a missing ACK is reported as an error).
