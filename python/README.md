<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->
# cctrack — Python consumer library

Read the CCTrack head-tracker's orientation stream over USB-CDC with a
one-liner. Decodes the COBS + CRC-16 framed QUAT stream (see
[`../docs/protocol.md`](../docs/protocol.md)) and hands you quaternions or
yaw/pitch/roll. The only runtime dependency is `pyserial`.

## Install

```bash
# straight from this repo, without cloning the firmware:
pip install "git+https://github.com/mpollow/CCTrack#subdirectory=python"
# or, for local development:
cd python && pip install -e ".[dev]"
```

## Use

```python
from cctrack import Tracker

with Tracker() as t:                  # auto-detects the device, waits for the first sample
    yaw, pitch, roll = t.get_ypr()    # newest sample, non-blocking, degrees
    qw, qx, qy, qz   = t.get_quat()
```

For throwaway scripts and the REPL, module-level helpers open a shared default
tracker on first use:

```python
import cctrack
cctrack.get_ypr()      # (yaw, pitch, roll)
cctrack.get_quat()     # (qw, qx, qy, qz)
```

Pass `port="/dev/ttyACM0"` (or a Windows `COM5`) if auto-detect can't pick a
single device.

## How it stays low-latency

A background daemon thread continuously drains the serial port and keeps only
the **newest** sample. Your code polls `get_quat()` / `get_ypr()` at whatever
rate it likes — a slow render loop, an audio tick — and always gets the newest
sample: the library adds no queueing delay beyond the sensor period and USB
polling, and no backlog builds up in the OS receive buffer. `t.age()` reports the seconds
since the newest sample if you want to watch freshness.

Right after `open()`, the first few samples can be stale — frames the device
buffered before the port was opened (see `docs/protocol.md` §1.3). `age()`
can't detect that, because it measures host receive time. If it matters,
skip them, e.g. wait ~50 ms after `wait()` (or entering `with`) before trusting the data.

## API

| Call | Returns |
|------|---------|
| `Tracker(port=None, baud=115200, wait_timeout=2.0)` | tracker (not yet open) |
| `with Tracker() as t:` | opens port, starts reader thread, **waits for the first sample** (raises `TrackerError` after `wait_timeout` s) |
| `t.open()` | same, but returns immediately — getters return `None` until data arrives; reconnects if the reader stopped |
| `t.close()` | stops the reader thread, closes the port |
| `t.alive` / `t.last_error` | whether the reader is running / the exception that stopped it (e.g. unplugged) |
| `t.get_quat()` | `(qw, qx, qy, qz)` or `None` |
| `t.get_ypr(degrees=True)` | `(yaw, pitch, roll)` or `None` |
| `t.latest()` | `Sample(quat, flags, accuracy, ts_us, recv_time)` or `None` |
| `t.wait(timeout=None)` | block until first sample; `True` if one arrived |
| `t.age()` | seconds since newest sample, or `None` |
| `t.frames_ok` / `t.frames_bad` | decoded / dropped frame counters |
| `find_port()` | device path of the lone attached CCTrack |
| `quat_to_ypr(qw,qx,qy,qz, degrees=True)` | standalone converter |

Euler angles use the firmware's convention: ZYX intrinsic Tait-Bryan
(yaw about Z, pitch about Y, roll about X), pitch clamped at the poles.

## Example

```bash
python examples/print_ypr.py            # auto-detect
python examples/print_ypr.py /dev/ttyACM0
```

## Tests

```bash
pytest
```
