#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Send a CMD frame to a CCTrack device and print the response.

Usage:
    tools/cctrack-cmd.py /dev/ttyACM0 ping
    tools/cctrack-cmd.py /dev/ttyACM0 get_config
    tools/cctrack-cmd.py /dev/ttyACM0 set_config mount_ypr=0,90,0      # sensor physically pitched 90° (yaw,pitch,roll deg, ZYX); firmware applies the inverse
    tools/cctrack-cmd.py /dev/ttyACM0 set_config midi_channel=2 midi_rate_hz=100
    tools/cctrack-cmd.py /dev/ttyACM0 set_config cc_yaw=20 cc_pitch=35 cc_roll=40
    tools/cctrack-cmd.py /dev/ttyACM0 set_config cc_base=16   # shorthand: sets cc_yaw=16 cc_pitch=17 cc_roll=18
    tools/cctrack-cmd.py /dev/ttyACM0 version
    tools/cctrack-cmd.py /dev/ttyACM0 bootloader
"""
import argparse
import math
import os
import struct
import sys
import time

import serial

# Share the wire-protocol helpers with the in-repo cctrack package, so the
# tools can't drift from the library (no install needed when run from the repo).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
from cctrack import _protocol as proto  # noqa: E402


# Ordered most-used → rarest, destructive ops last (drives --help / choices display).
# The numeric IDs are the wire protocol and are fixed regardless of this order;
# IDs 2 and 3 were cal_start / cal_cancel before DCD became always-on.
CMD = {
    'ping':              8,
    'version':          10,
    'get_config':        5,
    'set_config':        6,
    'rezero':            1,
    'save_calibration':  4,   # persist BNO085 DCD calibration to sensor NVM
    'factory_reset':     7,
    'bootloader':        9,
}

OUTPUT_MODES = {0: 'off', 1: 'cc7', 2: 'cc14', 3: 'quat_4cc'}
MOUNT_YPR_LIMIT_DEG = 360.0   # firmware rejects non-finite or larger angles
_OUTPUT_MODE_BY_NAME = {v: k for k, v in OUTPUT_MODES.items()}


def build_frame(cmd_id: int, payload: bytes = b'') -> bytes:
    return proto.build_frame(proto.TYPE_CMD, bytes([cmd_id]) + payload)


def read_frames(ser, timeout=2.0):
    buf = bytearray()
    frames = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            # Read only what's already buffered (fall back to a blocking 1-byte
            # read when idle). A fixed ser.read(256) accumulates internally and
            # is discarded whole if the port drops mid-call — which is exactly
            # what happens on a device_name/enable_midi change, where the ACK
            # arrives and is then lost when the USB reattach disconnects the
            # port. Reading what's waiting commits each frame as it arrives.
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException:
            # The USB reattach dropped the port; the ACK is already in `frames`.
            break
        for b in chunk:
            if b == 0:
                if buf:
                    body = proto.cobs_decode(bytes(buf))
                    if body and len(body) >= 3:
                        frames.append(body)
                buf.clear()
            else:
                buf.append(b)
    return frames


def decode_config_raw(payload: bytes):
    # 56-byte v1 layout (see docs/protocol.md §2). The schema is append-only,
    # so a *longer* blob from a future firmware still parses; shorter is malformed.
    if len(payload) < 56:
        return None
    i = 0
    cfg = {}
    cfg['schema_ver']         = payload[i]; i += 1
    cfg['midi_channel']       = payload[i]; i += 1
    cfg['output_mode']        = payload[i]; i += 1
    i += 1  # offset 3: reserved byte, ignored
    cfg['mount_ypr']          = list(struct.unpack_from('<fff', payload, i)); i += 12
    cfg['midi_rate_hz'],      = struct.unpack_from('<H', payload, i); i += 2
    cfg['conjugate_output']   = payload[i]; i += 1
    cfg['device_name']        = payload[i:i+32].rstrip(b'\x00').decode('ascii', errors='replace')
    i += 32
    cfg['enable_midi']        = bool(payload[i]); i += 1
    cfg['stream_quat_cdc']    = bool(payload[i]); i += 1
    cfg['cc_yaw']             = payload[i]; i += 1
    cfg['cc_pitch']           = payload[i]; i += 1
    cfg['cc_roll']            = payload[i]
    return cfg


def encode_config(cfg: dict) -> bytes:
    blob = bytearray()
    blob.append(cfg['schema_ver'])
    blob.append(cfg['midi_channel'])
    blob.append(cfg['output_mode'])
    blob.append(0)  # offset 3: reserved, always 0
    blob.extend(struct.pack('<fff', *cfg['mount_ypr']))
    blob.extend(struct.pack('<H', cfg['midi_rate_hz']))
    blob.append(1 if cfg.get('conjugate_output', True) else 0)
    name_bytes = cfg.get('device_name', '').encode('ascii', errors='replace')[:31]
    blob.extend(name_bytes + b'\x00' * (32 - len(name_bytes)))
    blob.append(1 if cfg.get('enable_midi',     True) else 0)
    blob.append(1 if cfg.get('stream_quat_cdc', True) else 0)
    blob.append(cfg.get('cc_yaw',   16))
    blob.append(cfg.get('cc_pitch', 17))
    blob.append(cfg.get('cc_roll',  18))
    return bytes(blob)


def _int_in_range(key: str, value_str: str, lo: int, hi: int) -> int:
    try:
        v = int(value_str, 0)
    except ValueError:
        raise ValueError(f'{key} must be an integer, got {value_str!r}') from None
    if not lo <= v <= hi:
        raise ValueError(f'{key} must be in {lo}..{hi}, got {v}')
    return v


def apply_override(cfg: dict, key: str, value_str: str):
    # Validate here rather than masking: a silently wrapped value (e.g. cc 200
    # -> 72) would slip past the firmware's own range checks.
    if key == 'output_mode':
        lower = value_str.lower()
        if lower in _OUTPUT_MODE_BY_NAME:
            cfg[key] = _OUTPUT_MODE_BY_NAME[lower]
        else:
            names = ', '.join(OUTPUT_MODES.values())
            try:
                mode = int(value_str, 0)
            except ValueError:
                raise ValueError(f'output_mode must be one of {names} or 0..3, got {value_str!r}') from None
            if mode not in OUTPUT_MODES:
                raise ValueError(f'output_mode must be one of {names} or 0..3, got {mode}')
            cfg[key] = mode
    elif key in ('conjugate_output', 'enable_midi', 'stream_quat_cdc'):
        cfg[key] = bool(_int_in_range(key, value_str, 0, 1))
    elif key == 'midi_channel':
        cfg[key] = _int_in_range(key, value_str, 1, 16)
    elif key == 'midi_rate_hz':
        cfg[key] = _int_in_range(key, value_str, 0, 65535)
    elif key in ('cc_yaw', 'cc_pitch', 'cc_roll'):
        cfg[key] = _int_in_range(key, value_str, 0, 127)
    elif key == 'cc_base':
        # Convenience shorthand: three consecutive CCs starting at the base.
        base = _int_in_range(key, value_str, 0, 125)
        cfg['cc_yaw']   = base
        cfg['cc_pitch'] = base + 1
        cfg['cc_roll']  = base + 2
    elif key == 'mount_ypr':
        try:
            vals = [float(v) for v in value_str.split(',')]
        except ValueError:
            raise ValueError(f'mount_ypr values must be numbers, got {value_str!r}') from None
        if len(vals) != 3:
            raise ValueError('mount_ypr must be three values: yaw,pitch,roll (degrees, ZYX)')
        for v in vals:
            if not math.isfinite(v) or abs(v) > MOUNT_YPR_LIMIT_DEG:
                raise ValueError(f'mount_ypr angles must be finite and within ±{MOUNT_YPR_LIMIT_DEG:g}, got {v}')
        cfg[key] = vals
    elif key == 'device_name':
        if not value_str.isascii():
            raise ValueError('device_name must be ASCII')
        if len(value_str) > 31:
            raise ValueError('device_name must be 31 characters or fewer')
        cfg[key] = value_str
    else:
        raise ValueError(f'unknown config key: {key!r}')


def pretty_config(cfg: dict) -> dict:
    return {
        'schema_ver':       cfg['schema_ver'],
        'device_name':      cfg.get('device_name', ''),
        'mount_ypr':        [round(v, 2) for v in cfg['mount_ypr']],
        'conjugate_output': bool(cfg['conjugate_output']),
        'output_mode':      OUTPUT_MODES.get(cfg['output_mode'], str(cfg['output_mode'])),
        'midi_channel':     cfg['midi_channel'],
        'cc_yaw':           cfg.get('cc_yaw',   16),
        'cc_pitch':         cfg.get('cc_pitch',  17),
        'cc_roll':          cfg.get('cc_roll',   18),
        'midi_rate_hz':     cfg['midi_rate_hz'],
        'enable_midi':      bool(cfg.get('enable_midi',     True)),
        'stream_quat_cdc':  bool(cfg.get('stream_quat_cdc', True)),
    }


def get_cmd_ack_payload(frames):
    for body in frames:
        if len(body) < 3:
            continue
        if proto.crc16_ccitt_false(body[:-2]) != struct.unpack_from('<H', body, len(body) - 2)[0]:
            continue
        if body[0] == 0x11 and body[2] == 0:
            return body[3:-2]
    return None


def error(msg: str) -> None:
    print(f'error: {msg}', file=sys.stderr)


def print_response(frames, cmd_name) -> bool:
    """Print the device's reply. Returns True only for an ACK with status OK."""
    if not frames:
        error('no response')
        return False
    got_ack = False
    ok = False
    for body in frames:
        if len(body) < 3:
            continue
        t = body[0]
        expected_crc = struct.unpack_from('<H', body, len(body) - 2)[0]
        if proto.crc16_ccitt_false(body[:-2]) != expected_crc:
            # Silently skip: with the quat stream running, partial frames caught
            # mid-stream (e.g. the first fragment right after connecting) fail
            # CRC and are just noise. A genuinely lost ACK still surfaces below
            # as "no response".
            continue
        if t == 0x11:
            got_ack = True
            status  = body[2]
            payload = body[3:-2]
            if status != 0:
                error(f'status={status}')
                continue
            ok = True
            if cmd_name == 'version' and len(payload) >= 4:
                major, minor, patch, dlen = payload[0], payload[1], payload[2], payload[3]
                describe = payload[4:4 + dlen].decode('ascii', 'replace')
                print(f'{major}.{minor}.{patch} ({describe})')
            elif cmd_name == 'get_config' and payload:
                cfg = decode_config_raw(payload)
                if cfg:
                    for k, v in pretty_config(cfg).items():
                        print(f'  {k}: {v}')
            else:
                print('ok' + (f'  payload={payload.hex()}' if payload else ''))
        elif t == 0x02:
            msg = body[7:7 + body[6]].decode('utf-8', 'replace') if len(body) > 7 else ''
            print(f'STATUS code={body[1]} {msg!r}')
    if not got_ack:
        types = {body[0] for body in frames if len(body) >= 3}
        error(f'no response ({len(frames)} frame(s) received, types: {sorted(types)})')
    return ok


def do_get_config(ser, timeout):
    ser.reset_input_buffer()
    ser.write(build_frame(CMD['get_config']))
    ser.flush()
    frames = read_frames(ser, timeout=timeout)
    payload = get_cmd_ack_payload(frames)
    return decode_config_raw(payload) if payload is not None else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port')
    ap.add_argument('cmd', choices=list(CMD.keys()))
    ap.add_argument('fields', nargs='*', metavar='KEY=VALUE',
                    help='field overrides for set_config (reads current config first)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=2.0)
    ap.add_argument('--retries', type=int, default=3,
                    help='max attempts before giving up (default: 3)')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()   # flush before any data arrives (avoids CDC throttle)
    time.sleep(0.05)           # let USB IN transfers establish before sending

    if args.cmd == 'set_config':
        cfg = do_get_config(ser, args.timeout)
        if cfg is None:
            error('could not read current config')
            return 1
        for field in args.fields:
            if '=' not in field:
                error(f'expected KEY=VALUE, got {field!r}')
                return 1
            key, value_str = field.split('=', 1)
            try:
                apply_override(cfg, key.strip(), value_str.strip())
            except ValueError as e:
                error(str(e))
                return 1
        ser.write(build_frame(CMD['set_config'], encode_config(cfg)))
        ser.flush()
        frames = read_frames(ser, timeout=args.timeout)
        return 0 if print_response(frames, 'set_config') else 1

    frames = []
    for attempt in range(args.retries):
        if attempt > 0:
            time.sleep(0.5)
        ser.write(build_frame(CMD[args.cmd]))
        ser.flush()
        frames = read_frames(ser, timeout=args.timeout)
        if any(b[0] == 0x11 for b in frames if len(b) >= 3):
            break
    return 0 if print_response(frames, args.cmd) else 1


if __name__ == '__main__':
    sys.exit(main())
