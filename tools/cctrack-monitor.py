#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Decode COBS+CRC frames from CCTrack CDC and print human-readable lines.

Usage:
    tools/cctrack-monitor.py /dev/ttyACM0
    tools/cctrack-monitor.py --help
"""
import argparse
import os
import struct
import sys
import time

import serial

# Share the wire-protocol helpers with the in-repo cctrack package.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
from cctrack import _protocol as proto  # noqa: E402


def parse_frame(raw: bytes):
    return proto.parse_frame(proto.cobs_decode(raw))


def fmt_quat(payload: bytes) -> str:
    if len(payload) < 22:
        return f"QUAT  <short payload: {len(payload)} bytes>"
    flags, cal_acc = payload[0], payload[1]
    qw, qx, qy, qz = struct.unpack_from('<ffff', payload, 2)
    ts_us = struct.unpack_from('<I', payload, 18)[0]
    return (f"QUAT  flags={flags:02x} acc={cal_acc} "
            f"q=({qw:+.4f}, {qx:+.4f}, {qy:+.4f}, {qz:+.4f}) ts={ts_us}us")


def fmt_status(payload: bytes) -> str:
    if len(payload) < 6:
        return f"STATUS <short payload: {len(payload)} bytes>"
    code = payload[0]
    arg  = struct.unpack_from('<i', payload, 1)[0]
    mlen = payload[5]
    msg  = payload[6:6 + mlen].decode('utf-8', 'replace')
    return f"STATUS code={code} arg={arg} msg={msg!r}"


def fmt_cmd_ack(payload: bytes) -> str:
    if len(payload) < 2:
        return f"CMD_ACK <short payload: {len(payload)} bytes>"
    return (f"CMD_ACK cmd={payload[0]} status={payload[1]} "
            f"payload={payload[2:].hex()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--quat-rate', type=float, default=10.0,
                    help='throttle QUAT prints to this many Hz (default 10)')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f'error: cannot open {args.port}: {e}', file=sys.stderr)
        return 1
    buf = bytearray()
    last_quat = 0.0
    interval = 1.0 / args.quat_rate

    while True:
        try:
            chunk = ser.read(256)
        except serial.SerialException:
            # Unplug, cable glitch or USB re-enumeration: the port is gone.
            print(f'error: device disconnected ({args.port})', file=sys.stderr)
            return 1
        if not chunk:
            continue
        for b in chunk:
            if b == 0:
                if buf:
                    f = parse_frame(bytes(buf))
                    if f is not None:
                        t, pl = f
                        if t == 0x01:
                            now = time.monotonic()
                            if now - last_quat >= interval:
                                print(fmt_quat(pl), flush=True)
                                last_quat = now
                        elif t == 0x02:
                            print(fmt_status(pl), flush=True)
                        elif t == 0x11:
                            print(fmt_cmd_ack(pl), flush=True)
                        else:
                            print(f"FRAME type=0x{t:02x} "
                                  f"payload={pl.hex()}", flush=True)
                    buf.clear()
            else:
                buf.append(b)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
