# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Pure (no-I/O) encoding and decoding of the CCTrack CDC wire protocol.

The wire format is documented in ``docs/protocol.md``. A frame on the wire is::

    cobs(type:u8 | payload[...] | crc16_lo:u8 crc16_hi:u8) 0x00

where the CRC is CRC-16/CCITT-FALSE over ``type + payload`` (little-endian),
and ``0x00`` delimits frames. This module is a self-contained copy of that
logic so the package has no dependency on the firmware tree.
"""

from __future__ import annotations

import struct
from collections import namedtuple

# Frame type IDs (see docs/protocol.md).
TYPE_QUAT = 0x01
TYPE_STATUS = 0x02
TYPE_CMD = 0x10
TYPE_CMD_ACK = 0x11

# QUAT payload: flags(u8) cal_accuracy(u8) qw qx qy qz(4×f32) ts_us(u32) +2 reserved.
_QUAT_STRUCT = struct.Struct("<BBffffI")  # 22 bytes; trailing 2 reserved ignored
QUAT_PAYLOAD_MIN = _QUAT_STRUCT.size

#: One decoded orientation sample.
#:
#: ``quat`` is ``(qw, qx, qy, qz)``; ``recv_time`` is the host ``time.monotonic()``
#: at which the frame was decoded (useful for measuring staleness).
Sample = namedtuple("Sample", "quat flags accuracy ts_us recv_time")


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode ``data``. The result contains no ``0x00`` and no delimiter."""
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes):
    """Decode one COBS block (the bytes between ``0x00`` delimiters).

    Returns the decoded body, or ``None`` if the block is malformed.
    """
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0 or i + code > n:
            return None  # 0x00 is never legal mid-block; or truncated run
        out.extend(data[i + 1 : i + code])
        i += code
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection/xor-out).

    The standard check value over ``b"123456789"`` is ``0x29B1``.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def parse_frame(body: bytes):
    """Validate a COBS-decoded frame body and split it.

    ``body`` is ``type | payload | crc16_le``. Returns ``(type, payload)`` if
    the CRC checks out, else ``None``.
    """
    if body is None or len(body) < 3:
        return None
    crc_seen = body[-2] | (body[-1] << 8)
    if crc_seen != crc16_ccitt_false(body[:-2]):
        return None
    return body[0], body[1:-2]


def build_frame(ftype: int, payload: bytes = b"") -> bytes:
    """Return a complete wire frame: COBS(type | payload | crc16_le) + ``0x00``."""
    body = bytes([ftype]) + bytes(payload)
    body += struct.pack("<H", crc16_ccitt_false(body))
    return cobs_encode(body) + b"\x00"


def parse_quat(payload: bytes, recv_time: float = 0.0):
    """Decode a QUAT payload into a :class:`Sample`, or ``None`` if too short."""
    if len(payload) < QUAT_PAYLOAD_MIN:
        return None
    flags, acc, qw, qx, qy, qz, ts_us = _QUAT_STRUCT.unpack_from(payload, 0)
    return Sample(
        quat=(qw, qx, qy, qz),
        flags=flags,
        accuracy=acc,
        ts_us=ts_us,
        recv_time=recv_time,
    )
