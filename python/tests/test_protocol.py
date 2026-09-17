# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Wire-protocol decode tests.

The COBS vectors mirror firmware/test/native/test_cobs, and the CRC check
value is the standard CRC-16/CCITT-FALSE constant.
"""

import struct

from cctrack import _protocol as proto


# -- CRC-16/CCITT-FALSE ----------------------------------------------------
def test_crc_check_value():
    assert proto.crc16_ccitt_false(b"123456789") == 0x29B1


def test_crc_empty():
    assert proto.crc16_ccitt_false(b"") == 0xFFFF


# -- COBS decode (mirrors firmware reference vectors) ----------------------
def test_cobs_single_zero():
    assert proto.cobs_decode(bytes([0x01, 0x01])) == bytes([0x00])


def test_cobs_two_nonzero():
    assert proto.cobs_decode(bytes([0x03, 0x11, 0x22])) == bytes([0x11, 0x22])


def test_cobs_zero_in_middle():
    assert proto.cobs_decode(bytes([0x02, 0x11, 0x02, 0x33])) == bytes([0x11, 0x00, 0x33])


def test_cobs_rejects_zero_in_input():
    assert proto.cobs_decode(bytes([0x02, 0x11, 0x00])) is None


def test_cobs_rejects_truncated_run():
    assert proto.cobs_decode(bytes([0x04, 0x11])) is None


# -- COBS encode / frame build ----------------------------------------------
def test_cobs_encode_reference_vectors():
    assert proto.cobs_encode(bytes([0x00])) == bytes([0x01, 0x01])
    assert proto.cobs_encode(bytes([0x11, 0x22])) == bytes([0x03, 0x11, 0x22])
    assert proto.cobs_encode(bytes([0x11, 0x00, 0x33])) == bytes([0x02, 0x11, 0x02, 0x33])


def test_cobs_roundtrip_long_runs():
    for data in (bytes(range(1, 256)), bytes([0xAB]) * 600, b"\x00" * 5, b""):
        enc = proto.cobs_encode(data)
        assert 0 not in enc
        assert proto.cobs_decode(enc) == data


def test_build_frame_roundtrip():
    payload = bytes([0x05, 0x00, 0x42])
    frame = proto.build_frame(proto.TYPE_CMD, payload)
    assert frame.endswith(b"\x00") and 0 not in frame[:-1]
    assert proto.parse_frame(proto.cobs_decode(frame[:-1])) == (proto.TYPE_CMD, payload)


# -- frame + QUAT round trip ----------------------------------------------
def _build_quat_payload(flags, acc, quat, ts_us):
    qw, qx, qy, qz = quat
    return struct.pack("<BBffffI", flags, acc, qw, qx, qy, qz, ts_us) + b"\x00\x00"


def test_parse_frame_good_crc():
    payload = _build_quat_payload(0x01, 3, (1.0, 0.0, 0.0, 0.0), 12345)
    body = bytes([proto.TYPE_QUAT]) + payload
    crc = proto.crc16_ccitt_false(body)
    body += struct.pack("<H", crc)
    parsed = proto.parse_frame(body)
    assert parsed is not None
    ftype, out_payload = parsed
    assert ftype == proto.TYPE_QUAT
    assert out_payload == payload


def test_parse_frame_bad_crc():
    body = bytes([proto.TYPE_QUAT]) + _build_quat_payload(0, 0, (1, 0, 0, 0), 0)
    body += b"\x00\x00"  # deliberately wrong CRC
    assert proto.parse_frame(body) is None


def test_parse_quat_roundtrip():
    payload = _build_quat_payload(0x05, 2, (0.5, -0.5, 0.5, -0.5), 999)
    s = proto.parse_quat(payload, recv_time=1.5)
    assert s is not None
    assert s.flags == 0x05
    assert s.accuracy == 2
    assert s.ts_us == 999
    assert s.recv_time == 1.5
    assert s.quat == (0.5, -0.5, 0.5, -0.5)


def test_parse_quat_too_short():
    assert proto.parse_quat(b"\x00\x01", recv_time=0.0) is None
