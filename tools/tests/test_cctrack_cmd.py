# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Config override validation and blob encoding for tools/cctrack-cmd.py."""

import importlib.util
import math
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parents[1] / "cctrack-cmd.py"
_spec = importlib.util.spec_from_file_location("cctrack_cmd", _SCRIPT)
cmd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cmd)


def default_cfg():
    return {
        "schema_ver": 1, "midi_channel": 1, "output_mode": 2,
        "mount_ypr": [-90.0, 0.0, 0.0], "midi_rate_hz": 100,
        "conjugate_output": 1, "device_name": "",
        "enable_midi": True, "stream_quat_cdc": True,
        "cc_yaw": 16, "cc_pitch": 17, "cc_roll": 18,
    }


@pytest.mark.parametrize("key,value,expected", [
    ("midi_channel", "16", {"midi_channel": 16}),
    ("midi_rate_hz", "0", {"midi_rate_hz": 0}),
    ("midi_rate_hz", "65535", {"midi_rate_hz": 65535}),
    ("cc_yaw", "127", {"cc_yaw": 127}),
    ("cc_base", "0x10", {"cc_yaw": 16, "cc_pitch": 17, "cc_roll": 18}),
    ("cc_base", "125", {"cc_yaw": 125, "cc_pitch": 126, "cc_roll": 127}),
    ("output_mode", "quat_4cc", {"output_mode": 3}),
    ("output_mode", "0", {"output_mode": 0}),
    ("enable_midi", "0", {"enable_midi": False}),
    ("mount_ypr", "-360,0,360", {"mount_ypr": [-360.0, 0.0, 360.0]}),
    ("device_name", "Studio A", {"device_name": "Studio A"}),
])
def test_valid_overrides(key, value, expected):
    cfg = default_cfg()
    cmd.apply_override(cfg, key, value)
    for k, v in expected.items():
        assert cfg[k] == v


@pytest.mark.parametrize("key,value", [
    ("midi_channel", "0"),
    ("midi_channel", "17"),
    ("midi_rate_hz", "65536"),
    ("cc_yaw", "200"),            # used to be masked silently to 72
    ("cc_roll", "-1"),
    ("cc_base", "126"),           # roll would be 128
    ("output_mode", "4"),
    ("output_mode", "sysex"),
    ("enable_midi", "2"),
    ("mount_ypr", "nan,0,0"),
    ("mount_ypr", "inf,0,0"),
    ("mount_ypr", "0,360.5,0"),
    ("mount_ypr", "0,0"),
    ("mount_ypr", "a,b,c"),
    ("device_name", "x" * 32),
    ("device_name", "Café"),
    ("midi_channel", "one"),
    ("no_such_key", "1"),
])
def test_invalid_overrides_raise(key, value):
    cfg = default_cfg()
    before = dict(cfg)
    with pytest.raises(ValueError):
        cmd.apply_override(cfg, key, value)
    assert cfg == before


def test_config_blob_roundtrip():
    cfg = default_cfg()
    cmd.apply_override(cfg, "device_name", "CCTrack Test")
    cmd.apply_override(cfg, "cc_base", "20")
    blob = cmd.encode_config(cfg)
    assert len(blob) == 56
    out = cmd.decode_config_raw(blob)
    assert out["device_name"] == "CCTrack Test"
    assert (out["cc_yaw"], out["cc_pitch"], out["cc_roll"]) == (20, 21, 22)
    assert out["midi_rate_hz"] == 100
    assert all(math.isclose(a, b) for a, b in zip(out["mount_ypr"], cfg["mount_ypr"]))


def test_version_ack_is_printed_and_ok(capsys):
    payload = bytes([0, 1, 2, 6]) + b"v0.1.2"
    body = bytes([0x11, cmd.CMD["version"], 0]) + payload
    body += cmd.proto.crc16_ccitt_false(body).to_bytes(2, "little")
    assert cmd.print_response([body], "version") is True
    assert capsys.readouterr().out.strip() == "0.1.2 (v0.1.2)"


def test_error_status_and_no_response_are_failures():
    body = bytes([0x11, cmd.CMD["set_config"], 1])
    body += cmd.proto.crc16_ccitt_false(body).to_bytes(2, "little")
    assert cmd.print_response([body], "set_config") is False
    assert cmd.print_response([], "ping") is False
