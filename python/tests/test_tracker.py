# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Reader-thread disconnect handling.

Regression: the reader thread used to exit silently on ``SerialException``,
leaving accessors returning stale data forever with no way to tell "head not
moving" from "tracker unplugged". A Tracker whose reader died also could not
be reopened, because ``open()`` saw the dead thread and returned early.
"""

import threading
import time

import serial

from cctrack.tracker import Tracker


class _RaisingSerial:
    def read(self, size):
        raise serial.SerialException("device disconnected")

    def close(self):
        pass


def test_reader_thread_records_disconnect_and_clears_alive():
    t = Tracker(port="/dev/fake")
    t._ser = _RaisingSerial()
    t._thread = threading.Thread(target=t._run, daemon=True)
    t._thread.start()
    t._thread.join(timeout=1.0)

    assert not t.alive
    assert isinstance(t.last_error, serial.SerialException)


class _IdleSerial:
    def __init__(self):
        self.closed = False

    def read(self, size):
        return b""

    def close(self):
        self.closed = True


def test_open_after_disconnect_reconnects(monkeypatch):
    opened = []

    def fake_serial(port, baud, timeout):
        ser = _RaisingSerial() if not opened else _IdleSerial()
        opened.append(ser)
        return ser

    monkeypatch.setattr(serial, "Serial", fake_serial)

    t = Tracker(port="/dev/fake").open()
    t._thread.join(timeout=1.0)
    assert not t.alive

    t.open()  # must reconnect instead of returning early on the dead thread
    try:
        assert len(opened) == 2
        assert t.alive
        assert t.last_error is None
    finally:
        t.close()
    assert opened[1].closed


def test_non_serial_exception_is_recorded():
    class _OSErrorSerial(_RaisingSerial):
        def read(self, size):
            raise OSError("I/O error")

    t = Tracker(port="/dev/fake")
    t._ser = _OSErrorSerial()
    t._thread = threading.Thread(target=t._run, daemon=True)
    t._thread.start()
    t._thread.join(timeout=1.0)

    assert not t.alive
    assert isinstance(t.last_error, OSError)


def _quat_frame():
    import struct

    from cctrack import _protocol as proto

    payload = struct.pack("<BBffffI", 0x01, 3, 1.0, 0.0, 0.0, 0.0, 1234) + b"\x00\x00"
    return proto.build_frame(proto.TYPE_QUAT, payload)


class _StreamingSerial:
    """Delivers one QUAT frame after 100 ms (like a real port), then nothing."""

    def __init__(self, *args, **kwargs):
        self._data = _quat_frame()
        self._ready_at = time.monotonic() + 0.1
        self.closed = False

    def read(self, size):
        if time.monotonic() < self._ready_at:
            time.sleep(0.01)
            return b""
        chunk, self._data = self._data[:size], self._data[size:]
        return chunk

    def close(self):
        self.closed = True


def test_context_manager_waits_for_first_sample(monkeypatch):
    # Regression: `with Tracker() as t: t.get_quat()` returned None because the
    # context manager returned before the reader thread had decoded anything.
    monkeypatch.setattr(serial, "Serial", _StreamingSerial)
    with Tracker(port="/dev/fake") as t:
        assert t.get_quat() == (1.0, 0.0, 0.0, 0.0)


def test_context_manager_raises_and_closes_without_data(monkeypatch):
    from cctrack import TrackerError

    opened = []

    def silent_serial(*args, **kwargs):
        ser = _IdleSerial()
        opened.append(ser)
        return ser

    monkeypatch.setattr(serial, "Serial", silent_serial)
    t = Tracker(port="/dev/fake", wait_timeout=0.2)
    try:
        with t:
            raise AssertionError("body must not run without a sample")
    except TrackerError:
        pass
    assert not t.alive
    assert opened[0].closed


def test_reopen_does_not_reuse_sample_from_previous_connection(monkeypatch):
    monkeypatch.setattr(serial, "Serial", _StreamingSerial)
    t = Tracker(port="/dev/fake")
    with t:
        assert t.latest() is not None
    monkeypatch.setattr(serial, "Serial", lambda *a, **k: _IdleSerial())
    t.open()
    try:
        assert t.latest() is None
        assert not t.wait(timeout=0.1)
    finally:
        t.close()
