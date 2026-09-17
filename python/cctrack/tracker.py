# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Low-latency reader for the CCTrack orientation stream.

A background daemon thread continuously drains the serial port and keeps only
the *newest* QUAT sample, so accessors return floor-latency data regardless of
how fast (or slowly) the consumer polls — and the OS receive buffer never
backlogs. See ``docs/protocol.md`` for the wire format.
"""

from __future__ import annotations

import threading
import time

import serial
from serial.tools import list_ports

from . import _protocol as proto
from ._protocol import Sample
from .orientation import quat_to_ypr

# USB identity of the CCTrack composite device (see firmware usb descriptors).
USB_VID = 0x1209
USB_PID = 0xCC3D


class TrackerError(RuntimeError):
    """Raised for port-discovery and connection problems."""


def find_port() -> str:
    """Return the serial device path of the one attached CCTrack.

    Matches on USB VID/PID (``0x1209:0xCC3D``). Raises :class:`TrackerError`
    if none — or more than one — is found, so callers know to pass ``port=``
    explicitly.
    """
    matches = [p.device for p in list_ports.comports() if p.vid == USB_VID and p.pid == USB_PID]
    if not matches:
        raise TrackerError(
            "No CCTrack found (looked for USB %04x:%04x). "
            "Pass port= explicitly, or check the connection." % (USB_VID, USB_PID)
        )
    if len(matches) > 1:
        raise TrackerError(
            "Multiple CCTrack devices found: %s. Pass port= to choose one." % ", ".join(matches)
        )
    return matches[0]


class Tracker:
    """Streaming reader for one CCTrack device.

    Typical use::

        with Tracker() as t:              # waits for the first sample
            yaw, pitch, roll = t.get_ypr()

    Construction does not open the port. Entering the context manager opens
    it, starts the reader thread and blocks until the first sample arrives
    (up to ``wait_timeout`` seconds, else :class:`TrackerError`), so the
    accessors return data straight away. :meth:`open` does the same without
    waiting. All accessors are non-blocking and return ``None`` until the
    first valid sample arrives (use :meth:`wait` to block for it).

    If the device disconnects, the reader thread stops and accessors keep
    returning the last sample forever rather than raising — check
    :attr:`alive` (or :attr:`last_error`) alongside :meth:`age` to distinguish
    "head not moving" from "tracker unplugged". Calling :meth:`open` again
    after a disconnect reconnects.
    """

    def __init__(self, port: str | None = None, baud: int = 115200,
                 wait_timeout: float = 2.0):
        self.port = port
        self.baud = baud
        self.wait_timeout = wait_timeout
        self._ser: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._first = threading.Event()
        self._latest: Sample | None = None
        self._last_error: Exception | None = None
        self.frames_ok = 0
        self.frames_bad = 0

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> "Tracker":
        """Open the serial port and start the background reader thread."""
        if self._thread is not None:
            if self._thread.is_alive():
                return self  # already open
            self.close()  # reader stopped (e.g. unplugged): release it, then reopen
        port = self.port or find_port()
        self.port = port
        # Short read timeout so the thread can observe the stop flag promptly.
        self._ser = serial.Serial(port, self.baud, timeout=0.05)
        self._stop.clear()
        self._last_error = None
        # A (re)opened tracker starts fresh: wait()/`with` must see a sample
        # from this connection, not one left over from before a disconnect.
        self._first.clear()
        with self._lock:
            self._latest = None
        self._thread = threading.Thread(target=self._run, name="cctrack-reader", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        """Stop the reader thread and close the serial port."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "Tracker":
        self.open()
        if not self._first.wait(self.wait_timeout):
            self.close()
            raise TrackerError(
                "No orientation sample from %s within %.1f s. Is the device "
                "streaming (stream_quat_cdc=1)?" % (self.port, self.wait_timeout)
            )
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- reader thread -----------------------------------------------------
    def _run(self) -> None:
        ser = self._ser
        assert ser is not None
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = ser.read(256)  # blocks up to the read timeout
            except Exception as exc:  # SerialException on unplug; OSError etc. on some platforms
                self._last_error = exc
                break
            if not chunk:
                continue
            for b in chunk:
                if b != 0x00:
                    buf.append(b)
                    continue
                # delimiter: decode the accumulated block, then reset
                block, buf = bytes(buf), bytearray()
                if not block:
                    continue
                self._consume(block)

    def _consume(self, block: bytes) -> None:
        parsed = proto.parse_frame(proto.cobs_decode(block))
        if parsed is None:
            with self._lock:
                self.frames_bad += 1
            return
        ftype, payload = parsed
        if ftype != proto.TYPE_QUAT:
            return  # STATUS/other frames are not orientation data
        sample = proto.parse_quat(payload, recv_time=time.monotonic())
        if sample is None:
            with self._lock:
                self.frames_bad += 1
            return
        with self._lock:
            self._latest = sample
            self.frames_ok += 1
        self._first.set()

    # -- accessors (non-blocking) -----------------------------------------
    @property
    def alive(self) -> bool:
        """Whether the reader thread is currently running.

        ``False`` before :meth:`open` is called, and after the device
        disconnects (see :attr:`last_error` for why).
        """
        return self._thread is not None and self._thread.is_alive()

    @property
    def last_error(self) -> Exception | None:
        """The exception that stopped the reader thread, or ``None``."""
        return self._last_error

    def latest(self) -> Sample | None:
        """Return the newest :class:`Sample`, or ``None`` if none yet."""
        with self._lock:
            return self._latest

    def get_quat(self):
        """Return the newest ``(qw, qx, qy, qz)``, or ``None`` if none yet."""
        s = self.latest()
        return s.quat if s is not None else None

    def get_ypr(self, degrees: bool = True):
        """Return the newest ``(yaw, pitch, roll)``, or ``None`` if none yet."""
        s = self.latest()
        if s is None:
            return None
        return quat_to_ypr(*s.quat, degrees=degrees)

    def age(self) -> float | None:
        """Seconds since the newest sample was decoded, or ``None`` if none yet."""
        s = self.latest()
        if s is None:
            return None
        return time.monotonic() - s.recv_time

    def wait(self, timeout: float | None = None) -> bool:
        """Block until the first sample arrives. Returns ``True`` if one did."""
        return self._first.wait(timeout)
