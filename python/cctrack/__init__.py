# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martin Pollow
"""Host-side consumer library for the CCTrack USB head-tracker.

Quick start::

    from cctrack import Tracker
    with Tracker() as t:                 # auto-detects the device, waits for the first sample
        yaw, pitch, roll = t.get_ypr()   # newest sample, non-blocking
        qw, qx, qy, qz   = t.get_quat()

Or, for throwaway scripts and the REPL, the module-level one-liners open a
shared default tracker on first use::

    import cctrack
    cctrack.get_ypr()
"""

from __future__ import annotations

import threading

from ._protocol import Sample
from .orientation import quat_to_ypr
from .tracker import Tracker, TrackerError, find_port, USB_PID, USB_VID

__all__ = [
    "Tracker",
    "TrackerError",
    "Sample",
    "find_port",
    "quat_to_ypr",
    "get_quat",
    "get_ypr",
    "default_tracker",
    "close_default",
    "USB_VID",
    "USB_PID",
]

__version__ = "0.1.0"

# Lazily-created shared tracker backing the module-level convenience functions.
_default: Tracker | None = None
_default_lock = threading.Lock()


def default_tracker() -> Tracker:
    """Return the shared default :class:`Tracker`, opening it on first call.

    Blocks until the first sample is available so the convenience accessors
    return real data rather than ``None`` on the very first call.
    """
    global _default
    with _default_lock:
        if _default is None:
            t = Tracker().open()
            t.wait(timeout=2.0)
            _default = t
        return _default


def close_default() -> None:
    """Close the shared default tracker, if one was opened."""
    global _default
    with _default_lock:
        if _default is not None:
            _default.close()
            _default = None


def get_quat():
    """Newest ``(qw, qx, qy, qz)`` from the shared default tracker."""
    return default_tracker().get_quat()


def get_ypr(degrees: bool = True):
    """Newest ``(yaw, pitch, roll)`` from the shared default tracker."""
    return default_tracker().get_ypr(degrees=degrees)
