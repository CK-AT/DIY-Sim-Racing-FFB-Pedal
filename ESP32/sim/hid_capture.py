"""HID gamepad capture for resonance hunting.

The firmware updates the joystick HID report every 10 ms (~100 Hz, see
`CommManager::send_joystick_values`). That's an order of magnitude faster than
the 10 Hz `AxisState` stream over the protocol channel and is more than enough
to characterize mechanical modes up to ~30 Hz.

This module is Linux-only (uses `evdev`). It exposes:

  - `controller_axis_to_evdev`: map a `ControllerAxis` enum value to the
    matching `ecodes.ABS_*` code.
  - `find_input_device`: pick a `/dev/input/eventX` whose absinfo lists the
    requested ABS axis. An optional `name_hint` substring narrows the choice
    when multiple gamepads are attached.
  - `HidCapture`: async-friendly capture loop. Buffers `(t_seconds, raw)`
    tuples and converts them to mm via the function's `output_min`/`output_max`.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


def controller_axis_to_evdev(controller_axis: int) -> Optional[int]:
    """Translate a `ControllerAxis` enum value to the matching evdev ABS code."""
    import diy_ffb_protocol_pb2 as ffb
    from evdev import ecodes
    return {
        ffb.CONTROLLER_AXIS_X: ecodes.ABS_X,
        ffb.CONTROLLER_AXIS_Y: ecodes.ABS_Y,
        ffb.CONTROLLER_AXIS_Z: ecodes.ABS_Z,
        ffb.CONTROLLER_AXIS_R_X: ecodes.ABS_RX,
        ffb.CONTROLLER_AXIS_R_Y: ecodes.ABS_RY,
        ffb.CONTROLLER_AXIS_R_Z: ecodes.ABS_RZ,
        ffb.CONTROLLER_AXIS_RUD: ecodes.ABS_RUDDER,
        ffb.CONTROLLER_AXIS_THR: ecodes.ABS_THROTTLE,
        ffb.CONTROLLER_AXIS_ACC: ecodes.ABS_GAS,
        ffb.CONTROLLER_AXIS_BRK: ecodes.ABS_BRAKE,
        ffb.CONTROLLER_AXIS_STEER: ecodes.ABS_WHEEL,
    }.get(controller_axis)


def find_input_device(axis_code: int, name_hint: Optional[str] = None) -> Optional[Tuple[str, str]]:
    """Find an /dev/input/eventX device that exposes `axis_code` (e.g. ABS_RX).

    Returns `(path, device_name)` or `None`. When multiple devices match, prefer
    the first whose `name` contains `name_hint`; otherwise the first match wins.
    """
    from evdev import InputDevice, ecodes, list_devices
    candidates: List[Tuple[str, str]] = []
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except (OSError, PermissionError):
            continue
        try:
            caps = dev.capabilities()
            abs_codes = [code for code, _info in caps.get(ecodes.EV_ABS, [])]
            if axis_code in abs_codes:
                candidates.append((path, dev.name))
        finally:
            dev.close()
    if not candidates:
        return None
    if name_hint:
        for path, name in candidates:
            if name_hint.lower() in name.lower():
                return path, name
    return candidates[0]


@dataclass
class HidCapture:
    """Capture absolute-axis events for one ABS axis on one device.

    Background-task pattern: call `await start()` to open the device and start
    reading; events flow into `samples` as `(t_relative_s, raw_value)`. Call
    `await stop()` to cancel and close.
    """
    device_path: str
    axis_code: int
    samples: List[Tuple[float, int]] = field(default_factory=list)
    abs_min: int = 0
    abs_max: int = 0
    _task: Optional[asyncio.Task] = None
    _device: Optional[object] = None
    _t0: float = 0.0

    async def start(self) -> None:
        from evdev import InputDevice
        self._device = InputDevice(self.device_path)
        info = self._device.absinfo(self.axis_code)
        self.abs_min = info.min
        self.abs_max = info.max
        self._t0 = 0.0
        self._task = asyncio.create_task(self._reader())

    async def _reader(self) -> None:
        from evdev import ecodes
        rebased = False
        try:
            async for ev in self._device.async_read_loop():
                if ev.type != ecodes.EV_ABS or ev.code != self.axis_code:
                    continue
                # ev.timestamp() is the kernel-recorded receive time. Rebase
                # exactly once on the first event so the trace starts at t=0.
                t = ev.timestamp()
                if not rebased:
                    self._t0 = t
                    rebased = True
                self.samples.append((t - self._t0, ev.value))
        except asyncio.CancelledError:
            raise
        except Exception:
            pass

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None

    def to_mm(self, output_min_mm: float, output_max_mm: float) -> Tuple[List[float], List[float]]:
        """Convert buffered raw samples to (t_seconds, position_mm) lists.

        Maps `[abs_min, abs_max]` linearly onto `[output_min_mm, output_max_mm]`,
        which is how `CommManager::send_joystick_values` projects axis position
        into the HID report range (via the function's `output_min`/`output_max`).
        """
        span = self.abs_max - self.abs_min
        if span == 0:
            span = 1
        scale = (output_max_mm - output_min_mm) / float(span)
        ts = [s[0] for s in self.samples]
        xs = [output_min_mm + (raw - self.abs_min) * scale for _, raw in self.samples]
        return ts, xs
