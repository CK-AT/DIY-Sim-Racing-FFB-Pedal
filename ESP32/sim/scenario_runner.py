"""
Scenario runner for testing FFB on hardware without SimHub / X-Plane.

A `Scenario` bundles three things:
  - A list of `FunctionConfig` messages to push to the gateway on startup.
  - A dict of per-function `FlightFfbAction` builders streamed at `tick_hz`.
  - An optional `DdsFundamentals` builder streamed at the same rate.

Connect a Linux laptop (or any host) to the gateway over USB-serial,
point the runner at the port, and exercise the entire FFB pipeline
without spinning up SimHub.

Quick start:

    pip install -r requirements.txt
    python scenario_runner.py /dev/ttyACM0          # default DDS smoke
    python scenario_runner.py /dev/ttyACM0 --ramp   # 0..15 Hz fundamental sweep

Authoring custom scenarios:

    import scenario_runner as sr
    scenario = sr.Scenario(
        configs=[sr.flight_stick_config(...)],
        ffb_streams={ffb.FUNCTION_ID_FLIGHT_STICK_PITCH: sr.constant_ffb(vib1=(0.5,0,0,0,0))},
        fundamentals_stream=sr.constant_fundamentals(dds1_hz=8.0),
        duration_s=30.0,
    )
    sr.run("/dev/ttyACM0", scenario)
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

import diy_ffb_protocol_pb2 as ffb

# Transport deps (`cobs`, `modbus_crc`, `serial_asyncio`) are imported lazily
# inside the runner so the module can be used to compose / inspect scenarios
# without those dependencies installed.


# ---------------------------------------------------------------------------
# Transport (COBS-framed protobuf over USB-serial)
# ---------------------------------------------------------------------------


def _make_protocol_class():
    """Construct the asyncio protocol once we know the transport deps load.

    Building the class lazily means importing this module doesn't fail when
    `cobs` / `modbus_crc` aren't installed (useful for unit-style scenario
    inspection on a machine without the serial stack).
    """
    from cobs import cobs as _cobs
    import modbus_crc as _modbus_crc

    class GatewayProtocol(asyncio.Protocol):
        def __init__(self):
            super().__init__()
            self.current_frame = bytearray()
            self.msg_queue: asyncio.Queue = asyncio.Queue()
            # Per-axis Futures for AxisConfig echoes claimed by request_axis_config().
            # When set, an incoming axis_config with a matching axis_id is routed to
            # the Future and *not* placed on msg_queue (so the log consumer never sees it).
            self.axis_config_waiters: Dict[int, "asyncio.Future[ffb.AxisConfig]"] = {}

        def connection_made(self, transport):
            self.transport = transport
            # 3-byte sync prefix matches what the C# plugin sends.
            transport.write(b"\x00\x00\x00")

        def data_received(self, data):
            last_idx = 0
            idx = data.find(b"\x00", last_idx)
            while idx >= 0:
                self.current_frame.extend(data[last_idx:idx])
                self._parse_frame(self.current_frame)
                last_idx = idx + 1
                self.current_frame = bytearray()
                idx = data.find(b"\x00", last_idx)
            self.current_frame.extend(data[last_idx:])

        def _parse_frame(self, frame):
            try:
                decoded = _cobs.decode(frame)
            except _cobs.DecodeError:
                return
            if not _modbus_crc.check_crc(decoded):
                return
            try:
                msg = ffb.Message().FromString(decoded[:-2])
            except Exception:
                return
            if msg.WhichOneof("payload") == "axis_config":
                fut = self.axis_config_waiters.pop(msg.axis_config.axis_id, None)
                if fut is not None and not fut.done():
                    fut.set_result(msg.axis_config)
                    return
            self.msg_queue.put_nowait(msg)

        def connection_lost(self, exc):
            if hasattr(self, "transport"):
                try:
                    self.transport.loop.stop()
                except Exception:
                    pass

        async def messages(self):
            while True:
                yield await self.msg_queue.get()

        def send_message(self, message: ffb.Message) -> None:
            data = message.SerializeToString()
            data = _modbus_crc.add_crc(data)
            data = _cobs.encode(data) + b"\x00"
            self.transport.write(data)

        async def request_axis_config(self, axis_id: int, timeout: float) -> ffb.AxisConfig:
            """Request the current AxisConfig from a specific axis. Returns the echoed
            config or raises asyncio.TimeoutError. The echo is captured via a Future so
            the message never reaches the regular msg_queue."""
            loop = asyncio.get_running_loop()
            existing = self.axis_config_waiters.get(axis_id)
            if existing is not None and not existing.done():
                existing.cancel()
            fut: "asyncio.Future[ffb.AxisConfig]" = loop.create_future()
            self.axis_config_waiters[axis_id] = fut
            req = ffb.Message()
            req.axis_action.axis_id = axis_id
            req.axis_action.return_axis_config = True
            self.send_message(req)
            try:
                return await asyncio.wait_for(fut, timeout)
            finally:
                # Drop the waiter if it's still ours (already popped on success).
                if self.axis_config_waiters.get(axis_id) is fut:
                    self.axis_config_waiters.pop(axis_id, None)

    return GatewayProtocol


# ---------------------------------------------------------------------------
# Scenario primitives
# ---------------------------------------------------------------------------


FfbBuilder = Callable[[float], ffb.FlightFfbAction]
FundamentalsBuilder = Callable[[float], ffb.DdsFundamentals]


@dataclass
class HidCaptureSpec:
    """When set on a Scenario, the runner captures the HID gamepad axis bound
    to `function_id` for the duration of the run, then dumps a CSV and (if
    `analyze` is set) calls it with the captured (t_s, position_mm) trace."""
    function_id: int
    device_hint: Optional[str] = None
    csv_path: Optional[str] = None
    # Optional callback invoked after capture ends. Receives the trace plus
    # the FunctionConfig the position was unscaled against. Used by the
    # resonance-sweep flow to run a stepped-sine lock-in analysis.
    analyze: Optional[Callable[[List[float], List[float], "ffb.FunctionConfig"], None]] = None


@dataclass
class Scenario:
    """A single test run."""
    configs: List[ffb.Message] = field(default_factory=list)
    ffb_streams: Dict[int, FfbBuilder] = field(default_factory=dict)
    fundamentals_stream: Optional[FundamentalsBuilder] = None
    duration_s: float = 30.0
    tick_hz: float = 50.0
    config_send_delay_s: float = 0.5
    settle_s: float = 1.0
    # Read-modify-write OscillationGuard patch applied per axis before streaming.
    # Keys are AxisConfig.OscillationGuard field names; values are floats/ints.
    # When None or empty, the patch step is skipped.
    oscillation_guard: Optional[Dict[str, Union[float, int]]] = None
    # Axes to patch. When None, the runner derives the set from
    # `configs[*].function_config.base.linked_axes`.
    oscillation_guard_axes: Optional[List[int]] = None
    # When True, the patched AxisConfig is persisted to EEPROM (`store=true`).
    oscillation_guard_store: bool = False
    # Per-axis timeout for the AxisConfig readback echo.
    oscillation_guard_readback_timeout_s: float = 2.0
    # Optional HID gamepad capture for resonance hunting.
    hid_capture: Optional[HidCaptureSpec] = None


# ---------------------------------------------------------------------------
# FFB amp packing (uint8 at 0.05 N/LSB)
# ---------------------------------------------------------------------------


def pack_vib_amp(amp_n: float) -> int:
    """Convert a Newton amplitude to the on-wire uint8 (0..255, 0.05 N/LSB)."""
    raw = int(round(amp_n * 20.0))
    if raw < 0:
        return 0
    if raw > 255:
        return 255
    return raw


def _fill_flight_action(
    a: ffb.FlightFfbAction,
    *,
    k_spring: float,
    k_damper: float,
    k_friction: float,
    trim_offset: float,
    buffet_amp: float,
    load_force: float,
    vib1: Sequence[float],
    vib2: Sequence[float],
) -> None:
    a.k_spring = float(k_spring)
    a.k_damper = float(k_damper)
    a.k_friction = float(k_friction)
    a.trim_offset = float(trim_offset)
    a.buffet_amp = float(buffet_amp)
    a.load_force = float(load_force)
    v1 = list(vib1) + [0.0] * (5 - len(vib1))
    v2 = list(vib2) + [0.0] * (2 - len(vib2))
    a.vib_amp_slot1 = pack_vib_amp(v1[0])
    a.vib_amp_slot2 = pack_vib_amp(v1[1])
    a.vib_amp_slot3 = pack_vib_amp(v1[2])
    a.vib_amp_slot4 = pack_vib_amp(v1[3])
    a.vib_amp_slot5 = pack_vib_amp(v1[4])
    a.vib2_amp_slot1 = pack_vib_amp(v2[0])
    a.vib2_amp_slot2 = pack_vib_amp(v2[1])


# ---------------------------------------------------------------------------
# Builder helpers — all return time->message callables
# ---------------------------------------------------------------------------


def constant_ffb(
    *,
    k_spring: float = 0.0,
    k_damper: float = 0.0,
    k_friction: float = 0.0,
    trim_offset: float = 0.0,
    buffet_amp: float = 0.0,
    load_force: float = 0.0,
    vib1: Sequence[float] = (0.0,) * 5,
    vib2: Sequence[float] = (0.0, 0.0),
) -> FfbBuilder:
    """Constant `FlightFfbAction`. Vib amps in N (0..12.75 N range)."""

    def _build(_t: float) -> ffb.FlightFfbAction:
        a = ffb.FlightFfbAction()
        _fill_flight_action(
            a,
            k_spring=k_spring, k_damper=k_damper, k_friction=k_friction,
            trim_offset=trim_offset, buffet_amp=buffet_amp, load_force=load_force,
            vib1=vib1, vib2=vib2,
        )
        return a

    return _build


def constant_fundamentals(
    dds1_hz: float = 0.0, dds2_hz: float = 0.0
) -> FundamentalsBuilder:
    """Constant `DdsFundamentals`."""

    def _build(_t: float) -> ffb.DdsFundamentals:
        m = ffb.DdsFundamentals()
        m.dds1_fundamental_hz = float(dds1_hz)
        m.dds2_fundamental_hz = float(dds2_hz)
        return m

    return _build


def ramp_fundamentals(
    *,
    dds1_from: float = 0.0,
    dds1_to: float = 12.0,
    period_s: float = 10.0,
    dds2_hz: float = 0.0,
) -> FundamentalsBuilder:
    """Triangle ramp on DDS 1: `from -> to -> from` over `period_s`."""

    def _build(t: float) -> ffb.DdsFundamentals:
        m = ffb.DdsFundamentals()
        if period_s <= 0.0:
            phase = 0.0
        else:
            phase = (t % period_s) / period_s  # 0..1
            if phase > 0.5:
                phase = 1.0 - phase  # 0..0.5..0
            phase *= 2.0  # 0..1..0
        m.dds1_fundamental_hz = dds1_from + (dds1_to - dds1_from) * phase
        m.dds2_fundamental_hz = float(dds2_hz)
        return m

    return _build


# ---------------------------------------------------------------------------
# Baselines — load FunctionConfig snapshots from sim/baselines.json
# ---------------------------------------------------------------------------


_BASELINES_PATH = Path(__file__).resolve().parent / "baselines.json"
_baselines_cache: Optional[Dict[int, ffb.FunctionConfig]] = None


def load_baselines(path: Optional[Path] = None) -> Dict[int, ffb.FunctionConfig]:
    """Load the per-function FunctionConfig snapshots from `baselines.json`.

    The file is a JSON object keyed by function id (string), each value is
    a protobuf-canonical-JSON FunctionConfig (camelCase fields, enum names
    as strings). Cached after first load.
    """
    global _baselines_cache
    if path is None and _baselines_cache is not None:
        return _baselines_cache
    p = Path(path) if path else _BASELINES_PATH
    with p.open(encoding="utf-8") as f:
        raw = json.load(f)
    from google.protobuf import json_format
    out: Dict[int, ffb.FunctionConfig] = {}
    for fid_str, cfg_dict in raw.items():
        cfg = ffb.FunctionConfig()
        json_format.ParseDict(cfg_dict, cfg, ignore_unknown_fields=True)
        out[int(fid_str)] = cfg
    if path is None:
        _baselines_cache = out
    return out


def baseline_message(
    function_id: int,
    *,
    phase_offset_rad: Optional[float] = None,
    vib_harmonic_ratios: Optional[Sequence[float]] = None,
    vib2_harmonic_ratios: Optional[Sequence[float]] = None,
    overrides: Optional[Callable[[ffb.FunctionConfig], None]] = None,
    path: Optional[Path] = None,
) -> ffb.Message:
    """Return a `Message` with the baseline `FunctionConfig` for `function_id`,
    optionally overlaying DDS fields or any other tweak via `overrides(cfg)`.
    """
    baselines = load_baselines(path)
    if function_id not in baselines:
        raise KeyError(
            f"baseline for function_id={function_id} not in {path or _BASELINES_PATH}"
        )
    cfg = ffb.FunctionConfig()
    cfg.CopyFrom(baselines[function_id])
    if phase_offset_rad is not None:
        cfg.flight_stick.phase_offset = float(phase_offset_rad)
    if vib_harmonic_ratios is not None:
        del cfg.flight_stick.vib_harmonic_ratios[:]
        for r in vib_harmonic_ratios:
            cfg.flight_stick.vib_harmonic_ratios.append(float(r))
    if vib2_harmonic_ratios is not None:
        del cfg.flight_stick.vib2_harmonic_ratios[:]
        for r in vib2_harmonic_ratios:
            cfg.flight_stick.vib2_harmonic_ratios.append(float(r))
    if overrides is not None:
        overrides(cfg)
    msg = ffb.Message()
    msg.function_config.CopyFrom(cfg)
    return msg


# ---------------------------------------------------------------------------
# Function config helpers
# ---------------------------------------------------------------------------


def flight_stick_config(
    function_id: int,
    axis_id: int,
    *,
    pos_min: int = -50,
    pos_max: int = 50,
    damping: float = 0.5,
    centering_spring_const: float = 1.5,
    phase_offset_rad: float = 0.0,
    vib_harmonic_ratios: Sequence[float] = (),
    vib2_harmonic_ratios: Sequence[float] = (),
    simulated_mass: float = 0.1,
    controller_axis: Optional[int] = None,
    output_mode: Optional[int] = None,
    store: bool = False,
) -> ffb.Message:
    """Build a `FunctionConfig` message for a single flight-stick axis.

    `phase_offset_rad` is the on-wire (radian) value — the SimHub
    plugin's override layer expresses it in degrees and converts here.
    """
    msg = ffb.Message()
    fc = msg.function_config
    fc.base.function_id = function_id
    fc.base.linked_axes.append(axis_id)
    for _ in range(3):
        fc.base.linked_axes.append(ffb.AXIS_UNDEFINED)
    fc.base.store = store
    fc.base.controller_output_axis = (
        controller_axis if controller_axis is not None else ffb.CONTROLLER_AXIS_X
    )
    fc.base.output_mode = (
        output_mode if output_mode is not None else ffb.OUTPUT_MODE_TRAVEL
    )
    fc.base.output_min = float(pos_min)
    fc.base.output_max = float(pos_max)
    fc.simulated_mass = float(simulated_mass)
    fc.flight_stick.pos_min = int(pos_min)
    fc.flight_stick.pos_max = int(pos_max)
    fc.flight_stick.damping = float(damping)
    fc.flight_stick.centering_spring_const = float(centering_spring_const)
    fc.flight_stick.phase_offset = float(phase_offset_rad)
    for r in vib_harmonic_ratios:
        fc.flight_stick.vib_harmonic_ratios.append(float(r))
    for r in vib2_harmonic_ratios:
        fc.flight_stick.vib2_harmonic_ratios.append(float(r))
    return msg


# ---------------------------------------------------------------------------
# OscillationGuard read-modify-write
# ---------------------------------------------------------------------------


_OSC_GUARD_FIELDS = {
    "k_max": float,
    "min_amplitude": float,
    "min_velocity": float,
    "min_frequency_hz": float,
    "max_frequency_hz": float,
    "hold_time_ms": int,
    "ramp_time_ms": int,
    "required_hits": int,
}


def _validate_osc_overrides(overrides: Dict[str, Any]) -> Dict[str, Union[float, int]]:
    """Coerce values to the right types and reject unknown OscillationGuard fields."""
    out: Dict[str, Union[float, int]] = {}
    for key, value in overrides.items():
        if key not in _OSC_GUARD_FIELDS:
            raise ValueError(
                f"unknown OscillationGuard field: {key!r}. "
                f"Allowed: {sorted(_OSC_GUARD_FIELDS)}"
            )
        out[key] = _OSC_GUARD_FIELDS[key](value)
    return out


def _axes_from_scenario_configs(configs: Sequence[ffb.Message]) -> List[int]:
    """Pull the unique linked axis ids out of any FunctionConfig messages, in order."""
    seen: set = set()
    out: List[int] = []
    for msg in configs:
        if msg.WhichOneof("payload") != "function_config":
            continue
        for ax in msg.function_config.base.linked_axes:
            ax_id = ax & ffb.AXIS_ID_MASK
            if ax_id == ffb.AXIS_UNDEFINED or ax_id in seen:
                continue
            seen.add(ax_id)
            out.append(ax_id)
    return out


async def _apply_oscillation_guard_patch(
    protocol,
    axis_ids: List[int],
    overrides: Dict[str, Union[float, int]],
    *,
    store: bool,
    readback_timeout_s: float,
    send_delay_s: float,
) -> None:
    """For each axis: read current AxisConfig, overlay OscillationGuard fields, send back."""
    print(f"--- patching oscillation guard on {len(axis_ids)} axis/axes: {sorted(overrides)} ---")
    for axis_id in axis_ids:
        axis_name = ffb.AxisID.Name(axis_id)
        try:
            cfg = await protocol.request_axis_config(axis_id, readback_timeout_s)
        except asyncio.TimeoutError:
            print(f"  [skip] {axis_name}: no axis_config readback within {readback_timeout_s:g}s")
            continue
        guard = cfg.oscillation_guard
        for key, value in overrides.items():
            setattr(guard, key, value)
        cfg.store = bool(store)
        msg = ffb.Message()
        msg.axis_config.CopyFrom(cfg)
        protocol.send_message(msg)
        store_tag = " [store]" if store else ""
        print(f"  [send] {axis_name}: oscillation_guard updated{store_tag}")
        await asyncio.sleep(send_delay_s)


# ---------------------------------------------------------------------------
# HID capture binding
# ---------------------------------------------------------------------------


def _find_function_config(
    configs: Sequence[ffb.Message], function_id: int
) -> Optional["ffb.FunctionConfig"]:
    for msg in configs:
        if (
            msg.WhichOneof("payload") == "function_config"
            and msg.function_config.base.function_id == function_id
        ):
            return msg.function_config
    return None


async def _start_hid_capture(spec: HidCaptureSpec, configs: Sequence[ffb.Message]):
    """Open the gamepad axis bound to `spec.function_id`. Returns a tuple of
    (HidCapture, FunctionConfig) so the caller can later unscale samples."""
    fc = _find_function_config(configs, spec.function_id)
    if fc is None:
        raise RuntimeError(
            f"hid_capture: no FunctionConfig for function_id={spec.function_id} "
            "in scenario.configs — capture needs the controller_output_axis mapping."
        )
    if fc.base.controller_output_axis == ffb.CONTROLLER_AXIS_UNDEFINED:
        raise RuntimeError(
            f"hid_capture: function {spec.function_id} has no controller_output_axis."
        )
    from hid_capture import HidCapture, controller_axis_to_evdev, find_input_device
    axis_code = controller_axis_to_evdev(fc.base.controller_output_axis)
    if axis_code is None:
        raise RuntimeError(
            f"hid_capture: unsupported ControllerAxis {fc.base.controller_output_axis}."
        )
    found = find_input_device(axis_code, name_hint=spec.device_hint)
    if found is None:
        raise RuntimeError(
            "hid_capture: no /dev/input device exposes the requested ABS axis. "
            "Make sure the gamepad is plugged in and the user has access."
        )
    device_path, device_name = found
    print(f"--- HID capture: {device_name} ({device_path}) axis_code={axis_code} ---")
    capture = HidCapture(device_path=device_path, axis_code=axis_code)
    await capture.start()
    return capture, fc


def _write_capture_csv(path: str, t_s: Sequence[float], pos_mm: Sequence[float]) -> None:
    with open(path, "w") as f:
        f.write("t_s,pos_mm\n")
        for t, x in zip(t_s, pos_mm):
            f.write(f"{t:.6f},{x:.6f}\n")
    print(f"--- HID trace written to {path} ({len(t_s)} samples) ---")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------


async def _run_scenario_async(port: str, scenario: Scenario, baud: int) -> None:
    import serial_asyncio  # deferred so the module is importable without it
    GatewayProtocol = _make_protocol_class()
    loop = asyncio.get_event_loop()
    transport, protocol = await serial_asyncio.create_serial_connection(
        loop, GatewayProtocol, port, baudrate=baud
    )
    # connection_made() runs via loop.call_soon, so it hasn't fired yet —
    # set transport eagerly so send_message works before the next loop tick.
    protocol.transport = transport
    print(f"Opened {port} @ {baud}")

    async def _log_consumer():
        async for msg in protocol.messages():
            kind = msg.WhichOneof("payload")
            if kind == "axis_log_message":
                lm = msg.axis_log_message
                print(f"[{ffb.AxisID.Name(lm.axis_id)}] {lm.msg.rstrip()}")
            elif kind == "gateway_log_message":
                lm = msg.gateway_log_message
                print(f"[{ffb.GatewayID.Name(lm.gateway_id)}] {lm.msg.rstrip()}")
            elif kind in ("axis_state", "gateway_state"):
                pass  # quiet; flip to True for verbose
            elif kind == "function_config":
                print("<- function_config echo")
            elif kind == "axis_config":
                print("<- axis_config echo")

    log_task = asyncio.create_task(_log_consumer())

    try:
        # 1. Send configs sequentially with a small delay each.
        if scenario.configs:
            print(f"--- pushing {len(scenario.configs)} config(s) ---")
            for cfg in scenario.configs:
                protocol.send_message(cfg)
                await asyncio.sleep(scenario.config_send_delay_s)

        # 2. Read-modify-write OscillationGuard patch on the targeted axes.
        if scenario.oscillation_guard:
            overrides = _validate_osc_overrides(scenario.oscillation_guard)
            target_axes = (
                scenario.oscillation_guard_axes
                if scenario.oscillation_guard_axes
                else _axes_from_scenario_configs(scenario.configs)
            )
            if target_axes:
                await _apply_oscillation_guard_patch(
                    protocol,
                    target_axes,
                    overrides,
                    store=scenario.oscillation_guard_store,
                    readback_timeout_s=scenario.oscillation_guard_readback_timeout_s,
                    send_delay_s=scenario.config_send_delay_s,
                )
            else:
                print("--- oscillation guard overrides set but no target axes resolved; skipping ---")

        # 3. Settle so the axes apply the config before streaming starts.
        if scenario.settle_s > 0:
            print(f"--- settling for {scenario.settle_s}s ---")
            await asyncio.sleep(scenario.settle_s)

        # 4. Optional HID gamepad capture starts just before streaming.
        capture = None
        capture_fc = None
        if scenario.hid_capture is not None:
            capture, capture_fc = await _start_hid_capture(scenario.hid_capture, scenario.configs)

        # 5. Stream FFB actions + fundamentals at tick_hz.
        period = 1.0 / scenario.tick_hz
        t0 = time.monotonic()
        next_due = t0
        print(f"--- streaming for {scenario.duration_s}s @ {scenario.tick_hz:g} Hz ---")
        while True:
            t = time.monotonic() - t0
            if t > scenario.duration_s:
                break
            for fid, builder in scenario.ffb_streams.items():
                action = builder(t)
                msg = ffb.Message()
                msg.ffb_action.function_id = fid
                msg.ffb_action.flight_ffb.CopyFrom(action)
                protocol.send_message(msg)
            if scenario.fundamentals_stream is not None:
                fund = scenario.fundamentals_stream(t)
                msg = ffb.Message()
                msg.dds_fundamentals.CopyFrom(fund)
                protocol.send_message(msg)
            next_due += period
            sleep_for = max(0.0, next_due - time.monotonic())
            await asyncio.sleep(sleep_for)
        print("--- done ---")

        # 6. Stop HID capture and run analysis / dump CSV.
        if capture is not None and capture_fc is not None:
            await capture.stop()
            t_s, pos_mm = capture.to_mm(capture_fc.base.output_min, capture_fc.base.output_max)
            print(f"--- captured {len(t_s)} HID samples ---")
            spec = scenario.hid_capture
            if spec is not None and spec.csv_path:
                _write_capture_csv(spec.csv_path, t_s, pos_mm)
            if spec is not None and spec.analyze is not None:
                spec.analyze(t_s, pos_mm, capture_fc)
    finally:
        log_task.cancel()


def run(port: str, scenario: Scenario, baud: int = 3_000_000) -> None:
    """Synchronous entry point. Opens the port, runs the scenario, returns."""
    try:
        asyncio.run(_run_scenario_async(port, scenario, baud))
    except KeyboardInterrupt:
        print("\n--- interrupted ---")


# ---------------------------------------------------------------------------
# Canonical DDS scenarios
# ---------------------------------------------------------------------------


def dds_smoke(
    *, fundamental_hz: float = 8.0, vib_amp_n: float = 1.0
) -> Scenario:
    """Constant 8 Hz / 1 N tone on pitch + roll with 90° phase offset.

    Uses the FunctionConfig baselines from `sim/baselines.json` (extracted
    from a SimHub plugin settings file) so the calibrated mass/damping/
    spring values are preserved — only DDS fields get overlaid.
    """
    pitch_cfg = baseline_message(
        ffb.FUNCTION_ID_FLIGHT_STICK_PITCH,
        phase_offset_rad=0.0,
        vib_harmonic_ratios=[1.0],
    )
    roll_cfg = baseline_message(
        ffb.FUNCTION_ID_FLIGHT_STICK_ROLL,
        phase_offset_rad=math.pi / 2.0,
        vib_harmonic_ratios=[1.0],
    )
    return Scenario(
        configs=[pitch_cfg, roll_cfg],
        ffb_streams={
            ffb.FUNCTION_ID_FLIGHT_STICK_PITCH: constant_ffb(k_spring=0.5, vib1=(vib_amp_n, 0, 0, 0, 0)),
            ffb.FUNCTION_ID_FLIGHT_STICK_ROLL: constant_ffb(k_spring=0.5, vib1=(vib_amp_n, 0, 0, 0, 0)),
        },
        fundamentals_stream=constant_fundamentals(dds1_hz=fundamental_hz),
        duration_s=30.0,
    )


def dds_ramp(
    *,
    hz_from: float = 0.0,
    hz_to: float = 15.0,
    period_s: float = 8.0,
    duration_s: float = 60.0,
    vib_amp_n: float = 1.0,
) -> Scenario:
    """Triangle ramp on the fundamental — useful to feel SyncVib track RPM."""
    base = dds_smoke(vib_amp_n=vib_amp_n)
    base.fundamentals_stream = ramp_fundamentals(
        dds1_from=hz_from, dds1_to=hz_to, period_s=period_s
    )
    base.duration_s = duration_s
    return base


# ---------------------------------------------------------------------------
# Resonance hunting (HID-driven)
# ---------------------------------------------------------------------------


_FUNCTION_ALIASES = {
    "pitch": "FUNCTION_ID_FLIGHT_STICK_PITCH",
    "roll": "FUNCTION_ID_FLIGHT_STICK_ROLL",
    "collective": "FUNCTION_ID_FLIGHT_STICK_COLLECTIVE",
    "pedals": "FUNCTION_ID_FLIGHT_PEDALS",
    "brake": "FUNCTION_ID_BRAKE_PEDAL",
    "accelerator": "FUNCTION_ID_ACCELERATOR_PEDAL",
    "clutch": "FUNCTION_ID_CLUTCH_PEDAL",
    "shifter": "FUNCTION_ID_SHIFTER",
}


def resolve_function_id(name_or_int: Union[int, str]) -> int:
    """Accept either a numeric FunctionID, an enum name, or a short alias."""
    if isinstance(name_or_int, int):
        return name_or_int
    text = str(name_or_int).strip()
    if text.isdigit():
        return int(text)
    key = text.lower()
    if key in _FUNCTION_ALIASES:
        return getattr(ffb, _FUNCTION_ALIASES[key])
    if hasattr(ffb, text):
        return getattr(ffb, text)
    raise ValueError(f"unknown function: {name_or_int!r}")


def _stepped_sine_freq_schedule(
    hz_from: float, hz_to: float, n_steps: int, dwell_s: float, log_spaced: bool = True
) -> List[Tuple[float, float, float]]:
    """Return [(t_start, t_end, freq_hz), ...] for a stepped-sine sweep.

    `log_spaced` puts equal weight per octave, which is conventional for
    mechanical-mode characterization.
    """
    import numpy as np
    n_steps = max(1, int(n_steps))
    if log_spaced and hz_from > 0.0 and hz_to > 0.0:
        freqs = np.geomspace(hz_from, hz_to, n_steps)
    else:
        freqs = np.linspace(hz_from, hz_to, n_steps)
    out: List[Tuple[float, float, float]] = []
    for i, f in enumerate(freqs):
        out.append((i * dwell_s, (i + 1) * dwell_s, float(f)))
    return out


def _stepped_sine_fundamentals_builder(
    schedule: Sequence[Tuple[float, float, float]]
) -> FundamentalsBuilder:
    """DDS 1 holds at each scheduled freq for its dwell window."""
    def _build(t: float) -> ffb.DdsFundamentals:
        m = ffb.DdsFundamentals()
        m.dds1_fundamental_hz = 0.0
        m.dds2_fundamental_hz = 0.0
        for t_start, t_end, freq in schedule:
            if t_start <= t < t_end:
                m.dds1_fundamental_hz = freq
                break
        return m
    return _build


def _lock_in(t_s: Sequence[float], pos_mm: Sequence[float], freq_hz: float) -> Tuple[float, float]:
    """Return (amplitude_mm, phase_rad) at `freq_hz` over the given trace."""
    import numpy as np
    if len(t_s) < 4 or freq_hz <= 0.0:
        return 0.0, 0.0
    t = np.asarray(t_s, dtype=float)
    x = np.asarray(pos_mm, dtype=float)
    x = x - x.mean()
    omega = 2.0 * math.pi * freq_hz
    real = float(np.trapezoid(x * np.cos(omega * t), t))
    imag = float(np.trapezoid(x * np.sin(omega * t), t))
    duration = float(t[-1] - t[0])
    if duration <= 0.0:
        return 0.0, 0.0
    a = 2.0 / duration * math.sqrt(real * real + imag * imag)
    phase = math.atan2(-imag, real)
    return a, phase


def _slice_trace(
    t_s: Sequence[float], pos_mm: Sequence[float], t_lo: float, t_hi: float
) -> Tuple[List[float], List[float]]:
    out_t: List[float] = []
    out_x: List[float] = []
    for t, x in zip(t_s, pos_mm):
        if t_lo <= t < t_hi:
            out_t.append(t)
            out_x.append(x)
    return out_t, out_x


def _make_stepped_sine_analyzer(
    schedule: Sequence[Tuple[float, float, float]],
    drive_amp_n: float,
    csv_path: Optional[str],
    settle_fraction: float = 0.3,
) -> Callable[[List[float], List[float], "ffb.FunctionConfig"], None]:
    """Build a stepped-sine lock-in analyzer that prints peak-by-peak output and
    optionally writes a per-step summary CSV (separate from the raw HID dump).

    `settle_fraction` discards the first portion of each dwell so transient
    build-up doesn't bias the lock-in. 0.3 = drop the first 30% of each step.
    """
    def _analyze(t_s, pos_mm, fc):
        if not t_s:
            print("--- resonance: empty HID trace, skipping analysis ---")
            return
        rows: List[Tuple[float, float, float, int]] = []
        print("--- resonance lock-in (stepped sine) ---")
        print(f"{'freq_hz':>10}  {'amp_mm':>10}  {'phase_deg':>10}  {'samples':>8}")
        for t_start, t_end, freq in schedule:
            cut = t_start + (t_end - t_start) * settle_fraction
            ts, xs = _slice_trace(t_s, pos_mm, cut, t_end)
            if len(ts) < 4:
                rows.append((freq, 0.0, 0.0, len(ts)))
                continue
            amp, phase = _lock_in(ts, xs, freq)
            rows.append((freq, amp, math.degrees(phase), len(ts)))
            print(f"{freq:10.3f}  {amp:10.4f}  {math.degrees(phase):10.2f}  {len(ts):8d}")
        if not rows:
            return
        peak = max(rows, key=lambda r: r[1])
        print(
            f"--- peak response: {peak[0]:.3f} Hz, "
            f"amp {peak[1]:.4f} mm @ drive {drive_amp_n:.3f} N "
            f"({peak[1] / drive_amp_n:.4f} mm/N) ---"
        )
        if csv_path:
            with open(csv_path, "w") as f:
                f.write("freq_hz,amp_mm,phase_deg,n_samples,drive_amp_n\n")
                for freq, amp, phase_deg, n in rows:
                    f.write(f"{freq:.6f},{amp:.6f},{phase_deg:.4f},{n},{drive_amp_n:.6f}\n")
            print(f"--- sweep summary written to {csv_path} ---")
    return _analyze


def _make_quiet_fft_analyzer(csv_path: Optional[str], n_peaks: int = 5):
    """Build an analyzer that FFTs the position trace and prints the top peaks.
    Useful for characterizing self-excited limit cycles where there's no drive."""
    def _analyze(t_s, pos_mm, _fc):
        import numpy as np
        if len(t_s) < 32:
            print("--- resonance: trace too short for FFT ---")
            return
        t = np.asarray(t_s, dtype=float)
        x = np.asarray(pos_mm, dtype=float)
        # Resample to a uniform grid since evdev timestamps jitter slightly.
        n = len(t)
        t_grid = np.linspace(t[0], t[-1], n)
        x_grid = np.interp(t_grid, t, x)
        x_grid = x_grid - x_grid.mean()
        fs = (n - 1) / (t_grid[-1] - t_grid[0]) if t_grid[-1] > t_grid[0] else 0.0
        if fs <= 0.0:
            print("--- resonance: degenerate timebase, skipping FFT ---")
            return
        spectrum = np.fft.rfft(x_grid * np.hanning(n))
        freqs = np.fft.rfftfreq(n, 1.0 / fs)
        # Window-aware amplitude scaling (Hann coherent gain = 0.5 → ×4 instead of ×2).
        mag = 4.0 * np.abs(spectrum) / n
        # Drop DC bin to avoid nominating it as a peak.
        if len(mag) > 1:
            mag[0] = 0.0
        # Local-max peak picking.
        peaks: List[Tuple[float, float]] = []
        for i in range(1, len(mag) - 1):
            if mag[i] > mag[i - 1] and mag[i] > mag[i + 1]:
                peaks.append((float(freqs[i]), float(mag[i])))
        peaks.sort(key=lambda p: -p[1])
        peaks = peaks[:n_peaks]
        print(f"--- resonance FFT (fs={fs:.1f} Hz, N={n}) ---")
        print(f"{'freq_hz':>10}  {'amp_mm':>10}")
        for freq, amp in peaks:
            print(f"{freq:10.3f}  {amp:10.4f}")
        if csv_path:
            with open(csv_path, "w") as f:
                f.write("freq_hz,amp_mm\n")
                for freq, amp in zip(freqs[1:], mag[1:]):
                    f.write(f"{freq:.6f},{amp:.6f}\n")
            print(f"--- spectrum written to {csv_path} ---")
    return _analyze


def resonance_sweep(
    *,
    function_id: int,
    hz_from: float = 1.0,
    hz_to: float = 30.0,
    n_steps: int = 25,
    dwell_s: float = 3.0,
    drive_amp_n: float = 0.5,
    log_spaced: bool = True,
    extra_function_ids: Sequence[int] = (),
    settle_fraction: float = 0.3,
    raw_csv_path: Optional[str] = None,
    sweep_csv_path: Optional[str] = None,
    device_hint: Optional[str] = None,
) -> Scenario:
    """Stepped-sine sweep on `function_id`, capturing HID position via evdev.

    The probed function plus any `extra_function_ids` get baseline configs
    (so spring/damper/etc. are calibrated). Drive is *only* SyncVib at
    `drive_amp_n` on the probed function — every other force is zeroed. Pair
    with `--osc-k-max 0` on the CLI to silence the OscillationGuard while
    measuring.
    """
    schedule = _stepped_sine_freq_schedule(hz_from, hz_to, n_steps, dwell_s, log_spaced)
    duration_s = schedule[-1][1] if schedule else 0.0

    target_cfg = baseline_message(
        function_id, phase_offset_rad=0.0, vib_harmonic_ratios=[1.0]
    )
    configs = [target_cfg]
    for fid in extra_function_ids:
        if fid != function_id:
            configs.append(baseline_message(fid, vib_harmonic_ratios=[1.0]))

    streams = {
        function_id: constant_ffb(vib1=(drive_amp_n, 0, 0, 0, 0)),
    }
    for fid in extra_function_ids:
        if fid != function_id:
            streams[fid] = constant_ffb()

    return Scenario(
        configs=configs,
        ffb_streams=streams,
        fundamentals_stream=_stepped_sine_fundamentals_builder(schedule),
        duration_s=duration_s,
        hid_capture=HidCaptureSpec(
            function_id=function_id,
            device_hint=device_hint,
            csv_path=raw_csv_path,
            analyze=_make_stepped_sine_analyzer(
                schedule, drive_amp_n, sweep_csv_path, settle_fraction=settle_fraction
            ),
        ),
    )


def resonance_quiet(
    *,
    function_id: int,
    duration_s: float = 10.0,
    extra_function_ids: Sequence[int] = (),
    raw_csv_path: Optional[str] = None,
    fft_csv_path: Optional[str] = None,
    device_hint: Optional[str] = None,
) -> Scenario:
    """Quiet capture: zero drive, just record HID position. Used to characterize
    self-excited limit cycles by FFT'ing the resulting waveform."""
    target_cfg = baseline_message(function_id, vib_harmonic_ratios=[1.0])
    configs = [target_cfg]
    for fid in extra_function_ids:
        if fid != function_id:
            configs.append(baseline_message(fid, vib_harmonic_ratios=[1.0]))
    streams = {function_id: constant_ffb()}
    for fid in extra_function_ids:
        if fid != function_id:
            streams[fid] = constant_ffb()
    return Scenario(
        configs=configs,
        ffb_streams=streams,
        fundamentals_stream=constant_fundamentals(dds1_hz=0.0),
        duration_s=duration_s,
        hid_capture=HidCaptureSpec(
            function_id=function_id,
            device_hint=device_hint,
            csv_path=raw_csv_path,
            analyze=_make_quiet_fft_analyzer(fft_csv_path),
        ),
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("port", help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    p.add_argument("--baud", type=int, default=3_000_000)
    p.add_argument("--ramp", action="store_true",
                   help="Run the dds_ramp scenario (default: dds_smoke)")
    p.add_argument("--hz", type=float, default=8.0, help="Fundamental Hz (smoke mode)")
    p.add_argument("--amp", type=float, default=1.0, help="Vib amp in N (smoke + ramp)")
    p.add_argument("--duration", type=float, default=None, help="Override duration_s")
    p.add_argument("--ramp-from", type=float, default=0.0,
                   help="Ramp mode: low end of the DDS 1 sweep in Hz (default 0)")
    p.add_argument("--ramp-to", type=float, default=15.0,
                   help="Ramp mode: high end of the DDS 1 sweep in Hz (default 15)")
    p.add_argument("--ramp-period", type=float, default=8.0,
                   help="Ramp mode: full triangle period in seconds (from→to→from, default 8)")

    res = p.add_argument_group(
        "resonance hunting (HID capture)",
        "Probe a function's mechanical response by reading the gamepad axis "
        "the firmware exposes for it. Pick exactly one of --resonance-sweep "
        "(stepped sine + lock-in) or --resonance-quiet (zero drive + FFT, for "
        "self-excited limit cycles). Works on Linux only (uses evdev).",
    )
    res.add_argument("--resonance-sweep", action="store_true",
                     help="Run the stepped-sine resonance sweep")
    res.add_argument("--resonance-quiet", action="store_true",
                     help="Run a zero-drive HID capture and FFT it")
    res.add_argument("--resonance-function", default="roll",
                     help="Function to probe — alias (pitch/roll/collective/pedals/"
                          "brake/accelerator/clutch/shifter), FunctionID enum name, "
                          "or numeric id (default: roll)")
    res.add_argument("--resonance-from", type=float, default=1.0,
                     help="Sweep low end in Hz (default 1.0)")
    res.add_argument("--resonance-to", type=float, default=30.0,
                     help="Sweep high end in Hz (default 30.0)")
    res.add_argument("--resonance-steps", type=int, default=25,
                     help="Number of stepped-sine frequencies (default 25)")
    res.add_argument("--resonance-dwell", type=float, default=3.0,
                     help="Dwell time per step in seconds (default 3.0)")
    res.add_argument("--resonance-amp", type=float, default=0.5,
                     help="Drive amplitude in N (default 0.5)")
    res.add_argument("--resonance-linear", action="store_true",
                     help="Linear-spaced sweep instead of log-spaced (default: log)")
    res.add_argument("--resonance-quiet-duration", type=float, default=10.0,
                     help="Quiet-capture duration in seconds (default 10.0)")
    res.add_argument("--resonance-input", default=None,
                     help="Substring of the gamepad device name when multiple match")
    res.add_argument("--resonance-raw-csv", default=None,
                     help="Write the raw HID trace to this CSV path")
    res.add_argument("--resonance-summary-csv", default=None,
                     help="Write the per-step lock-in summary (sweep) or full spectrum (quiet)")

    osc = p.add_argument_group(
        "oscillation guard overrides",
        "Read-modify-write the AxisConfig.oscillation_guard fields on the "
        "targeted axes before streaming starts. Any flag set is patched in; "
        "untouched fields keep their on-device value.",
    )
    osc.add_argument("--osc-k-max", type=float, default=None,
                     help="Damping gain when oscillation detected")
    osc.add_argument("--osc-min-amplitude", type=float, default=None,
                     help="Min amplitude in mm")
    osc.add_argument("--osc-min-velocity", type=float, default=None,
                     help="Min velocity in mm/s")
    osc.add_argument("--osc-min-frequency", type=float, default=None,
                     help="Min oscillation frequency in Hz")
    osc.add_argument("--osc-max-frequency", type=float, default=None,
                     help="Max oscillation frequency in Hz")
    osc.add_argument("--osc-hold-ms", type=int, default=None,
                     help="Hold time after detection in ms")
    osc.add_argument("--osc-ramp-ms", type=int, default=None,
                     help="Ramp time for damping gain in ms")
    osc.add_argument("--osc-required-hits", type=int, default=None,
                     help="Hits required before damping engages")
    osc.add_argument("--osc-axis", type=int, action="append", metavar="N",
                     help="Axis number to patch (1..8). Repeatable. "
                          "Defaults to the axes referenced by the scenario's FunctionConfigs.")
    osc.add_argument("--osc-store", action="store_true",
                     help="Persist patched AxisConfig to EEPROM")
    return p


_OSC_CLI_TO_FIELD = {
    "osc_k_max": "k_max",
    "osc_min_amplitude": "min_amplitude",
    "osc_min_velocity": "min_velocity",
    "osc_min_frequency": "min_frequency_hz",
    "osc_max_frequency": "max_frequency_hz",
    "osc_hold_ms": "hold_time_ms",
    "osc_ramp_ms": "ramp_time_ms",
    "osc_required_hits": "required_hits",
}


def _osc_overrides_from_args(args) -> Dict[str, Union[float, int]]:
    return {
        field_name: getattr(args, attr)
        for attr, field_name in _OSC_CLI_TO_FIELD.items()
        if getattr(args, attr) is not None
    }


def _osc_axes_from_args(args) -> Optional[List[int]]:
    if not args.osc_axis:
        return None
    out: List[int] = []
    for n in args.osc_axis:
        if n < 1 or n > 8:
            raise SystemExit(f"--osc-axis must be in 1..8, got {n}")
        out.append(getattr(ffb, f"AXIS_ID_{n}"))
    return out


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _build_parser().parse_args(argv)
    if args.resonance_sweep and args.resonance_quiet:
        raise SystemExit("--resonance-sweep and --resonance-quiet are mutually exclusive")
    if args.resonance_sweep:
        function_id = resolve_function_id(args.resonance_function)
        scenario = resonance_sweep(
            function_id=function_id,
            hz_from=args.resonance_from,
            hz_to=args.resonance_to,
            n_steps=args.resonance_steps,
            dwell_s=args.resonance_dwell,
            drive_amp_n=args.resonance_amp,
            log_spaced=not args.resonance_linear,
            raw_csv_path=args.resonance_raw_csv,
            sweep_csv_path=args.resonance_summary_csv,
            device_hint=args.resonance_input,
        )
    elif args.resonance_quiet:
        function_id = resolve_function_id(args.resonance_function)
        scenario = resonance_quiet(
            function_id=function_id,
            duration_s=args.resonance_quiet_duration,
            raw_csv_path=args.resonance_raw_csv,
            fft_csv_path=args.resonance_summary_csv,
            device_hint=args.resonance_input,
        )
    elif args.ramp:
        scenario = dds_ramp(
            hz_from=args.ramp_from,
            hz_to=args.ramp_to,
            period_s=args.ramp_period,
            vib_amp_n=args.amp,
        )
    else:
        scenario = dds_smoke(fundamental_hz=args.hz, vib_amp_n=args.amp)
    if args.duration is not None:
        scenario.duration_s = args.duration
    osc_overrides = _osc_overrides_from_args(args)
    if osc_overrides:
        scenario.oscillation_guard = osc_overrides
        scenario.oscillation_guard_axes = _osc_axes_from_args(args)
        scenario.oscillation_guard_store = args.osc_store
    elif args.osc_store or args.osc_axis:
        raise SystemExit("--osc-store / --osc-axis require at least one --osc-* value override")
    run(args.port, scenario, baud=args.baud)


if __name__ == "__main__":
    main()
