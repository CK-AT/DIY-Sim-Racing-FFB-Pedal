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
from typing import Callable, Dict, List, Optional, Sequence

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
                self.msg_queue.put_nowait(msg)
            except Exception:
                pass

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

    return GatewayProtocol


# ---------------------------------------------------------------------------
# Scenario primitives
# ---------------------------------------------------------------------------


FfbBuilder = Callable[[float], ffb.FlightFfbAction]
FundamentalsBuilder = Callable[[float], ffb.DdsFundamentals]


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


# ---------------------------------------------------------------------------
# FFB amp packing (uint8 at 0.01 N/LSB)
# ---------------------------------------------------------------------------


def pack_vib_amp(amp_n: float) -> int:
    """Convert a Newton amplitude to the on-wire uint8 (0..255, 0.01 N/LSB)."""
    raw = int(round(amp_n * 100.0))
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
    """Constant `FlightFfbAction`. Vib amps in N (0..2.55 N range)."""

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
# Runner
# ---------------------------------------------------------------------------


async def _run_scenario_async(port: str, scenario: Scenario, baud: int) -> None:
    import serial_asyncio  # deferred so the module is importable without it
    GatewayProtocol = _make_protocol_class()
    loop = asyncio.get_event_loop()
    transport, protocol = await serial_asyncio.create_serial_connection(
        loop, GatewayProtocol, port, baudrate=baud
    )
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

        # 2. Settle so the axes apply the config before streaming starts.
        if scenario.settle_s > 0:
            print(f"--- settling for {scenario.settle_s}s ---")
            await asyncio.sleep(scenario.settle_s)

        # 3. Stream FFB actions + fundamentals at tick_hz.
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
            ffb.FUNCTION_ID_FLIGHT_STICK_PITCH: constant_ffb(vib1=(vib_amp_n, 0, 0, 0, 0)),
            ffb.FUNCTION_ID_FLIGHT_STICK_ROLL: constant_ffb(vib1=(vib_amp_n, 0, 0, 0, 0)),
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
    return p


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _build_parser().parse_args(argv)
    scenario = (
        dds_ramp(vib_amp_n=args.amp) if args.ramp else dds_smoke(fundamental_hz=args.hz, vib_amp_n=args.amp)
    )
    if args.duration is not None:
        scenario.duration_s = args.duration
    run(args.port, scenario, baud=args.baud)


if __name__ == "__main__":
    main()
