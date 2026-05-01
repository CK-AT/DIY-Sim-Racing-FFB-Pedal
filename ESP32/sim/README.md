# Simulation Assets

Sample configs:

- `sample_general_kinematic.json`: general kinematics example.
- `axis1_diy_pedal_config.json`: legacy DIY pedal axis config used to validate migration into general kinematics.

## scenario_runner.py — drive the gateway from a Linux laptop

Lets you exercise FFB / DDS without SimHub or X-Plane. Connects over
USB-serial to the gateway (same protocol the SimHub plugin uses) and
streams `FlightFfbAction` + `DdsFundamentals` at chosen rates.

Setup (once):

    pip install -r requirements.txt
    bash compile_protobuf.sh

Run the canonical DDS smoke test (8 Hz / 1 N tone, pitch + roll with
90° phase offset, 30 s):

    python scenario_runner.py /dev/ttyACM0

Or sweep the fundamental 0..15 Hz:

    python scenario_runner.py /dev/ttyACM0 --ramp

### Baselines

`baselines.json` holds the per-function `FunctionConfig` snapshots used as
starting points for scenarios — pulled from a SimHub plugin settings
export so the calibrated mass / damping / spring / motion-range values
are real, not hand-coded defaults. Scenarios overlay DDS fields (phase
offset, harmonic ratios) on top.

Refresh after re-tuning in SimHub:

    python -c "
    import json
    src = r'Z:\Projects\FFB\DiyFfbPlugin.GeneralSettings.json'
    with open(src, encoding='utf-8') as f: data = json.load(f)
    LEGACY = ('flightStickPitch', 'flightStickRoll', 'flightStickCollective')
    def migrate(s):
        for k in LEGACY: s = s.replace(f'\"{k}\":', '\"flightStick\":')
        return s
    out = {fid: json.loads(migrate(s)) for fid, s in data['FunctionBaselines'].items()}
    with open('baselines.json', 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2, sort_keys=True)
    "

(The `flightStickPitch`/`Roll`/`Collective` → `flightStick` rename
handles settings files saved before the FlightStick proto consolidation.)

### Custom scenarios from Python

    import scenario_runner as sr
    import diy_ffb_protocol_pb2 as ffb

    pitch_cfg = sr.baseline_message(
        ffb.FUNCTION_ID_FLIGHT_STICK_PITCH,
        phase_offset_rad=0.0,
        vib_harmonic_ratios=[1.0, 2.0, 3.0],
    )
    scenario = sr.Scenario(
        configs=[pitch_cfg],
        ffb_streams={
            ffb.FUNCTION_ID_FLIGHT_STICK_PITCH: sr.constant_ffb(vib1=(0.5, 0.2, 0.1, 0, 0)),
        },
        fundamentals_stream=sr.constant_fundamentals(dds1_hz=8.0),
        duration_s=60.0,
    )
    sr.run("/dev/ttyACM0", scenario)

If you need to start from scratch instead of from a baseline, use
`sr.flight_stick_config(...)`. Stream builders are plain `t -> Message`
callables, so any time-varying pattern works (ramps, sines, steps,
pre-recorded traces).
