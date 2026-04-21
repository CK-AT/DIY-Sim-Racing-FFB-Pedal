# Helicopter Vibration

Helicopter-specific vibration envelope drivers, slot profiles, graph
templates, and per-aircraft tuning. Uses the DDS subsystem from plan 07
and telemetry signals from plan 04.

**Prerequisites:** Plan 03 (ConfigOut & FunctionScope), Plan 04 (X-Plane signals), Plan 07 (DDS subsystem)


---

## 1. Vibration Sources and Envelope Drivers

All envelope drivers come from live X-Plane datarefs — no synthetic
IAS-based approximations needed. Harmonic ratios are per-aircraft config,
set once on profile load.

### Amplitude envelopes

```
VibFundamental = rpm / 60

-- 1/rev: from per-axis blade alpha (already in cyclic axis frame)
--   No axis swap — X-Plane's elev/ailn decomposition matches our axes
Vib1Rev_pitch = gain_1rev * abs(blade_alph_pitch)   # longitudinal (dominant)
Vib1Rev_roll  = gain_1rev * abs(blade_alph_roll)     # lateral (weak)

-- 2/rev: two components summed
--   ETL component: vrs transition zone (0.50 -> 0.25) IS the ETL
--   High-speed component: blade slap builds with speed
etl_factor = max(0, (vrs - 0.25) / 0.25)    # 1.0 at hover, 0.0 at 60+ kt
Vib2Rev = etl_gain * etl_factor + slap_gain * slap_rat

-- 3/rev: retreating blade stall onset
--   blade_alpha approaching stall values
Vib3Rev = rbs_gain * max(0, blade_alpha - rbs_threshold)

-- N/rev: blade-passing, relatively constant when rotor is turning
VibNRev = base_nrev * rpm_norm

-- 2N/rev: 2nd blade-passing, low-level
Vib2NRev = base_2nrev * rpm_norm
```


---

## 2. Slot Assignment Profiles

### `heli_cyclic` — helicopter pitch/roll axes

DDS 1 fundamental: main rotor RPM / 60 (gateway-synced).
DDS 2 fundamental: engine RPM / 60 (free-running, no sync needed).

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Rotor 1/rev (disc tilt) | `blade_alph_pitch/roll` — **axis-split** |
| 1 | 2 | 2.0 | Rotor 2/rev (ETL + high-speed) | `vrs` + `slap_rat` |
| 1 | 3 | 3.0 | Rotor 3/rev (retreating blade stall) | `blade_alpha` threshold |
| 1 | 4 | N | Blade-passing N/rev | RPM-proportional constant |
| 1 | 5 | 2N | Blade-passing 2nd harmonic | RPM-proportional constant |
| 2 | 1 | 1.0 | Engine 1/rev | base + `torque_norm` |
| 2 | 2 | 2.0 | Engine 2/rev | base constant |

`rotation_sign = +1` (US/CCW) or `-1` (EU/CW).

Graph template: `heli_vibration_cyclic.json`.

### `heli_pedal` — helicopter yaw axis

DDS 1 fundamental: main rotor RPM / 60 (gateway-synced).
DDS 2: disabled (fundamental = 0).

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Main rotor 1/rev (weak on pedals) | RPM-proportional |
| 1 | 2 | 2.0 | Main rotor 2/rev | `vrs` + `slap_rat` |
| 1 | 3 | gear | Tail rotor blade-passing | RPM-proportional |
| 1 | 4 | 2×gear | Tail rotor 2nd harmonic | RPM-proportional |
| 1 | 5 | — | Unused | amplitude = 0 |

Gear ratio is aircraft-specific (e.g., 4.62 for MD 500E). Tail rotor
vibration rides on the main rotor DDS — no second oscillator needed.

`rotation_sign = 0` (no axis split on pedals — single axis).

Graph template: `heli_vibration_pedal.json` (parameterized by gear ratio).

### `heli_cyclic_with_engine` — helicopter with engine vibration on DDS 2

Same as `heli_cyclic` above. The DDS 2 engine slots are optional — set
amplitudes to zero if engine vibration is not desired (e.g., turbine
helicopters where the engine is too smooth to feel).


---

## 3. Graph Template: `heli_vibration_cyclic.json`

Reusable sub-graph that takes rotor state and produces all vibration outputs.

### Inputs (wired from parent)

* `blade_count` ← `Aircraft.BladeCount`
* `rotation_sign` ← `Aircraft.RotationSign`
* `blade_alph_pitch` ← `XPlane.Rotor.BladeAlphPitch`
* `blade_alph_roll` ← `XPlane.Rotor.BladeAlphRoll`
* `slap_rat` ← `XPlane.Rotor.Slap`
* `vrs` ← `XPlane.Rotor.VRS`
* `blade_alpha` ← `XPlane.Rotor.BladeAlpha`
* `rpm` ← `XPlane.MainRotor.Speed`
* `torque` ← engine torque signal

### Scoped ConfigOut → FunctionScope's FlightStickConfig

```text
rotation_sign     → ConfigField "flight_stick.rotation_sign"
Const(1.0)        → ConfigField "flight_stick.vib_harmonic_ratios.0"
Const(2.0)        → ConfigField "flight_stick.vib_harmonic_ratios.1"
Const(3.0)        → ConfigField "flight_stick.vib_harmonic_ratios.2"
blade_count       → ConfigField "flight_stick.vib_harmonic_ratios.3"
blade_count × 2   → ConfigField "flight_stick.vib_harmonic_ratios.4"
```

### Params (exposed in vehicle tab UI)

```
slot 1 (ratio 1.0)   gain_1rev
slot 2 (ratio 2.0)   etl_gain | slap_gain
slot 3 (ratio 3.0)   rbs_gain | rbs_threshold
slot 4 (ratio N)     base
slot 5 (ratio 2N)    base
```

### Scoped Output → FunctionScope's FlightFfbAction

```
SpringGain, DamperGain, Friction, LoadForce, TrimOffset,
VibSlot1..5, Vib2Slot1..2
```

### Pedal variant

For pedals with tail rotor vibration, the graph sets different ratios:

```
slot 1 (ratio 1.0)   main rotor 1/rev (weak on pedals)
slot 2 (ratio 2.0)   main rotor 2/rev
slot 3 (ratio 4.62)  tail rotor blade-passing (2-blade TR at 4.62:1 gear)
slot 4 (ratio 9.24)  tail rotor 2nd harmonic
```

### Parent template wiring

```
heli_default.json (parent template)

  Aircraft.BladeCount   [Param = 5]     ← shared, one slider
  Aircraft.RotationSign [Param = 1]     ← shared
  Aircraft.TRGearRatio  [Param = 4.62]  ← shared

  cyclic_pitch_include
    Kind: Include, FunctionScope: "FlightStickPitch"
    Inputs wired: torque, rpm, aero_trq, blade_count, rotation_sign
    (no output links — scoped nodes handle it)

  cyclic_roll_include
    Kind: Include, FunctionScope: "FlightStickRoll"
    Inputs wired: torque, rpm, aero_trq, blade_count, rotation_sign

  pedals_include
    Kind: Include, FunctionScope: "FlightPedals"
    Inputs wired: torque, rpm, aero_trq, tr_gear_ratio
```

Same `Aircraft.BladeCount` Param feeds both cyclic includes. Changing
blade count → graph re-evaluates → scoped ConfigOut values change for
pitch and roll → plugin detects → config upload for both functions.

| | Before (generic) | After (scoped) |
| --- | --- | --- |
| Links per cyclic include | ~14 (7 in + 7 out) | ~7 (7 in, 0 out) |
| ConfigOut links per include | ~10 (new) | 0 |
| Parent Output nodes | 4 (one per function) | 0 |
| Parent ConfigOut nodes | 4 | 0 |


---

## 4. Per-Aircraft Tuning

### MD500E (reference implementation)

Static config: `rotation_sign = +1` (CCW, US)

Cyclic ratios: `[1.0, 2.0, 3.0, 5.0, 10.0]`
Pedal ratios: `[1.0, 2.0, 4.62, 9.24]` (tail rotor gear ratio 4.62:1, 2-blade TR)

| Slot | Ratio | Freq at 100% RPM | Dataref driver | Physical cue |
| --- | --- | --- | --- | --- |
| 1 | 1.0 | 8.2 Hz | `cyclic_elev_blad_alph` | Disc-loading "alive" feel |
| 2 | 2.0 | 16.4 Hz | `vortex_ring_state` + `blade_slap_rat` | ETL + high-speed |
| 3 | 3.0 | 24.6 Hz | `rotor_blade_alpha_deg` | Retreating blade stall |
| 4 | 5.0 (N) | 41.0 Hz | RPM-proportional | Blade-passing "whirr" |
| 5 | 10.0 (2N) | 82.0 Hz | RPM-proportional | 2nd blade-passing |

Pedal-specific slots:

| Slot | Ratio | Freq at 100% RPM | Physical cue |
| --- | --- | --- | --- |
| 3 | 4.62 | 37.9 Hz | Tail rotor blade-passing (2-blade × 4.62:1) |
| 4 | 9.24 | 75.8 Hz | Tail rotor 2nd harmonic |

Observed dataref ranges (from dataref logger flight):

| IAS | blade_alph_pitch | vrs_0 | slap_rat | blade_alpha |
| --- | --- | --- | --- | --- |
| hover | -0.2 deg | 0.500 | 0.000 | 2.9 deg |
| 20 kt | -1.2 deg | 0.470 | 0.005 | 2.7 deg |
| 40 kt | -1.1 deg | 0.359 | 0.016 | 2.6 deg |
| 80 kt | -1.7 deg | 0.250 | 0.024 | 2.6 deg |
| 120 kt | -4.3 deg | 0.250 | 0.039 | 3.0 deg |
| 160 kt | -5.6 deg | 0.250 | 0.037 | 2.8 deg |
| 170 kt | -6.2 deg | 0.250 | 0.052 | 3.0 deg |

Expected force amplitudes (starting points for tuning):

| Harmonic | Hover | 80 kt | 160 kt |
| --- | --- | --- | --- |
| 1/rev | 0.1 N | 1.0 N | 2.0 N |
| 2/rev (ETL) | 0.5 N | 0.0 N | 0.0 N |
| 2/rev (slap) | 0.0 N | 0.3 N | 0.5 N |
| 5/rev (N) | 0.5 N | 0.5 N | 0.5 N |

### R22 (2-blade teetering reference)

Static config: `blade_count = 2`, `rotation_sign = +1`

Slots 2 and N both render 2/rev. Graph output strategy: populate `Vib2Rev`
only, leave `VibNRev` at zero.

The R22's teetering rotor produces pronounced 2/rev lateral vibration from
the see-saw flapping. `rotor_slap` should be the primary 2/rev driver.


---

## 5. In-Sim Validation

1. **MD500E hover**: fundamental ~8 Hz, feel light 1/rev pulse (~0.3 N).
   `pitch_flap` ~-1.5 deg, `rotor_slap` = 0 → no 2/rev. Correct.
2. **MD500E ETL (15-35 kt)**: 2/rev appears from VRS transition zone.
   Feel a distinct "thumping" overlaid on the 1/rev.
3. **MD500E 80 kt cruise**: 1/rev builds to ~1 N from `blade_alph_pitch` -3.6 deg.
   2/rev light. 5/rev (blade-passing) constant background.
4. **MD500E 160 kt (near Vne)**: 1/rev strong (~2 N), 2/rev from
   `rotor_slap` building. `blade_alpha` approaching stall region.
5. **On ground, rotor turning**: all flapping signals ~0 → vibration
   naturally zero. Disc tilt tracks cyclic but doesn't create vibration.
6. **Autorotation**: RPM changes smoothly, fundamental tracks,
   amplitudes scale with RPM (no glitches).
7. **Rotor brake / shutdown**: amplitudes fade as RPM drops below ~30%;
   no clicks at the `fundamental_hz = 0` transition.

### Pilot sanity checks

* Can a real MD500 pilot identify "this feels like a 500" from vibration alone?
* Does the stick trace an ellipse in hover (1/rev quadrature), not a line?
* Does reversing `rotation_sign` feel noticeably wrong (ellipse goes wrong way)?


---

## 6. Implementation

1. Create `heli_vibration_cyclic.json` sub-graph with ConfigOut + scoped outputs
2. Create `heli_vibration_pedal.json` variant for tail rotor slots
3. Wire vibration sub-graphs into `heli_default.json` with FunctionScope
4. Test MD 500E across flight envelope
5. Create R22 profile override (blade_count = 2)
6. Tune amplitude gains per aircraft


---

## 7. Open Questions

1. **Collective coupling**: collective-axis vibration has physically distinct
   amplitudes (vertical blade-passing dominates). Defer to later or include
   in initial implementation?
2. **Per-harmonic phase control**: specific aircraft may need it. Defer.
