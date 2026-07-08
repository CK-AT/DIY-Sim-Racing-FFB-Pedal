# Helicopter Vibration

Helicopter-specific vibration envelope drivers, slot profiles, graph
templates, and per-aircraft tuning. Uses the DDS subsystem from plan 07
and telemetry signals from plan 04.

**Prerequisites:** Plan 03 (ConfigOut & FunctionScope), Plan 04 (X-Plane signals), Plan 07 (DDS subsystem)


---

## 1. Vibration Sources and Envelope Drivers

All envelope drivers come from live X-Plane datarefs — no synthetic
IAS-based approximations needed. Harmonic ratios are derived from
`blade_count`, set once on profile load.

### Why these specific harmonics

For an N-blade rotor with equally spaced, identical blades, the Coleman
multiblade transformation filters the per-blade rotating-frame loads when
they couple into the fixed (fuselage) frame. The harmonics that survive
are essentially:

* **1/rev** — only when the blades behave non-identically. In forward
  flight the advancing/retreating airspeed asymmetry produces 1/rev disc
  flapping → 1/rev pitch/roll moment at the hub. This is what the pilot
  feels strongly through the cyclic stick.
* **N/rev** — blade-passing. Each blade passing the same azimuth in turn
  produces a vertical bounce + in-plane pulse. Dominant body vibration in
  any healthy heli, scaled by airspeed and gross weight.
* **2N/rev** — second harmonic of blade-passing. Smaller, always present.

Plain integer harmonics like 2/rev or 3/rev are *not* eigenmodes of an
N-blade rotor in general. They appear strongly only on:

* **2-blade teetering rotors** (R22, Bell 47, UH-1) — for these, N/rev =
  2/rev, so the see-saw thump *is* blade-passing.
* **3-blade rotors** (AS350, B206 family) — N/rev = 3/rev.

ETL, blade slap, and retreating blade stall are NOT separate harmonic
lines. They modulate the amplitudes of the existing harmonics:

* **ETL** (16–24 kt transition through the rotor's own wake) is broad-spectrum
  buffeting; what the pilot feels is enhanced N/rev and 2N/rev amplitude.
* **Blade slap (BVI)** — each blade encountering the previous blade's tip
  vortex once per pass → frequency = N/rev.
* **Retreating blade stall** — the stall on the retreating side once per
  disc rotation produces enhanced 1/rev moment (asymmetric pitching) plus
  more N/rev (per-blade pass through the stall region).

### Amplitude envelopes

```
VibFundamental = rpm / 60

-- 1/rev: combination of three sources
--   (a) Forward-flight asymmetric flapping (per-axis blade alpha,
--       already in cyclic axis frame — no axis swap). Dominant at speed.
--   (b) Static mass imbalance: F = m·ω²·r → 1/rev hub force, scales with
--       RPM². Always present, gives the rotor its "alive" feel at idle.
--   (c) Track / aerodynamic imbalance: blade-to-blade coning differences
--       producing a 1/rev vertical force. Scales with RPM, roughly with
--       collective. (b) and (c) are NOT exposed by X-Plane datarefs;
--       they're synthetic parameters that capture real-rotor character.
Vib1Rev_pitch = gain_1rev       * abs(blade_alph_pitch)
              + mass_imbalance  * rpm_norm * rpm_norm
              + track_drift     * rpm_norm
Vib1Rev_roll  = gain_1rev       * abs(blade_alph_roll)
              + mass_imbalance  * rpm_norm * rpm_norm
              + track_drift     * rpm_norm

-- N/rev: blade-passing — dominant body vibration. ETL and high-speed slap
-- modulate the AMPLITUDE of this harmonic, not separate frequencies.
etl_factor = max(0, (vrs - 0.25) / 0.25)                     # 1.0 at hover, 0.0 at 60+ kt
VibNRev = base_nrev * rpm_norm
        + etl_gain  * etl_factor                              # ETL boost on body bounce
        + slap_gain * slap_rat                                # high-speed BVI slap

-- 2N/rev: blade-passing 2nd harmonic, low-level
Vib2NRev = base_2nrev * rpm_norm

-- Retreating blade stall onset (DEFERRED — blade_alpha_deg not yet in UDP)
-- Affects 1/rev (asymmetric moment) and N/rev (per-blade stall pass).
-- Requires adding XPlane.Rotor.BladeAlpha to a future UDP packet version.
rbs_factor = max(0, blade_alpha - rbs_threshold)
Vib1Rev += rbs_1rev_gain * rbs_factor
VibNRev += rbs_nrev_gain * rbs_factor
```


---

## 2. Slot Assignment Profiles

### `heli_cyclic` — helicopter pitch/roll axes

DDS 1 fundamental: main rotor RPM / 60 (gateway-synced).
DDS 2 fundamental: engine RPM / 60 (free-running, no sync needed).

Three rotor harmonics are physically grounded; the remaining two slots
stay zero (firmware reads ratio = 0 as "oscillator disabled").

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | 1/rev — disc flapping in forward flight | `abs(blade_alph)` — **axis-split** |
| 1 | 2 | N (= `blade_count`) | N/rev — blade-passing | RPM + ETL boost + high-speed slap |
| 1 | 3 | 2N (= 2 × `blade_count`) | 2N/rev — blade-passing 2nd | RPM-proportional |
| 1 | 4 | 0 | unused (reserved for RBS once `blade_alpha` UDP lands) | — |
| 1 | 5 | 0 | unused | — |
| 2 | 1 | 1.0 | Engine 1/rev | base + `torque_norm` |
| 2 | 2 | 2.0 | Engine 2/rev | base constant |

For an MD 500E (N = 5), the rotor slots become `[1, 5, 10, 0, 0]`.
For an R22 (N = 2), they become `[1, 2, 4, 0, 0]` — the 2-blade rotor's
characteristic strong "2/rev" feel falls naturally out of N = 2.
For an AS350 (N = 3): `[1, 3, 6, 0, 0]`.

`rotation_sign = +1` (US/CCW) or `-1` (EU/CW). Phase offset per axis is
derived in the parent template (`0°` for pitch, `sign × 90°` for roll).

Graph template: `heli_vibration_cyclic.json`.

### `heli_pedal` — helicopter yaw axis

DDS 1 fundamental: main rotor RPM / 60 (gateway-synced).
DDS 2: disabled (fundamental = 0).

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Main rotor 1/rev (weak on pedals) | RPM-proportional |
| 1 | 2 | N (= main rotor `blade_count`) | Main rotor blade-passing | RPM + ETL + slap |
| 1 | 3 | gear | Tail rotor blade-passing | RPM-proportional |
| 1 | 4 | 2 × gear | Tail rotor 2nd harmonic | RPM-proportional |
| 1 | 5 | 0 | unused | — |

`gear` is the tail-rotor reduction ratio multiplied by tail-rotor blade
count (e.g., 4.62 × 2 ≈ 9.24 for MD 500E's 2-blade TR at 4.62:1; pre-
multiply if the firmware needs a single integer ratio). Tail rotor
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
* `phase` ← derived in parent (`0` for pitch, `sign × 90°` for roll)
* `blade_alph` ← `XPlane.Rotor.BladeAlph{Pitch|Roll}` (per FunctionScope)
* `slap_rat` ← `XPlane.Rotor.Slap`
* `vrs` ← `XPlane.Rotor.VRS`
* `rpm_norm` ← `XPlane.MainRotor.Speed` normalized by `heli_scale`
* `blade_alpha` ← `XPlane.Rotor.BladeAlpha` (DEFERRED — not yet in UDP v4)

### Scoped ConfigOut → FunctionScope's FlightStickConfig

```text
Const(1.0)         → FlightStick.Vib1HarmRatio1   # 1/rev
blade_count        → FlightStick.Vib1HarmRatio2   # N/rev
blade_count × 2    → FlightStick.Vib1HarmRatio3   # 2N/rev
Const(0.0)         → FlightStick.Vib1HarmRatio4   # unused (reserved for RBS)
Const(0.0)         → FlightStick.Vib1HarmRatio5   # unused
phase              → FlightStick.Vib1Phase
```

### Params (exposed in vehicle tab UI)

```
gain_1rev        1/rev amplitude per degree of disc flap (forward-flight component)
mass_imbalance   1/rev synthetic baseline, scales with rpm_norm² (rotor imbalance)
track_drift      1/rev synthetic baseline, scales with rpm_norm (track/aero imbalance)
base_nrev        N/rev base amplitude, RPM-proportional
etl_gain         N/rev amplitude boost in the ETL transition window
slap_gain        N/rev amplitude boost from blade-vortex slap (high speed)
base_2nrev       2N/rev base amplitude, RPM-proportional
```

When the BladeAlpha UDP signal lands, two more params appear: `rbs_1rev_gain`
and `rbs_nrev_gain` (modulating slot 1 and slot 2 amplitudes), plus a
`rbs_threshold` constant.

### Scoped Output → FunctionScope's FlightFfbAction

```
Vib1Ampl1   = gain_1rev × |blade_alph|
            + mass_imbalance × rpm_norm × rpm_norm
            + track_drift × rpm_norm
Vib1Ampl2   = base_nrev × rpm_norm + etl_gain × etl_factor + slap_gain × slap_rat
Vib1Ampl3   = base_2nrev × rpm_norm
Vib1Ampl4   = 0
Vib1Ampl5   = 0
```

### Pedal variant

For pedals with tail rotor vibration, the graph sets ratios from
`blade_count`, `tail_rotor_blade_count`, and `tr_gear_ratio`:

```
slot 1 (ratio 1.0)                              main rotor 1/rev (weak on pedals)
slot 2 (ratio = blade_count)                    main rotor blade-passing
slot 3 (ratio = tr_blade_count × tr_gear_ratio) tail rotor blade-passing
slot 4 (ratio = 2 × tr_blade_count × tr_gear)   tail rotor 2nd harmonic
slot 5 (ratio = 0)                              unused
```

For an MD 500E (2-blade TR at 4.62:1): slots `[1, 5, 9.24, 18.48, 0]`.

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

Static config: `blade_count = 5`, `rotation_sign = +1` (CCW, US)

Cyclic ratios derived from `blade_count`: `[1, 5, 10, 0, 0]`
Pedal ratios: `[1, 5, 9.24, 18.48, 0]` (TR: 2-blade × 4.62:1 gear)

| Slot | Ratio | Freq at 100% RPM (8.2 Hz fundamental) | Dataref driver | Physical cue |
| --- | --- | --- | --- | --- |
| 1 | 1.0 | 8.2 Hz | `blade_alph_{pitch,roll}` | Cyclic-stick disc-flap feel |
| 2 | 5 (N) | 41 Hz | RPM × `rpm_norm` + `vrs` (ETL) + `slap_rat` (BVI) | Body bounce / blade-passing |
| 3 | 10 (2N) | 82 Hz | RPM × `rpm_norm` | 2nd blade-passing |
| 4 | 0 | — | — | Reserved for retreating blade stall (BladeAlpha pending) |
| 5 | 0 | — | — | Spare |

Pedal-specific slots:

| Slot | Ratio | Freq at 100% RPM | Physical cue |
| --- | --- | --- | --- |
| 3 | 9.24 | 75.8 Hz | Tail rotor blade-passing (2-blade × 4.62:1) |
| 4 | 18.48 | 151.6 Hz | Tail rotor 2nd harmonic |

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

| Harmonic component | Hover | 80 kt | 160 kt |
| --- | --- | --- | --- |
| 1/rev forward-flight (`gain_1rev × abs(blade_alph)`) | 0.1 N | 1.0 N | 2.0 N |
| 1/rev mass imbalance (`mass_imbalance × rpm_norm²`) | 0.05 N | 0.05 N | 0.05 N |
| 1/rev track drift (`track_drift × rpm_norm`) | 0.10 N | 0.10 N | 0.10 N |
| N/rev base | 0.5 N | 0.5 N | 0.5 N |
| N/rev ETL boost | 0.5 N | 0.0 N | 0.0 N |
| N/rev slap boost | 0.0 N | 0.3 N | 0.5 N |
| 2N/rev | 0.1 N | 0.1 N | 0.2 N |

Synthetic-imbalance terms (`mass_imbalance`, `track_drift`) tuned for a
"well-maintained but not pristine" rotor. Crank them up for high-time
airframes; zero them for unrealistic perfectly-balanced feel.

### R22 (2-blade teetering reference)

Static config: `blade_count = 2`, `rotation_sign = +1`

Cyclic ratios become `[1, 2, 4, 0, 0]`. The slot 2 = 2/rev that the R22
is famous for falls out of `blade_count = 2` automatically — same wiring
as the MD500E, no special-case logic. Slap is its own dataref so its
amplitude can be tuned higher to capture the R22's see-saw thump.

### AS350 (3-blade reference)

`blade_count = 3` → `[1, 3, 6, 0, 0]`. N/rev = 3/rev gives the
characteristic 3-blade body buzz; same template, same params.


---

## 5. In-Sim Validation

1. **MD500E hover**: fundamental ~8 Hz, light 1/rev pulse (~0.3 N) in cyclic.
   `blade_alph_pitch` ~-1.5 deg, `slap_rat` = 0, `vrs` ≈ 0.5 → ETL boost on
   N/rev (slot 2, 41 Hz) lifts body bounce; 2N/rev (slot 3, 82 Hz) low.
2. **MD500E ETL (15–35 kt)**: ETL boost on slot 2 (N/rev) intensifies
   body bounce. Pilot feels a distinct "thumping" — but it's enhanced
   blade-passing, not a separate harmonic line.
3. **MD500E 80 kt cruise**: 1/rev builds to ~1 N (cyclic, `blade_alph_pitch`
   -3.6 deg). N/rev base + slap_rat starting to add. 2N/rev steady background.
4. **MD500E 160 kt (near Vne)**: 1/rev strong (~2 N). Slap on N/rev adds
   body buzz. `blade_alpha` approaching stall — once UDP exposes it, slot
   1 and slot 2 amplitudes get an additional kick from RBS.
5. **On ground, rotor turning**: `blade_alph` ~0 (no forward flight) so
   the forward-flight 1/rev term is gone, but `mass_imbalance × rpm_norm²`
   and `track_drift × rpm_norm` still produce a small always-on 1/rev pulse
   at slot 1. N/rev base term produces a steady idle hum at slot 2. Together
   these give the rotor its "alive" feel even at idle.
6. **Autorotation**: RPM changes smoothly, fundamental tracks, amplitudes
   scale with `rpm_norm` on slots 2 + 3 (no glitches).
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
