# MSFS 2024 Helicopter Telemetry & Derived Cues

Bring MSFS 2024 helicopters up to feature parity with the existing X-Plane
helicopter pipeline (plans 04, 05, 08). The hard problem is that MSFS's
SimConnect surface does **not** expose the per-blade aerodynamic state that
the X-Plane plugin currently relies on. This plan covers how to bridge the
sim, which signals come for free, and how to **derive** the missing rotor
cues from the variables MSFS does expose.

**Prerequisites:** Plan 04 (X-Plane signals — provides the reference signal
contract), Plan 05 (heli load forces), Plan 08 (heli vibration), Plan 07
(DDS subsystem).

**Out of scope:** fixed-wing (covered by tier 1 only — see §10).

---

## 1. Goal & Strategy

**Top priority: ship something testable end-to-end ASAP.** Calibration,
fidelity, and namespace cleanup come later. The fastest path:

1. Build the SimConnect bridge with all natively-available SimVars +
   the **tier-C IAS-only derivations** (see §3.1) for `BladeAlph`,
   `VRS`, `Slap`, `Propwash`. No reliance on caveat-laden SimVars.
   No regression notebook gates this step — physics-grounded defaults
   only.
2. Ship a new intermediate graph `heli_unboosted_msfs.json` (see §3.7)
   wired to those derived signals.
3. Fly an MSFS heli end-to-end. Confirm the bridge streams, the graph
   evaluates, the ESP32 actuates, no crashes.
4. **Only then** run the X-Plane calibration campaign (§4) to replace
   the rough defaults with fitted constants.
5. If phase 0b clears the "multiplayer: far aircraft only" caveat
   (§2.3), layer tier-A or tier-B `BladeAlph` derivations on top as an
   optional refinement.

Why this order: the bridge + intermediate graph captures the
**dominant** unboosted-heli cues (collective load, RPM-scaled pedals,
speed-dependent cyclic load, ETL bump, cruise slap) on day one. Plan
04 §3 already established that these are mostly IAS-driven, so
IAS-only derivations are not a placeholder — they're the right baseline.
The calibration campaign improves numeric accuracy; it doesn't unlock
new cues.

### Signals consumed by `heli_unboosted` and their MSFS path

| Signal in graph | MSFS path (intermediate graph) | Cue produced |
| --- | --- | --- |
| `MainRotor.Speed` (rpm) | Direct: `ROTOR RPM:0` | RPM-scaled pedal spring, DDS fundamental, all vibration RPM-scaling |
| `MainRotor.Torque` | `ENG TORQUE PERCENT:1 × max_torque_nm` (§3.5) | Collective load force, collective vibration |
| `Rotor.BladeAlphPitch` | **Derived** tier C: `−k · mu` from `AIRSPEED INDICATED` (§3.1) | Cyclic longitudinal load force; 1/rev pitch vibration amplitude |
| `Rotor.BladeAlphRoll` | **Derived** tier C: `−k · BetaDeg · mu` | Cyclic lateral load (weak); 1/rev roll vibration amplitude |
| `Rotor.VRS` | **Derived** from IAS + `VELOCITY WORLD Y` (§3.2) | ETL bump on N/rev pitch/roll/collective vibration |
| `Rotor.Slap` | **Derived** linear IAS ramp (§3.3) | High-speed BVI on N/rev pitch/roll/collective |
| `Rotor.Propwash` | **Derived** from weight × G + IAS (§3.4) | Downwash/ground-effect proxy (small effect — not consumed by `heli_unboosted` directly; future use) |

`BladeAlphPitch` is the single most important signal — it drives **both**
the dominant cyclic vibration and the dominant cyclic load force on
unboosted rotors. Plan 04 §3 already showed it is dominated by airspeed,
so the tier-C `mu`-only derivation captures the dominant physics on day
one. Calibration tightens the numbers; tier A/B refinements add
manoeuvre-induced subtleties only.

---

## 2. MSFS SimConnect Surface (what we get for free)

Verified against the [official MSFS SDK SimVar
reference](https://docs.flightsimulator.com/html/Programming_Tools/SimVars/Simulation_Variables.htm),
specifically the [Helicopter
Variables](https://docs.flightsimulator.com/html/Programming_Tools/SimVars/Helicopter_Variables.htm),
[Aircraft Misc
Variables](https://docs.flightsimulator.com/html/Programming_Tools/SimVars/Aircraft_SimVars/Aircraft_Misc_Variables.htm),
[Aircraft Flight Model
Variables](https://docs.flightsimulator.com/html/Programming_Tools/SimVars/Aircraft_SimVars/Aircraft_FlightModel_Variables.htm),
and [Miscellaneous
Variables](https://docs.flightsimulator.com/html/Programming_Tools/SimVars/Miscellaneous_Variables.htm)
pages.

### Available — direct mapping

| MSFS SimVar | Units (SDK) | Maps to |
| --- | --- | --- |
| `AIRSPEED INDICATED` | knots | `MSFS.Speed.IAS` |
| `AIRSPEED TRUE` | knots | (also useful — true vs indicated divergence at altitude) |
| `INCIDENCE ALPHA` | radians | `MSFS.Angle.Alpha` (×180/π) |
| `INCIDENCE BETA` | radians | `MSFS.Angle.Beta` (×180/π) |
| `ROTATION VELOCITY BODY X/Y/Z` | fps per docs (likely rad/s — see §10 Q3) | `MSFS.Rate.Pitch/Roll/Yaw` |
| `ROTATION ACCELERATION BODY X/Y/Z` | rad/s² | (rate-damping derivatives) |
| `VELOCITY BODY X/Y/Z` | fps | (longitudinal/lateral airspeed components — propwash, advance ratio) |
| `RELATIVE WIND VELOCITY BODY X/Y/Z` | fps | (true relative-wind components — preferred over `VELOCITY BODY` for derivations in §3) |
| `G FORCE` | gforce | `MSFS.G_Nrml` |
| `ROTOR RPM:index` | RPM | `MSFS.MainRotor.Speed` (`:0` main, `:1` tail) |
| `ROTOR RPM PCT:index` | percent over 100 | (normalized — convenient for graph params) |
| `ENG TORQUE PERCENT:index` | percent scalar 16K | `MSFS.MainRotor.Torque` — see §3.5 for conversion |
| `ENG ROTOR RPM:index` | percent scalar 16K | (alternative engine-side rotor RPM read) |
| `ROTOR ROTATION ANGLE:index` | radians | `MSFS.Rotor.Azimuth` — actual rotor disc azimuth, candidate for DDS phase sync |
| `ROTOR COLLECTIVE BLADE PITCH PCT` | percent over 100 | (collective blade pitch — better than `COLLECTIVE POSITION` for load force) |
| `ROTOR CYCLIC BLADE PITCH PCT` | percent over 100 | (cyclic pitch magnitude — see §3.1 fallback) |
| `ROTOR CYCLIC BLADE MAX PITCH POSITION` | degrees | (azimuth of max cyclic pitch — cyclic input direction) |
| `TAIL ROTOR BLADE PITCH PCT` | percent over 100 | `MSFS.TailRotor.Pitch` — direct pedal-output cue |
| `TAIL ROTOR PEDAL POSITION` | percent over 100 | (pilot pedal input) |
| `COLLECTIVE POSITION` | percent over 100 | (pilot collective input) |
| `DISK PITCH ANGLE:index` | radians | (see §3.1 + MP caveat in §2.3) |
| `DISK BANK ANGLE:index` | radians | (see §3.1 + MP caveat in §2.3) |
| `DISK PITCH PCT:index` | percent over 100 | (normalized form) |
| `DISK BANK PCT:index` | percent over 100 | (normalized form) |
| `DISK CONING PCT:index` | percent over 100 | (intermediate) |
| `ROTOR LATERAL TRIM PCT` / `LONGITUDINAL TRIM PCT` | percent over 100 | `MSFS.Trim.Lateral`/`Longitudinal` |
| `VERTICAL SPEED` | fps | (intermediate — VRS derivation; instrument-style, can lag) |
| `VELOCITY WORLD Y` | fps | (true earth-frame VV — preferred VRS input over `VERTICAL SPEED`) |
| `TOTAL WEIGHT` | pounds | (rotor lift derivation: see §3.4) |
| `AMBIENT DENSITY` | slugs/ft³ | (momentum-theory propwash, §3.4) |
| `SIM ON GROUND` | bool | `MSFS.OnGround` |
| `STRUCT ROTOR POSITION:index` | feet (XYZ struct) | (rotor location in airframe — useful for tip-speed via radius if not directly exposed) |
| `IS ATTACHED TO SLING` | bool | (load-state cue for slingload feel — future work) |

### NOT available — must derive

| X-Plane signal | MSFS counterpart | Notes |
| --- | --- | --- |
| `cyclic_elev_blad_alph[N]` | none | Per-blade alpha is internal to MSFS's flight model. The closest pair are `DISK PITCH ANGLE` (response) and `ROTOR CYCLIC BLADE PITCH PCT` + `ROTOR CYCLIC BLADE MAX PITCH POSITION` (command). Must derive — see §3.1. |
| `cyclic_ailn_blad_alph[N]` | none | Same. |
| `rotor_blade_slap_rat[N]` | none | Derive from IAS — see §3.3. |
| `vortex_ring_state[N]` | none | Derive from low-IAS + descent — see §3.2. |
| `propwash_mtr_sec[N]` | none | Derive from weight + G + IAS via momentum theory — see §3.4. |
| `rotor_lift` (X-Plane has none directly either) | **NO `ROTOR LIFT` SimVar exists** | Was incorrectly listed in early draft. Derive as `T ≈ TOTAL_WEIGHT_lb × 4.4482 × G_FORCE` (steady flight). |
| `L_aero / M_aero / N_aero` | `ROTATION ACCELERATION BODY × I_body` (inertia tensor not exposed) | Not needed for heli — only fixed-wing aero-torque load model that plan 06 already deprecates. |
| `rotor_radius_mtr` | not directly | Per-aircraft constant from graph params; alternatively derive from `STRUCT ROTOR POSITION` if rotor hub position vs blade tip is recoverable (unlikely useful in practice — just configure as a constant). |
| `n_blades` | not in SimVars | Per-aircraft constant from graph params (same as existing X-Plane heli profiles). |

### 2.3 Critical caveat: "multiplayer: far aircraft only"

The MSFS SDK annotates most rotor-disc SimVars (`DISK BANK ANGLE`,
`DISK PITCH ANGLE`, `DISK CONING PCT`, `ROTOR ROTATION ANGLE`,
`ROTOR LATERAL/LONGITUDINAL TRIM PCT`, `ROTOR COLLECTIVE BLADE PITCH PCT`,
`ROTOR CYCLIC BLADE PITCH PCT`, `ROTOR CYCLIC BLADE MAX PITCH POSITION`)
with the phrase **"multiplayer: far aircraft only"**.

The convention is ambiguous. It usually means *the value is also
provided as a network update for visualising other aircraft*, not
*the value is unavailable for the user's own aircraft*. But this
needs explicit empirical verification before we depend on these
signals for the player's own helicopter (which is the entire point of
this plan).

**Phase 0 verification (added):** write a minimal SimConnect probe that
reads `DISK PITCH ANGLE:0` and `ROTOR CYCLIC BLADE PITCH PCT` while the
user pilots a default MSFS 2024 helicopter (Bell 407, H145, Cabri G2,
or AS350). If the values move with cyclic input, we can rely on them.
If they stay zero or static, all §3.1 derivations must use only
unannotated SimVars (`AIRSPEED INDICATED`, `VELOCITY BODY`,
`G FORCE`, body rates, attitude). This is doable but loses the
disc-tilt term entirely — falls back to the pure-speed §3.1
alternative model.

### 2.4 MSFS 2024 native blade datarefs

The MSFS 2024 SDK helicopter page does **not** mark anything as "new in
2024". The blade-pitch percents (`ROTOR COLLECTIVE BLADE PITCH PCT`,
`ROTOR CYCLIC BLADE PITCH PCT`, `TAIL ROTOR BLADE PITCH PCT`,
`ROTOR CYCLIC BLADE MAX PITCH POSITION`) appear unchanged from the
MSFS 2020 surface. If MSFS 2024-specific SimVars exist they're not
on the public SDK page yet — re-check release notes during phase 0.

---

## 3. Derivation of Missing Cues

These are the synthetic replacements for the signals SimConnect doesn't
publish. Each is calibrated against the published X-Plane MD 500E data
from plan 04 §3 — that flight log gives us a reference curve to match.

The driving principle: **match the shape and the cross-coupling, not the
exact numeric value**. The graphs already scale every input via a per-axis
gain parameter; what matters is that `BladeAlphPitch` rises monotonically
with IAS, that VRS is high in hover and drops past ETL, and that slap
rises with speed. The pilot calibrates the rest with the gain knobs.

### 3.1 `Rotor.BladeAlphPitch` and `Rotor.BladeAlphRoll`

**Physics.** Per-axis blade alpha in the cyclic frame measures
advancing/retreating asymmetry. It's driven by the advance ratio
`μ = V_horiz / (Ω·R)` and the disc tilt relative to the relative wind.
X-Plane exposes this directly as `cyclic_elev_blad_alph` / `_ailn_`,
already decomposed into the cyclic axes.

MSFS exposes three candidate disc-state SimVars (all with the §2.3 MP
caveat to verify in phase 0):

* `DISK PITCH ANGLE` / `DISK BANK ANGLE` (radians) — disc-plane
  attitude *response*, includes flapping.
* `ROTOR CYCLIC BLADE PITCH PCT` + `ROTOR CYCLIC BLADE MAX PITCH POSITION`
  — cyclic *command*: magnitude (%) and azimuth (degrees) of maximum
  cyclic pitch. Together these encode the cyclic input vector. Decompose:
  `cmd_pitch = pct * cos(max_pos_rad)`, `cmd_roll = pct * sin(max_pos_rad)`
  (sign convention TBD by calibration).
* Always available, no caveat: `AIRSPEED INDICATED`, `VELOCITY BODY Z`
  (longitudinal speed), body attitude (`PLANE PITCH/BANK DEGREES`).

Three derivation tiers, in order of preference:

```text
mu = max(0, ias_kts) / rotor_tip_speed_kts   # per-aircraft constant
# tip-speed example: MD 500E ≈ 488 ft/s ≈ 290 kt at full RPM

# TIER A — disc response (preferred IF DISK PITCH ANGLE is user-readable)
BladeAlphPitch_synth = k_pitch_disc * mu * DiskPitchAngle_deg
BladeAlphRoll_synth  = k_roll_disc  * mu * DiskBankAngle_deg

# TIER B — cyclic command (fallback IF blade pitch SimVars are user-readable
#          but DISK ANGLEs are not)
cyc_lon = ROTOR_CYCLIC_BLADE_PITCH_PCT * cos(ROTOR_CYCLIC_BLADE_MAX_PITCH_POSITION_rad)
cyc_lat = ROTOR_CYCLIC_BLADE_PITCH_PCT * sin(ROTOR_CYCLIC_BLADE_MAX_PITCH_POSITION_rad)
BladeAlphPitch_synth = k_pitch_cyc * mu * cyc_lon
BladeAlphRoll_synth  = k_roll_cyc  * mu * cyc_lat

# TIER C — pure speed (fallback IF all rotor SimVars are MP-only)
BladeAlphPitch_synth = -k_speed_pitch * (mu - mu_hover)
BladeAlphRoll_synth  = -k_speed_roll  * BetaDeg * mu
```

Sign convention: X-Plane reports negative `blade_alph_pitch` in forward
flight (range -0.2 hover → -6.2 at 170 kt). Calibrate each tier's
`k_*` constants from §4's regression against the X-Plane reference table.

**Recommendation:** ship tier C first (cannot fail — only depends on
SimVars without caveats) and validate the §4 calibration produces a
satisfactory curve. Then once phase-0 verification clears the MP
caveat, layer tier B (or A) on top as an optional refinement. The
plan 04 §3 data already showed blade_alph_pitch is dominated by speed
and *non-monotonic* with g-load — so even tier C captures the
dominant physics. The disc/cyclic terms add manoeuvre dynamics; they
are not load-bearing.

### 3.2 `Rotor.VRS`

**Physics.** VRS occurs in low forward speed + significant descent
through the rotor's own downwash. X-Plane's `vortex_ring_state` is
0.50 in hover (full recirculation) and decays to 0.25 past ETL
(~60 kt).

```text
descent_fpm = max(0, -vertical_speed_fpm)    # 0 in climb/level

# Hover/low-speed recirculation factor: 1.0 at hover, 0 past ETL
fwd_factor = clamp(1.0 - ias_kts / etl_speed_kts, 0, 1)    # etl_speed ≈ 60 kt

# Descent-through-own-wake amplifier
desc_factor = clamp(descent_fpm / 1500.0, 0, 1)

VRS_synth = 0.25 + 0.25 * fwd_factor * (0.5 + 0.5 * desc_factor)
           # = 0.50 at hover with descent, ~0.25 in cruise, 0.375 at hover in climb
```

This matches the qualitative shape the ETL envelope (`etl_factor` in
plan 08 §1) actually uses: graphs only consume `(vrs - 0.25) / 0.25`.

### 3.3 `Rotor.Slap`

**Physics.** Blade-vortex interaction; builds with forward speed when
the rotor wake skews back enough that each blade strikes the previous
blade's tip vortex. X-Plane range: 0 (hover) to 0.05+ at 170 kt,
monotonically increasing.

```text
Slap_synth = clamp(k_slap * max(0, ias_kts - slap_onset_kts) / 100.0, 0, slap_max)
            # slap_onset ≈ 20 kt, k_slap ≈ 0.5
```

Simple linear ramp matches the plan 04 §3 table within calibration
tolerance. Could be refined with disc tilt, but the underlying
vibration gain knob (`slap_gain` in plan 08) covers individual-aircraft
variation already.

### 3.4 `Rotor.Propwash`

**Physics.** Downwash velocity below the disc. X-Plane reports m/s:
19 in hover, dropping to 2.5 at 170 kt as forward velocity carries the
wake away. Rotor momentum theory: `v_i ≈ sqrt(T / (2·ρ·A))` in hover,
diminishing with forward flight.

**MSFS gotcha:** there is no `ROTOR LIFT` SimVar. Derive thrust from
weight and load factor (in steady or quasi-steady flight, thrust
balances weight × G):

```text
# Thrust ≈ weight × load factor (good to ~5% except in violent transients)
weight_N    = TOTAL_WEIGHT_lb * 4.4482               # lb → N
rotor_lift_N = weight_N * G_FORCE                    # SimVar units: g
rho_kgm3    = AMBIENT_DENSITY_slugs_ft3 * 515.379    # slugs/ft³ → kg/m³ (usually ≈1.225)

# Hover induced velocity from disc loading
v_i_hover = sqrt(rotor_lift_N / (2 * rho_kgm3 * disc_area_m2))      # ≈ 19 m/s MD 500E

# Forward flight reduction: roughly 1 / sqrt(1 + (V/v_i)^2)
v_fwd = ias_kts * 0.5144                                            # kt → m/s
Propwash_synth = v_i_hover / sqrt(1 + (v_fwd / v_i_hover)^2)
```

`disc_area_m2` is per-aircraft (MD 500E ≈ 49 m²). Provided as a graph
parameter, same as `rotor_tip_speed_kts`. Both go into the per-aircraft
graph param block, no plumbing changes.

### 3.5 `MainRotor.Torque`

**MSFS exposes `ENG TORQUE PERCENT:index` (percent scalar 16K), not an
absolute torque.** Convert via a per-aircraft `max_torque_nm` constant:

```text
torque_pct = ENG_TORQUE_PERCENT * (16384 / 100)   # SDK "percent scalar 16K" convention
MainRotor_Torque_Nm = torque_pct * max_torque_nm  # max_torque per-aircraft
```

`max_torque_nm` becomes another per-aircraft graph parameter alongside
`rotor_tip_speed_kts`, `disc_area_m2`. Document defaults for the
calibrated aircraft set (MD 500E, R22, Bell 206, etc.). The collective
load curve in plan 05 §5 already operates on torque-normalized inputs,
so the absolute scale only needs to be in the right ballpark for the
graph to feel right.

### 3.6 What we deliberately don't try to derive

* `BladeAlpha` (3/rev retreating stall indicator) — X-Plane defers this
  too (plan 04 §1). Future work if MSFS exposes blade-pitch SimVars.
* `AeroTorque.L/M/N` — only used by the soon-to-be-deprecated fixed-wing
  aero-torque load model (plan 06). For heli not needed.

### 3.7 Intermediate graph: `heli_unboosted_msfs.json`

The **first testable deliverable** is a new graph template that wires
the existing `heli_cyclic_unboosted.json`, `heli_collective_unboosted.json`,
`heli_pedals_unboosted.json`, `heli_vibration_*.json` sub-graphs to
MSFS-derived inputs instead of `XPlane.*`.

Structurally identical to [SimHubPlugin/graphs/templates/heli_unboosted.json](SimHubPlugin/graphs/templates/heli_unboosted.json)
but with input nodes changed:

| `heli_unboosted.json` | `heli_unboosted_msfs.json` |
| --- | --- |
| `in_torque` SignalGroup `XPlane`, suffixes `MainRotor.Torque/Speed` | SignalGroup `MSFS`, same suffixes |
| `in_blade_alph_pitch` from `XPlane.Rotor.BladeAlphPitch` | from `MSFS.Rotor.BladeAlphPitch` (derived tier C in bridge) |
| `in_blade_alph_roll` from `XPlane.Rotor.BladeAlphRoll` | from `MSFS.Rotor.BladeAlphRoll` (derived) |
| `in_vrs` from `XPlane.Rotor.VRS` | from `MSFS.Rotor.VRS` (derived) |
| `in_slap` from `XPlane.Rotor.Slap` | from `MSFS.Rotor.Slap` (derived) |

All `Cyclic.*`, `Aircraft.BladeCount`, `Aircraft.RotationSign`, etc.
params carry over unchanged. Identical sub-graph wiring. The graph
*does not know* its inputs are derived — that's the bridge's job.

This makes the graph swap a ~1-day mechanical clone+rewire job once
the bridge is up. **Critical for ASAP testability**: the moment the
bridge ships an `MSFS.Rotor.BladeAlphPitch` UDP packet, this graph runs
end-to-end with the existing ESP32 stack and the existing pedal hardware.

Implementation note: do not block this on the `Flight.*` namespace
abstraction (§6). Ship `heli_unboosted_msfs.json` as a sibling of
`heli_unboosted.json`, accept some duplication, fold both into a
single `heli_unboosted.json` referencing `Flight.*` only when phase 4
happens.

---

## 4. Calibration (post first-light)

**This section is a refinement, not a gate.** Phase 1 ships the bridge
and intermediate graph using physics-grounded *default* constants
(rotor tip speeds from published tech sheets, slap onset 20 kt, ETL
speed 60 kt, etc. — see §3 inline values). Those defaults will already
produce a believable feel. The calibration campaign happens *after*
end-to-end testability is established (phase 2), to swap rough
defaults for fitted constants.

The exception: extending the DataRefLogger (§4.1) is cheap and can
run **in parallel** with bridge work, since it only touches X-Plane code.

The advantage we exploit: X-Plane exposes **both** the ground-truth
signals (`blade_alph_pitch`, `slap_rat`, `vrs_0`, `propwash_mps`) AND
the inputs we plan to feed the derivations in §3 (IAS, disc tilt,
vertical speed, rotor RPM, alpha/beta, etc.). That means we can **fit
the derivation formulas offline against X-Plane flight data** rather
than guessing.

The existing [XPlanePlugin/DataRefLogger.cpp](XPlanePlugin/DataRefLogger.cpp)
already captures most of what we need. The campaign extends it slightly,
runs a structured flight profile, and produces calibrated constants for §3.

### 4.1 DataRefLogger additions

The current logger captures:

* **Ground-truth signals** for derivation targets: `blade_alph_pitch`,
  `blade_alph_roll`, `slap_rat`, `vrs_0`, `propwash_mps`, `blade_alpha`,
  `disc_alpha`
* **MSFS-equivalent inputs** already present: `ias_kts`, `tas_mps`,
  `alpha_deg`, `beta_deg`, `g_nrml`, `P/Q/R_rad_s`, `pitch_deg`,
  `roll_deg`, `vvi_fpm`, `gs_mps`, `on_ground`, `rpm_main`/`tail`,
  `torque_main`/`tail`, `yoke_pitch/roll/yaw`, `collective`,
  `cycli_pitch`, `cycli_roll`, `rotor_radius`

Add the following datarefs to fully parallel the MSFS SimConnect surface
we'll have in §2 and to enable the propwash/VRS derivations:

| Add | Dataref | MSFS counterpart | Purpose |
| --- | --- | --- | --- |
| `m_total_kg` | `sim/flightmodel/weight/m_total` | `TOTAL WEIGHT` (lb) | Rotor lift derivation: T ≈ m·g·n |
| `local_vy_mps` | `sim/flightmodel/position/local_vy` | `VELOCITY WORLD Y` | True earth-frame VV — VRS input that won't lag like VSI |
| `rho` | `sim/weather/rho` | `AMBIENT DENSITY` | Momentum-theory propwash: v_i = √(T/2ρA) |
| `disc_pitch_actual_deg`* | `sim/flightmodel2/engines/rotor_disc_pitch_deg` (verify name) | `DISK PITCH ANGLE` (rad) | Actual disc-plane attitude including flapping — closer match to MSFS's signal than the cyclic-input-only `cycli_pitch` already logged |
| `disc_roll_actual_deg`* | `sim/flightmodel2/engines/rotor_disc_roll_deg` (verify name) | `DISK BANK ANGLE` (rad) | Same, lateral |
| `n_blades` | `sim/aircraft/prop/acf_num_blades` (per-engine, int) | `ROTOR LIFT` (lb) doesn't expose this — set per-aircraft in MSFS bridge config | Harmonic ratios in plan 08 |
| `omega_rad_s` | `sim/flightmodel/engine/POINT_omega` | `ENG ROTOR RPM` × π/30 | Direct angular velocity, avoids rounding through RPM |

*Verify exact dataref names against DataRefs.txt before merging — if the
"actual disc plane" datarefs don't exist or only expose cyclic-input
tilt, document and either (a) compute disc tilt from cyclic input +
estimated flapping or (b) accept that this signal will be sim-specific
and excluded from the MSFS derivation.

Aircraft constants (rotor_radius, m_total at start, n_blades) are logged
every sample for simplicity — 20 Hz × 8 extra floats is negligible CSV
overhead.

### 4.2 Flight test profile

One profile per helicopter class, in this order:

1. **Cold-start trim sweep** (30 s on ground, rotor spooling) —
   establishes baseline noise floor for all signals.
2. **Hover hold** (60 s in ground effect, 60 s out of ground effect at
   100 ft) — high VRS, low blade alpha, zero slap. Captures
   `vrs_0` baseline at hover, propwash hover peak.
3. **ETL transition sweep** (3 passes through 15–30 kt, accelerate +
   decelerate) — VRS should swing 0.50 → 0.25 → 0.50 visibly.
4. **Forward speed ladder** (5 s stabilised at 20, 40, 60, 80, 100,
   120, 140, 160, VNE) — the headline curve we'll regress against.
   Reproduces plan 04 §3 table.
5. **Banked turn sweep** (level 30°, 45°, 60° banked turns left and
   right at 80 kt) — exercises lateral blade alpha + roll-rate cues.
6. **G-load sweep** (gentle to aggressive pull-ups at 100 kt) —
   confirms blade_alph is dominated by speed not G (plan 04 §3
   finding) and validates that our derivation reproduces this.
7. **Vertical descent / VRS approach** (controlled descent <40 kt with
   descent rate ramping from 300 fpm to 1500+ fpm) — the canonical
   VRS envelope, the hardest case for the derivation in §3.2.
8. **Autorotation entry + recovery** — torque/RPM transients that
   exercise pedal-spring scaling.

Each segment ≥30 s for binning robustness. Total ~25 minutes airborne
per profile. Run the profile across at least:

* One **unboosted** type (MD 500E, the existing X-Plane reference)
* One **2-blade** teetering type (R22 or Bell 47 — to validate harmonic-ratio
  handling in §3 derivations holds at N=2)
* One **boosted** type (Bell 206) — sanity check, derivation should still produce sensible numbers even though graph doesn't consume them as load force

### 4.3 Offline regression

Python notebook in `docs/calibration/` consumes the CSV and:

1. Reproduces the plan 04 §3 IAS-binned table for **ground truth** —
   confirms data quality matches the prior dataset.
2. For each derived signal in §3, computes the synthetic value from the
   logged inputs and joins against ground truth.
3. **Fits the constants** (`k_pitch`, `k_speed_pitch`, `slap_onset_kts`,
   `etl_speed_kts`, etc.) by least-squares regression on the per-bin
   averages — not raw samples, to avoid over-weighting noisy hover data.
4. Reports per-signal residual stats: R², max relative error,
   IAS region where the model is worst.
5. **Sensitivity analysis**: which inputs does each derivation actually
   depend on? If `BladeAlphPitch` regression shows `mu × disc_tilt`
   adds <5% explanatory power over pure `mu`, drop the disc-tilt term
   and simplify §3.1.

### 4.4 Deliverables

* Extended `DataRefLogger.cpp` (additions in §4.1)
* `docs/calibration/MD500E.csv`, `R22.csv`, `B206.csv` (one per type)
* `docs/calibration/derive_msfs_signals.ipynb` — regression notebook
* `docs/calibration/calibrated_constants.md` — final fitted values
  for each per-aircraft and per-aircraft-class constant, plus residual
  diagnostics
* **Updated §3 of this plan** with constants replaced by fitted values
  and any formula simplifications justified by the sensitivity analysis

### 4.5 What this buys us (post first-light)

By phase 2, end-to-end is already working with rough defaults. The
campaign:

* Replaces hand-picked constants with fitted values — typically tightens
  numeric error from ±50% to ±10%.
* Tells us which constants are aircraft-class properties vs
  per-aircraft tuning (sensitivity analysis).
* Documents calibration *defaults* for the X-Plane heli graphs too —
  the X-Plane plugin currently uses ground truth, but the constants
  matter for any aircraft where ground truth differs from average
  (delta-3 hinges, two-blade teetering quirks).
* The regression notebook is reusable for any future signal additions.

This is "make it accurate" work, not "make it work" work. The latter
ships in phase 1.

### 4.6 In-MSFS validation (after bridge ships)

Once the SimConnect bridge ships (phase 1) and the derivations from §3
are wired (phase 2), repeat profile §4.2 in MSFS 2024 with the bridge
in logging mode:

* For signals MSFS exposes directly, log raw vs derived to confirm
  no bridge-side bugs.
* For derived signals, we have no ground truth — but we can compare
  **shape** against the X-Plane curves at matched IAS bins. If MSFS
  blade-alpha-derived feels totally wrong in flight, that's a sign
  MSFS's underlying disc-tilt / IAS signals differ in scale or sign
  from X-Plane's and the calibrated constants need a sim-specific
  override. Document the override as part of phase 3.

Cross-sim spreadsheet `docs/calibration/xplane_vs_msfs.md` records any
divergences and their resolutions — feeds into the phase 4 `Flight.*`
abstraction decision.

---

## 5. Implementation: SimConnect Bridge

### Architecture

Mirror the X-Plane pattern exactly:

```text
MSFS 2024  ──SimConnect──►  MsfsFfbDataProvider.exe  ──UDP──►  SimHub plugin
                            (out-of-process)                   (existing
                                                                listener loop)
```

* New folder: `MsfsPlugin/` next to existing `XPlanePlugin/`
* Native C++ SimConnect client (matches X-Plane plugin's C++ style)
* Publishes a binary UDP packet with magic `0x4D464642` ("MFFB") on the
  same port the X-Plane receiver uses — or a separate configurable port
* Same struct layout shape as `XPlaneUdpPacket` so the SimHub-side
  parser can be near-identical

### Why out-of-process and not a WASM gauge

* SimConnect from an external EXE has access to every SimVar we need
  (none of the derived inputs require gauge-API LVars)
* Easier to develop/debug than a WASM module sandboxed inside the sim
* User can launch it independently of MSFS (script + auto-start)
* WASM would only be required if we discover we need a SimVar that's
  gated to in-sim modules — unlikely for the cues in §3

### SimHub-side changes

Two options:

**Option A — second listener (recommended).** Add an `MsfsUdpReceiver`
parallel to `StartXPlaneUdpReceiver` in [DiyFfbPlugin.cs:937](SimHubPlugin/DiyFfbPlugin.cs#L937).
Separate thread, separate port (configurable), separate packet magic.
Signals published as `MSFS.*`.

**Option B — multiplex.** One receiver, magic word selects packet
flavour. Simpler config but couples the two source plugins.

Recommend A: zero risk to the X-Plane path.

### Settings

Add to `DiyFfbPluginSettings`:

* `MsfsUdpEnabled` (bool, default false)
* `MsfsUdpPort` (ushort, default e.g. 49001)
* Per-aircraft constants for derivations (`MsfsRotorTipSpeedKts`,
  `MsfsDiscAreaM2`, etc.) — or roll these into graph params per profile

The per-aircraft constants are really aircraft-not-sim properties.
Better to expose them as **graph parameters** on `heli_*` templates so
they live with the rest of the per-aircraft tuning, not in plugin
settings. The bridge sends raw SimVars (`RotorLift_N`, `IAS_kts`); the
derivation lives in the graph.

### Where the derivation lives

Two options:

**A. In the bridge** — bridge runs the formulas in §3 and ships derived
`MSFS.Rotor.BladeAlphPitch` directly. Pro: SimHub plugin doesn't change
graph signal catalogue. Con: per-aircraft constants need to live in the
bridge config, away from the graph parameters.

**B. In the graph** — bridge ships only raw SimVars (`MSFS.Raw.DiskPitchAngle`,
`MSFS.Raw.RotorLift_N`, `MSFS.Raw.VerticalSpeed`). The `heli_*` graphs
gain a derivation sub-graph that produces the synthetic `Rotor.*`. Pro:
all tuning lives in one place; users can override derivations per
aircraft via graph parameters. Con: graphs get more complex; can't
easily reuse with X-Plane bridge.

**Recommend a hybrid:** ship both. Bridge publishes derived `MSFS.Rotor.*`
with reasonable defaults (so the graphs work out-of-box), AND publishes
the raw inputs (`MSFS.Raw.*`) so power users can route around the
default derivations via an alternate sub-graph. This matches how
SimHub-native data is treated elsewhere.

---

## 6. Signal Namespace: `MSFS.*` vs `Flight.*`

Two routes (raised in the prior message):

### Route 1 — keep them separate: `XPlane.*` and `MSFS.*`

* Simplest to implement
* Requires duplicating every `heli_*` template per sim (`heli_unboosted_xplane.json`
  vs `heli_unboosted_msfs.json`)
* Or: the user has to manually rewire signal inputs when switching sims

### Route 2 — abstract: `Flight.*`

Introduce a sim-agnostic namespace `Flight.Speed.IAS`, `Flight.Rotor.BladeAlphPitch`,
etc. A "sim selector" routes whichever bridge is currently providing data into
the `Flight.*` slot. Graph templates only reference `Flight.*`.

* One template per heli class, works for any sim
* Cleaner long-term — matches how the rest of the FFB stack abstracts
  hardware vs sim
* Migration cost: every `heli_*` and `plane_*` template, every signal
  reference in [GraphSignalCatalogData.cs:48](SimHubPlugin/GraphSignalCatalogData.cs#L48),
  every plan doc that hardcodes `XPlane.` (plans 04, 05, 06, 08, 09)

### Recommendation

**Phase 1 ships under `MSFS.*` to validate the bridge and the derivations
in isolation.** Once both bridges produce equivalent signals and the
derivations are tuned, **phase 3 does the `Flight.*` migration** in a
single sweep with a sed-style rename across templates and a thin alias
shim in `GraphSignals.cs` so external configs don't break immediately.

The migration is mechanical but invasive; doing it after both sims work
means we know we got the abstraction right.

---

## 7. Validation

Per-tier acceptance:

**Tier 1 (basic SimVars, no derivations):** `MSFS.Speed.IAS`,
`MSFS.G_Nrml`, `MSFS.Rate.*`, `MSFS.MainRotor.Speed`,
`MSFS.MainRotor.Torque`, `MSFS.OnGround` all stream and update in the
graph editor's live-value display. Fixed-wing graphs reachable.

**Tier 2 (derivations):** `MSFS.Rotor.BladeAlphPitch` is monotonic with
IAS and within ±50% of the X-Plane reference curve at matched speeds on
matched aircraft class. VRS goes high in hover-descent and drops past
60 kt. Slap rises with speed. (We don't need numeric parity — graph
gains absorb sim-to-sim differences — but the *shape* must match.)

**Tier 3 (in-flight feel):** Fly the MD 500E (X-Plane) and the closest
MSFS counterpart back-to-back with the same `heli_unboosted` graph
and an aircraft-tuned param set. Pedal stiffness scales with RPM in
both. Cyclic feels heavier at speed in both. Vibration intensifies
through ETL in both. Subjective but mandatory.

**Tier 4 (cross-aircraft):** Repeat tier 3 for at least one boosted
aircraft (Bell 206/H125 in MSFS) — verify that the boosted profile's
zero-load assumption still works and only spring/friction/vibration
are felt.

---

## 8. Phasing — testable end-to-end first

| Phase | Scope | Effort | Gates |
| --- | --- | --- | --- |
| **1a — Bridge** | SimConnect EXE in `MsfsPlugin/`, native SimVars + **tier-C `BladeAlph`/VRS/Slap/Propwash derivations** with default constants from §3. UDP packet. SimHub-side `MsfsUdpReceiver` + `MSFS.*` signal registration. | 3-4 days | — |
| **1b — Intermediate graph** | Clone `heli_unboosted.json` → `heli_unboosted_msfs.json` per §3.7, rewire 4 input nodes to `MSFS.*`. | 0.5-1 day | needs 1a streaming |
| **1c — First light** | Launch MSFS heli (Bell 407 or H145), confirm bridge streams, graph evaluates, ESP32 actuates, hardware moves. Subjective "is this plausible?" check. Document gross issues. | 0.5-1 day | needs 1a + 1b |
| **1d (parallel) — MP-caveat probe** | Minimal SimConnect probe per §2.3 to determine whether tier A/B is reachable. Plus unit-quirk verification (§10 Q3). Runs alongside 1a, doesn't block. | 0.5 day | — |
| **1e (parallel) — DataRefLogger additions** | Add the 7 datarefs from §4.1 to `XPlanePlugin/DataRefLogger.cpp`. Build, ship. Runs alongside 1a. | 0.5 day | — |
| **2 — X-Plane calibration** | Fly the §4.2 profile across 3 aircraft, run §4.3 regression notebook, ship `calibrated_constants.md`. Update bridge defaults from `k_speed_pitch = 17` etc. to fitted values. | 3-5 days | needs 1e (logger) + first-light success from 1c |
| **3 — Tier A/B refinements** | If phase 1d cleared the MP caveat, layer disc-tilt and cyclic-command terms onto tier-C `BladeAlph` derivation per §3.1. Re-fit constants. Otherwise skip. | 1-2 days | needs 1d + 2 |
| **4 — `Flight.*` migration** | Single PR sweeping templates + catalogue + plans. Fold `heli_unboosted.json` + `heli_unboosted_msfs.json` back into one template referencing `Flight.*`. | 2 days | needs both sims producing satisfactory feel |

**Critical path to first testable: 1a + 1b + 1c ≈ 4-6 days.** Everything
else is refinement that ships incrementally. The strict gate is "MSFS
heli moves the hardware sensibly" — not "BladeAlph matches X-Plane
ground truth to within ±5%".

Phase 4 should not start until both sims independently produce
satisfactory FFB feel; otherwise we'll bake mismatched assumptions
into the abstraction.

---

## 9. Files Anticipated to Change

**Phase 1 (first-light deliverables):**

* **New:** `MsfsPlugin/` — C++ SimConnect bridge, mirrors `XPlanePlugin/`
* **New:** `SimHubPlugin/graphs/templates/heli_unboosted_msfs.json` (§3.7)
* `SimHubPlugin/DiyFfbPlugin.cs` — add `MsfsUdpReceiver` thread, parser, `latestMsfsPacket`
* `SimHubPlugin/DiyFfbPluginSettings.cs` — `MsfsUdpEnabled`, `MsfsUdpPort`
* `SimHubPlugin/GraphSignals.cs` — `BuildMsfsInputs` (or extend `BuildXPlaneInputs`)
* `SimHubPlugin/GraphSignalCatalogData.cs` — add `MSFS` to `InputGroups`,
  register `MSFS.*` signal names

**Phase 2 (calibration):**

* `XPlanePlugin/DataRefLogger.cpp` — 7 dataref additions per §4.1
* **New:** `docs/calibration/` — flight logs, regression notebook,
  `calibrated_constants.md`

**Phase 4 (long-term):**

* Every `heli_*.json` / `plane_*.json` template, all references in
  plans 04/05/06/08/09 ("XPlane" → "Flight"); fold `heli_unboosted_msfs.json`
  back into `heli_unboosted.json`.

---

## 10. Open Questions

1. **"Multiplayer: far aircraft only" caveat (BLOCKER for tier A/B
   derivations).** All disc-state and blade-pitch SimVars are tagged this
   way in the SDK. Phase 0 must verify empirically whether
   `DISK PITCH ANGLE:0` and `ROTOR CYCLIC BLADE PITCH PCT` are populated
   for the user's own helicopter, or only for network aircraft. If
   user-only-zero, §3.1 collapses to tier C (pure-speed). Plan accepts this
   degradation by design.

2. **Engine torque scaling.** `ENG TORQUE PERCENT:index` is "percent
   scalar 16K" — the SDK convention is 0–16384 representing 0–100% of
   *the aircraft's max torque*. Per-aircraft `max_torque_nm` constant is
   required (see §3.5). Verify sign in flight (collective up → positive
   torque expected).

3. **`ROTATION VELOCITY BODY` unit mismatch.** SDK page lists units as
   "Feet per second" which doesn't match a rotation rate. `ROTATION
   ACCELERATION BODY` is correctly listed as rad/s², so the velocity page
   appears to be an SDK doc bug — the value is almost certainly rad/s.
   Verify with a known maneuver (level 30°/s roll → expect ~0.52 rad/s
   on body-X).

4. **Rotor index on multi-engine helis.** X-Plane code assumes
   `rotorIndex 0` = main rotor. MSFS uses `:0` for main, `:1` for tail
   on most rotor-indexed SimVars but engine indices (`ENG TORQUE PERCENT:1`)
   start at 1 for the first engine. Document the per-aircraft mapping
   in phase 1.

5. **Vertical speed source for VRS.** `VERTICAL SPEED` (instrument,
   laggy) vs `VELOCITY WORLD Y` (true, no lag). Use `VELOCITY WORLD Y`
   for §3.2 — instrument lag would smear the VRS envelope.

6. **MSFS 2024 native blade datarefs.** The SDK Helicopter page as
   fetched marks nothing as "new in 2024". If MSFS 2024 adds per-blade
   variables in a later SU, prefer them over the §3.1 derivations.

7. **Aircraft auto-detect for per-aircraft constants.**
   `rotor_tip_speed_kts`, `disc_area_m2`, `max_torque_nm`, `n_blades`
   are aircraft-specific. Tie to active SimHub vehicle profile (existing
   mechanism), or alternatively read MSFS's `ATC MODEL` / `ATC TYPE`
   SimVars to auto-select a preset. Defer until phase 3.

8. **Co-existence with X-Plane receiver.** Both UDP receivers can run
   simultaneously. For phase 4 `Flight.*` abstraction, add a "last
   updated within Nms" freshness check to pick which sim's signals
   are live.

9. **Helicopter availability in default MSFS 2024 install.** The §4.6
   in-MSFS validation needs a known good helicopter. Confirm a stock
   default heli ships with MSFS 2024 (Bell 407, H145, Cabri G2, or
   AS350 from the 2024 helicopter lineup) — if all defaults are
   payware, fall back to a free community add-on. Affects only timeline,
   not technical approach.
