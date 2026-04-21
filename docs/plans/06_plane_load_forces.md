# Fixed-Wing Load Forces

Force model for fixed-wing aircraft `LoadForce` outputs. All computation
happens in the graph system using existing telemetry signals — no new
datarefs, no ESP32 changes.


---

## 1. Motivation

The current plane graphs (`plane_pitch.json`, `plane_roll.json`,
`plane_yaw.json`) produce `SpringGain`, `DamperGain`, `Friction`, and
`BuffetAmplitude`. `LoadForce` is wired but driven by whole-airframe
aerodynamic moments (`XPlane.AeroTorque.Pitch/Roll/Yaw` = `L_aero`,
`M_aero`, `N_aero`), normalized by a per-aircraft max and scaled by
`LoadGain`. However, these are NOT control hinge moments — they
represent total aerodynamic torque about the CG, dominated by
wing/tail/rotor disc forces. The relationship to stick force varies
with control geometry per aircraft, and the data analysis in plan 04
section 3 shows that the underlying datarefs don't track manoeuvre
loads correctly (e.g., `blade_alph_pitch` decreases when pulling g).

This plan replaces the aero-torque approach with a force model based
on flight state signals (g-load, body rates, sideslip) that map more
directly to what a pilot feels through the controls.

Real aircraft have control surface hinge moments that the pilot feels
through the stick/yoke. The dominant effects:

* **Pitch**: stick force per g (manoeuvre stability) — the most important
  feel cue. FAR 23/25 require a minimum force gradient.
* **Roll**: ailerons get heavier at speed (hinge moment scales with q).
* **Yaw**: rudder hinge moment scales with q; sideslip creates a pedal
  force cue ("ball not centered").


---

## 2. Design Constraints

### No stick-position feedback

Using `yoke_pitch * qhat` as a load force creates a feedback loop: stick
position depends on total force, which includes load force, which depends
on stick position. This can cause oscillation or instability in the
admittance-control force loop.

**Rule**: load force must depend only on **flight state signals** (rates,
g-load, angles, airspeed), never on stick position.

### Separation from spring and damper

* `SpringGain` provides speed-dependent centering (already qhat-scaled).
* `DamperGain` provides rate-proportional resistance.
* `LoadForce` adds a **constant offset** based on flight state — it's the
  force you must hold against, independent of displacement from trim.

The key distinction: spring force is zero at trim center, load force is
NOT. A 2g pull requires holding back against `LoadForce` even when the
stick is at the trimmed position.


---

## 3. Available Signals

All already in the X-Plane UDP packet and exposed as graph inputs:

| Signal | Unit | Pitch | Roll | Yaw |
| --- | --- | --- | --- | --- |
| `XPlane.G_Nrml` | g | Primary driver | — | — |
| `XPlane.Rate.Pitch` (Q) | rad/s | Rate damping | — | — |
| `XPlane.Rate.Roll` (P) | rad/s | — | Rate resistance | — |
| `XPlane.Rate.Yaw` (R) | rad/s | — | — | Rate resistance |
| `XPlane.Angle.Beta` | deg | — | — | Sideslip force cue |
| `XPlane.Angle.Alpha` | deg | Stall warning | — | — |
| `XPlane.Speed.IAS` | kts | Via qhat_eff | Via qhat_eff | Via qhat_eff |


---

## 4. Per-Axis Load Force Model

### 4.1 Pitch — manoeuvre stability + rate damping

The pitch axis `LoadForce` provides two cues:

1. **G-load force gradient**: pulling g requires proportionally more force.
   This is the primary "feel" difference between straight flight and turns.

2. **Pitch rate opposition**: resists uncommanded pitch rates. Supplements
   the `DamperGain` with a constant force component (damper is velocity-
   proportional, this adds a step force when pitching).

```
LoadForce_pitch = g_gain * (g_nrml - 1.0)
                + q_gain * Q_rad_s
```

**Rate-term interaction with DamperGain:** The `q_gain * Q_rad_s` term
is functionally a damper (opposes pitch rate) but applied as a constant
force offset rather than velocity-proportional resistance. At high
`LoadRateGain` values this can interact with the existing `DamperGain`
and cause oscillation — the load force opposes the rate, the spring
pulls the stick back, generating a new rate, which generates more load
force. Start with conservative defaults and increase only if the damper
alone doesn't provide sufficient pitch rate cue. The same interaction
applies to the roll and yaw rate terms below.

| Param | Default | Range | Unit | Description |
| --- | --- | --- | --- | --- |
| `FlightStickPitch.LoadGGain` | 5.0 | 0 - 30 | N/g | Force per g increment |
| `FlightStickPitch.LoadRateGain` | 0.0 | 0 - 10 | N/(rad/s) | Force per pitch rate |

**Expected feel:**

| Manoeuvre | g_nrml | LoadForce |
| --- | --- | --- |
| Straight and level | 1.0 | 0 N |
| 30 deg bank (1.15g) | 1.15 | 0.75 N |
| 45 deg bank (1.41g) | 1.41 | 2.1 N |
| 60 deg bank (2.0g) | 2.0 | 5.0 N |
| 2.5g pull-up | 2.5 | 7.5 N |
| Pushover (0.5g) | 0.5 | -2.5 N (forward push) |

The sign convention: positive `LoadForce` pushes the stick forward (the
pilot must pull back to hold position in a turn). This matches the real
aircraft: tail downforce increases with g, requiring aft stick force.

### 4.2 Roll — rate resistance

Roll `LoadForce` opposes sustained roll rates. This supplements the damper
with a feel cue that the pilot must actively "push through" to maintain a
roll. Provides resistance proportional to roll rate, scaled by dynamic
pressure (heavier ailerons at higher speed).

```
LoadForce_roll = p_gain * P_rad_s * qhat
```

| Param | Default | Range | Unit | Description |
| --- | --- | --- | --- | --- |
| `FlightStickRoll.LoadRateGain` | 0.0 | 0 - 5 | N/(rad/s) | Force per roll rate at Vref |

At low speed (`qhat` < 1), roll forces are light. At high speed, they
increase — matching the heavier aileron feel of real aircraft.

Note: for most GA aircraft, the spring + damper already provide adequate
roll feel. `LoadForce` on roll is a refinement, not essential. Start with
zero and tune up if the roll axis feels too "free" in manoeuvres.

### 4.3 Yaw — sideslip cue + rate resistance

Pedal `LoadForce` provides two cues:

1. **Sideslip force**: rudder force proportional to beta (sideslip angle).
   When the ball is not centered, the pilot feels a pedal force pushing
   toward coordinated flight. This is the most useful pedal force cue.

2. **Yaw rate resistance**: opposes sustained yaw rates.

```
LoadForce_yaw = beta_gain * beta_deg
              + r_gain * R_rad_s
```

Sign convention: positive `LoadForce` pushes the pedal axis in the
positive direction (right pedal forward / left pedal aft, matching the
existing `ConstForce` convention on `FlightPedalsFunction`). Positive
beta (nose right) produces positive load force — the pilot must push
left rudder to return to coordinated flight, which matches real aircraft
feel (sideslip creates a restoring pedal force).

| Param | Default | Range | Unit | Description |
| --- | --- | --- | --- | --- |
| `FlightPedals.LoadBetaGain` | 0.3 | 0 - 2 | N/deg | Force per degree sideslip |
| `FlightPedals.LoadRateGain` | 0.0 | 0 - 5 | N/(rad/s) | Force per yaw rate |

**Expected feel:**

| Condition | Beta | LoadForce |
| --- | --- | --- |
| Coordinated flight | 0 deg | 0 N |
| Slight slip (3 deg) | 3 deg | 0.9 N |
| Significant slip (10 deg) | 10 deg | 3.0 N |
| Engine-out (15+ deg) | 15 deg | 4.5 N |


---

## 5. Graph Implementation

### New sub-graph: `plane_load.json`

Optional Include block for fixed-wing load forces. Keeps the existing
plane_pitch/roll/yaw sub-graphs unchanged — load force is wired at the
template level.

Inputs:

* `g_nrml` (from `XPlane.G_Nrml`)
* `q_rate` (from `XPlane.Rate.Pitch`)
* `p_rate` (from `XPlane.Rate.Roll`)
* `r_rate` (from `XPlane.Rate.Yaw`)
* `beta` (from `XPlane.Angle.Beta`)
* `qhat` (from `qhat_eff` node in parent graph)

Outputs:

* `load_pitch` → wired to `FlightStickPitch.LoadForce` output
* `load_roll` → wired to `FlightStickRoll.LoadForce` output
* `load_yaw` → wired to `FlightPedals.LoadForce` output

All params as inputs (not embedded Param nodes) so the parent graph
controls naming per function group.

### Wiring in `plane_default.json`

```
                    g_nrml ──┐
XPlane inputs ──►   q_rate  ─┤
                    p_rate  ─┼──► plane_load ──► LoadForce outputs
                    r_rate  ─┤
                    beta   ──┤
qhat_eff node ──►   qhat   ──┘
```

The load force outputs are added to the existing Output nodes alongside
SpringGain, DamperGain, Friction, BuffetAmplitude.

### Alternative: inline in axis sub-graphs

Instead of a separate sub-graph, each axis sub-graph (plane_pitch.json
etc.) could compute its own load force internally. Simpler wiring but
less reusable. Either approach works — graph-level choice.


---

## 6. Per-Aircraft Tuning

### Light GA (Cessna 172, PA-28)

Light, cable-controlled aircraft with relatively direct control feel.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 3.0 N/g | Light gradient, low max forces |
| Pitch LoadRateGain | 0.5 N/(rad/s) | Mild pitch rate cue |
| Roll LoadRateGain | 0.0 | Not needed — spring/damper sufficient |
| Yaw LoadBetaGain | 0.2 N/deg | Light rudder forces |

### Heavy GA (Bonanza, Baron, TBM)

Heavier controls, more pronounced force gradients.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 8.0 N/g | Firmer gradient |
| Pitch LoadRateGain | 1.5 N/(rad/s) | More authority feel |
| Roll LoadRateGain | 0.3 N/(rad/s) | Slight aileron heaviness |
| Yaw LoadBetaGain | 0.5 N/deg | Heavier rudder |

### Turboprop / Light jet

Higher speeds, boosted or mass-balanced controls.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 12.0 N/g | Strong gradient for higher-g capability |
| Pitch LoadRateGain | 2.0 N/(rad/s) | Crisp pitch response |
| Roll LoadRateGain | 0.5 N/(rad/s) | Roll stiffens with speed via qhat |
| Yaw LoadBetaGain | 0.8 N/deg | Firm rudder |

### Warbird / aerobatic

High g-capability, direct controls, need clear manoeuvre cues.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 15.0 N/g | Very clear g cue for aerobatics |
| Pitch LoadRateGain | 2.0 N/(rad/s) | Crisp |
| Roll LoadRateGain | 1.0 N/(rad/s) | Aileron authority feel |
| Yaw LoadBetaGain | 0.5 N/deg | |

All values are starting points — tune by feel in-sim.


---

## 7. Implementation

1. Create `plane_load.json` sub-graph (Mul, Sub, Add nodes — no new Funcs)
2. Wire `LoadForce` outputs in `plane_default.json`
3. Test with Cessna 172 — verify g-load cue in turns
4. Test with Baron 58 — verify pedal beta cue
5. Tune default params for GA categories


---

## 8. Open Questions

1. **Alpha cue**: should pitch `LoadForce` include an alpha component
   for stick-pusher feel at high AoA? Could add
   `alpha_gain * max(0, alpha - alpha_threshold)` as a stall warning
   force. Already have `BuffetAmplitude` for stall — may be redundant.

2. **Asymmetric engine-out**: single-engine aircraft have a strong yaw
   moment on engine failure. `N_aero` or `beta` would capture this, but
   the magnitude needs tuning. Worth it for training realism?

3. **Trim change with config**: flaps, gear, and power changes create
   trim offsets that the pilot feels as transient stick forces. Currently
   our trim system is manual (hat switch). Adding a `XPlane.Trim.Elevator`
   feedback to the graph could auto-trim, but that's a different feature.

4. **Qhat scaling on pitch g-cue**: should `LoadGGain` scale with qhat?
   In a real aircraft, the elevator is more effective at high speed, so
   less stick displacement (but not less force) is needed per g. The
   g-load cue is arguably independent of speed since it represents the
   structural load, not the control effectiveness. Start without qhat
   scaling and evaluate.
