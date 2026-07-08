# Cyclic Force Feedback Specification — Helicopter FFB System

## Document Purpose

This document specifies the force model, parameters, and graph implementation for a
cyclic (pitch + roll) force feedback system for four helicopter types in X-Plane 12.

The target hardware is an ESP32-based axis controller driving AC servo motors via CAN
bus, with admittance-control (force-in → position-out) architecture. All force computation
happens in the SimHub plugin's graph system; the ESP32 applies the resulting FFB parameters.

All units are SI unless otherwise noted. The ESP32 force model uses mm and N.

---

## 1. Architecture Overview

### 1.1 Signal Flow

```
X-Plane UDP telemetry ──► SimHub Plugin ──► Graph Evaluator ──► FlightFfbAction ──► ESP32
  (IAS, alpha, rates,      (20 Hz)         (per axis)           (protobuf)         (1 kHz force loop)
   rotor torque/RPM,                                                               
   g-load, aero torques)                                                           

Grip buttons (SPI) ──► ESP32 USB HID ──► DirectInput ──► Plugin ButtonInputReader ──► Graph Inputs
  (74HC165 chain)       (100 Hz)          (joystick)      (Grip.* signals)
```

### 1.2 Graph Outputs → ESP32 Force Elements

The graph evaluates per axis and produces these outputs, sent via `FlightFfbAction`:

| Graph Output | Proto Field | ESP32 Element | Unit |
|---|---|---|---|
| `SpringGain` | `k_spring` | `CenteringSpring` | N/mm |
| `DamperGain` | `k_damper` | `Damper` | N·s/mm |
| `Friction` | `k_friction` | `Friction` | N |
| `TrimOffset` | `trim_offset` | `CenteringSpring.offset` | mm |
| `LoadForce` | `load_force` | `LoadForce` | N |
| `BuffetAmplitude` | `buffet_amp` | `Buffet` | N |

The ESP32's `FlightStickFunction` applies these at 1 kHz:
- `centering_spring`: `F = k_spring × (position - (center + trim_offset))`
- `damper`: `F = k_damper × velocity`
- `friction`: constant Coulomb friction
- `load_force`: constant force added to output
- `buffet`: oscillating force
- `simulated_mass`: inertia (from `FunctionConfig`, not graph-driven)

### 1.3 Control System Categories

| Type | Control System | Force Source |
|------|---------------|-------------|
| MD 500E | Unboosted mechanical linkage + electric trim | Aero hinge moments + trim spring |
| Bell 206B-3 | Hydraulically boosted irreversible + FTR + beep trim | Trim spring only |
| H125 (AS350 B3e) | Hydraulically boosted irreversible + FTR + beep trim | Trim spring only |
| Bell 222 | Hydraulically boosted dual + SAS + electric trim | Trim spring + SAS forces |

**Key distinction**: Boosted/irreversible systems (Bell 206, H125, Bell 222) isolate
the pilot from rotor aero loads. The pilot feels ONLY the artificial force-feel system.
The unboosted MD 500E transmits actual rotor hinge moments to the stick.

### 1.4 Available X-Plane Inputs

Graph input signals (from `GraphSignalCatalogData`):

```
XPlane.Speed.IAS              # knots
XPlane.Angle.Alpha            # degrees
XPlane.Angle.Beta             # degrees
XPlane.Trim.Elevator          # normalized
XPlane.Trim.Aileron           # normalized
XPlane.Rate.Roll              # rad/s (P)
XPlane.Rate.Pitch             # rad/s (Q)
XPlane.Rate.Yaw               # rad/s (R)
XPlane.G_Nrml                 # g
XPlane.AeroTorque.Roll        # Nm
XPlane.AeroTorque.Pitch       # Nm
XPlane.AeroTorque.Yaw         # Nm
XPlane.MainRotor.Torque       # Nm
XPlane.MainRotor.Speed        # RPM
XPlane.OnGround               # 0 or 1
```

Grip input signals:

```
Grip.TrimHat.Up / Down / Left / Right   # 0 or 1
Grip.ForceTrimRelease                    # 0 or 1
Grip.TrimReset                           # 0 or 1
```

Axis position signals:

```
Axis.FlightStickPitch.Position   # mm (contact point)
Axis.FlightStickPitch.Center     # mm (midpoint of travel)
Axis.FlightStickRoll.Position    # mm
Axis.FlightStickRoll.Center      # mm
```

---

## 2. Force Model Mapping to Graph Nodes

### 2.1 Common Components (all types)

#### Trim Spring (centering)

Handled by `SpringGain` + `TrimOffset` outputs. The ESP32 computes:
`F = SpringGain × (position - (center + TrimOffset))`

The graph sets `SpringGain` based on the helicopter's spring rate (scaled by flight
condition for unboosted types) and `TrimOffset` from the trim system.

#### Damping

`DamperGain` output → ESP32 `Damper` element: `F = DamperGain × velocity`

Graph can modulate damping based on flight condition (e.g., increase with airspeed
for unboosted types).

#### Friction / Breakout

`Friction` output → ESP32 `Friction` element: constant Coulomb force.

This maps directly to the spec's breakout force. The ESP32 friction model already
provides a constant opposing force around the center — equivalent to the spec's
breakout detent.

#### Load Force (aero / SAS / OWL)

`LoadForce` output → ESP32 `LoadForce` element: constant force added to output.

This is the catch-all for:
- **MD 500E**: aerodynamic hinge moments synthesised from telemetry
- **Bell 222**: SAS-injected forces
- **MD 500E OWL**: one-way lock (load force clamped to one sign in graph)

#### Buffet / Vibration

`BuffetAmplitude` output → ESP32 `Buffet` element: oscillating force.

Graph computes amplitude from alpha/airspeed; ESP32 generates the oscillation.

### 2.2 Trim System Implementation

Both trim strategies are implemented using stateful graph nodes:

#### Type A: Electric Trim (MD 500E, Bell 222)

Uses `trim_hat.json` sub-graph:
- `accumulator` node integrates hat press at `TrimStep` mm/s
- `TrimOffset` output shifts the spring center
- Spring remains active at all times

#### Type B: FTR + Beep Trim (Bell 206, H125)

Uses `trim_combined.json` sub-graph:
- `sample_hold` captures `position - center` on FTR button release
- `accumulator` adds beep trim offset, resets on each FTR release
- `SpringGain` gated to 0 while FTR held (stick free)
- `TrimOffset = FTR_captured + hat_offset`

### 2.3 One-Way Lock (MD 500E, longitudinal only)

The OWL blocks rotor-originated aft cyclic creep. In our architecture this maps to
clamping `LoadForce` to only oppose aft movement:

```
Graph: LoadForce = Min(aero_load, 0)   # only aft-opposing (negative) forces pass
```

Or equivalently: compute the full aerodynamic load force, then clamp it so it can
only push forward (resist aft creep) but never resist forward pilot inputs.

The relief valve (133 N for seized OWL malfunction) would be a max clamp on the
negative load force — not implemented in normal operation.

---

## 3. Per-Helicopter Graph Structure

### 3.1 MD 500E — Unboosted with Aero Forces

**Graph template**: `heli_md500e_cyclic.json`

```
Inputs:  IAS, Alpha, AeroTorque, RotorTorque, RotorRPM, G_Nrml, Rate.Pitch/Roll
         Grip hat, Grip FTR (not used — Type A trim), Axis position/center
Params:  Aircraft.Rotor.TorqueNom, Aircraft.Rotor.SpeedNom, Aircraft.AeroTorque.PitchMax
         Cyclic.SpringGain, Cyclic.DamperGain, Cyclic.FrictionBase
         Cyclic.LoadGain, Cyclic.TrimStep/Min/Max

Processing:
  1. heli_scale → torque_norm, rpm_norm, damp_scale, assist_loss
  2. SpringGain = Cyclic.SpringGain (constant — real spring, not speed-dependent)
  3. DamperGain = damp_scale × Cyclic.DamperGain
  4. Friction = base + torque-scaled + low-RPM contributions
  5. LoadForce = aero_torque / PitchMax × LoadGain × (-1)
     → Clamp to ≤ 0 for OWL (lon axis only; lat axis unclamped)
  6. TrimOffset from trim_hat.json (Type A — accumulator only)
  7. BuffetAmplitude from alpha + qhat (not rotor-specific)

Outputs: SpringGain, DamperGain, Friction, LoadForce, TrimOffset, BuffetAmplitude
```

**Key characteristic**: `LoadForce` is the dominant force at airspeed, computed from
`XPlane.AeroTorque.Pitch/Roll`. This is what makes the MD 500E feel different — the
pilot fights real (synthesised) rotor loads.

#### Parameter Values

| Parameter | Pitch | Roll | Unit | Notes |
|---|---|---|---|---|
| `SpringGain` | 0.175 | 0.175 | N/mm | 175 N/m = 0.175 N/mm |
| `DamperGain` | 0.025 | 0.025 | N·s/mm | 25 N·s/m = 0.025 N·s/mm |
| `Friction` | 2.0 | 2.0 | N | Adjustable 0–4.5 N |
| `LoadGain` | 0.5 | 0.5 | N/Nm_ref | Scale aero torque to force |
| `TrimStep` | 5.0 | 5.0 | mm/s | Electric trim motor speed |
| `simulated_mass` | 0.8 | 0.8 | kg | In FunctionConfig |

### 3.2 Bell 206B-3 — Boosted, FTR + Beep Trim

**Graph template**: uses `heli_cyclic.json` (existing) with `trim_combined.json`

```
Processing:
  1. heli_scale → torque_norm, rpm_norm, damp_scale, assist_loss
  2. SpringGain = Cyclic.SpringGain (gated by FTR via trim_combined)
  3. DamperGain = damp_scale × Cyclic.DamperGain
  4. Friction = base + torque-scaled + low-RPM contributions
  5. LoadForce = 0 (no aero feedback — irreversible)
  6. TrimOffset from trim_combined.json (FTR + beep hat)
  7. BuffetAmplitude = 0 or minimal

Outputs: SpringGain, DamperGain, Friction, LoadForce, TrimOffset
```

**Key characteristic**: No `LoadForce` — the pilot feels only spring + friction.
All force variation comes from the trim system (FTR changes center, spring from
new center).

#### Parameter Values

| Parameter | Value | Unit |
|---|---|---|
| `SpringGain` | 0.325 | N/mm |
| `DamperGain` | 0.030 | N·s/mm |
| `Friction` | 5.0 | N | (includes breakout) |
| `TrimStep` | 5.0 | mm/s | (beep trim rate) |
| `simulated_mass` | 1.0 | kg |

### 3.3 H125 / AS350 B3e — Boosted, FTR + Beep Trim

Same graph structure as Bell 206 with different parameters.

#### Parameter Values

| Parameter | Value | Unit |
|---|---|---|
| `SpringGain` | 0.275 | N/mm |
| `DamperGain` | 0.028 | N·s/mm |
| `Friction` | 4.0 | N |
| `TrimStep` | 5.0 | mm/s |
| `simulated_mass` | 0.9 | kg |

### 3.4 Bell 222 — Boosted with SAS

**Graph template**: `heli_bell222_cyclic.json`

```
Processing:
  1. heli_scale → torque_norm, rpm_norm, damp_scale, assist_loss
  2. SpringGain = Cyclic.SpringGain (with Type A trim — hat only, no FTR)
  3. DamperGain = damp_scale × Cyclic.DamperGain
  4. Friction = base + torque-scaled + low-RPM contributions
  5. LoadForce = SAS forces (when SAS engaged):
     - Pitch: G_Nrml force gradient + pitch rate damping
     - Roll: roll rate damping
     When SAS off: LoadForce = 0
  6. TrimOffset from trim_hat.json (Type A — electric trim only)
  7. BuffetAmplitude = minimal (2-blade 2/rev optional)

Outputs: SpringGain, DamperGain, Friction, LoadForce, TrimOffset
```

**Key characteristic**: `LoadForce` comes from SAS, not aerodynamics. SAS provides
speed stability and manoeuvre cues (force-per-g). With SAS off, reverts to pure
spring model (like a heavier Bell 206 without FTR).

#### Parameter Values

| Parameter | Value | Unit |
|---|---|---|
| `SpringGain` | 0.475 | N/mm |
| `DamperGain` | 0.040 | N·s/mm |
| `Friction` | 6.0 | N |
| `TrimStep` | 4.0 | mm/s |
| `SAS_G_Gain` | 20.0 | N/g |
| `SAS_Q_Gain` | 5.0 | N/(rad/s) |
| `SAS_P_Gain` | 4.0 | N/(rad/s) |
| `simulated_mass` | 1.2 | kg |

---

## 4. Expected Force Levels (Validation Targets)

### MD 500E

| Condition | F_lon (N) | F_lat (N) |
|---|---|---|
| Hover, trimmed | ~0 | ~0 |
| Hover, 10 mm off-trim | 1.5–3 | 1.5–3 |
| 60 kt cruise, trimmed | ~0 | ~0 |
| 80 kt, 60° bank (2g) | 15–25 | — |
| 100 kt, near Vne | 10–20 (fwd) | 5–10 |
| Max normal | 89 | 45 |

### Bell 206 / H125

| Condition | F_lon (N) | F_lat (N) |
|---|---|---|
| Trimmed (any speed) | ~0 | ~0 |
| 10 mm off-trim | 5–8 | 5–8 |
| 50 mm off-trim | 16–22 | 16–22 |
| Full deflection from trim | 30–38 | 30–38 |
| FTR held | ~2 (friction only) | ~2 |
| Max normal | 89 | 45 |

### Bell 222

| Condition | F_lon (N) | F_lat (N) |
|---|---|---|
| Trimmed (any speed) | ~0 | ~0 |
| 10 mm off-trim | 8–12 | 8–12 |
| 50 mm off-trim | 28–35 | 28–35 |
| 30° bank, SAS on | +3 aft (SAS) | — |
| 60° bank, SAS on | +20 aft (SAS) | — |
| Full deflection from trim | 48–55 | 48–55 |
| Max normal | 130 | 65 |

---

## 5. Implementation Checklist

### Already implemented
- [x] Graph evaluator with stateful nodes (accumulator, sample_hold, edge_detect)
- [x] Grip button reading via SPI (74HC165) and DirectInput binding
- [x] Axis position and center as graph inputs
- [x] `trim_hat.json` — Type A hat trim
- [x] `trim_combined.json` — Type B FTR + beep trim
- [x] `heli_cyclic.json` — existing helicopter cyclic template
- [x] `heli_scale.json` — rotor torque/RPM normalisation
- [x] State persistence across vehicle switches and sessions
- [x] Graph state sync to editor live preview

### To implement (graph templates only — no code changes needed)
- [ ] MD 500E cyclic template with aero `LoadForce` computation
- [ ] MD 500E OWL: clamp `LoadForce` to ≤ 0 for lon axis
- [ ] Bell 222 SAS template with G/rate `LoadForce` computation
- [ ] Per-helicopter parameter tuning profiles
- [ ] Optional vibration overlays (2/rev for Bell types)

### Optional future enhancements
- [ ] SAS engage/disengage input signal
- [ ] Density altitude scaling for MD 500E aero forces
- [ ] Rate limiting on force output changes (MAX_FORCE_RATE)

---

## 6. Coefficient Sources and Confidence

| Parameter | Source | Confidence |
|---|---|---|
| Spring rates | ADS-33E tables, FAA cert findings, pilot accounts | Medium |
| Breakout/friction | ADS-33E Table X/XI | Medium-High |
| Force limits | ADS-33E Table XIII | High |
| MD 500E aero coefficients | Synthesised from known flight condition targets | Low-Medium |
| Bell 222 SAS gains | Estimated from handling qualities targets | Low |
| Cyclic travel | Maintenance manual dimensions | Medium-High |
| Trim system types | Flight manuals, pilot accounts | High |
| MD 500E unboosted status | Maintenance manual confirms no hydraulic servo | High |
| MD 500E OWL | Maintenance manual: 133 N relief valve, ~2 mm unseat | High |

### Key references
- ADS-33E-PRF: Handling Qualities Requirements for Military Rotorcraft (2000)
- FAA TCDS H3WE (MD 500 series)
- MD 500E RFM CSP-E-1
- U. Tennessee thesis: "Static Longitudinal Stability of MD 500E" (Kimberlin)
- FAR/CS 27.397: Limit pilot forces
