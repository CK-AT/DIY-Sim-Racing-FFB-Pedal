# Helicopter Load Forces

Load force models for helicopter axes. Uses telemetry signals from plan 04.
All computation happens in the graph system — no ESP32 changes needed.

**Prerequisites:** Plan 04 (X-Plane signal extensions)


---

## 1. Overview

| Heli type | Cyclic | Pedals | Collective |
| --- | --- | --- | --- |
| MD 500E (unboosted) | LoadForce from blade alpha | RPM-scaled spring (new) | LoadForce from torque (existing) |
| Bell 206 (boosted) | Zero — irreversible | RPM-scaled spring (new) | Zero — friction lock only |
| H125 (boosted) | Zero — irreversible | RPM-scaled spring (new) | Zero — friction lock only |
| Bell 222 (boosted + SAS) | SAS-injected LoadForce | RPM-scaled spring (new) | Zero — friction lock only |


---

## 2. Available Datarefs vs Requirements

**`L_aero`, `M_aero`, `N_aero`** (whole-airframe aerodynamic moments, Nm):
NOT control hinge moments. They represent total aerodynamic torque about the
aircraft CG — dominated by wing/tail/rotor disc forces. Observed range:
M_aero = -8600 to +6764 Nm on MD 500E. Far too large and not directly
proportional to stick force. Using them requires extreme scaling (~0.005
N/Nm) and the relationship to stick force varies with control geometry.

**`blade_alph_pitch` / `blade_alph_roll`** (per-axis blade alpha, degrees):
Better proxy for hinge moments on unboosted rotors. X-Plane has already
decomposed blade AoA into elevator (longitudinal) and aileron (lateral)
cyclic axes — no precession mapping needed. Scales linearly with IAS
(-0.2 hover to -6.2 at 170 kt). Also used for 1/rev vibration amplitude
on the corresponding axis.

See plan 04 section 3 for the blade_alph_pitch vs g-load analysis showing
it is primarily an airspeed signal, not a g-load signal.

**`torque_main`** (main rotor torque, Nm):
Used for collective `LoadForce` on unboosted types (existing).
Not used for cyclic or pedal forces.


---

## 3. Cyclic Load Force

### MD 500E (unboosted) — speed-dependent model

Unboosted helicopter cyclic force comes from blade flapping hinge moments,
which are about *asymmetry* between advancing and retreating blades. This
asymmetry scales with airspeed, not with g-load. There is no natural
"stick force per g" on helicopter cyclic — the pilot feels g through their
body, not through the stick.

The data confirms this: `blade_alph_pitch` actually *decreases* when
pulling g at cruise speeds (see plan 04 section 3). Physically, pulling
back tilts the disc aft, reducing the advancing blade's encounter angle.

`blade_alph_pitch` is always negative in forward flight (-0.2 hover →
-6.2 at 170 kt). Using `gain * (-alpha)` preserves the physical sign
relationship: negative alpha → positive aft stick force.

```text
LoadForce_pitch = speed_gain * (-blade_alph_pitch)  # lon: dominant, aft force
LoadForce_roll  = speed_gain * (-blade_alph_roll)   # lat: weak
```

For OWL (one-way lock, longitudinal axis only): clamp pitch `LoadForce` to
negative values only (resist aft creep, don't resist forward input):

```text
LoadForce_pitch = Min(0, LoadForce_pitch)
```

Graph params:

```text
Cyclic.LoadSpeedGain    — N per degree of blade alpha
```

### Bell 222 (SAS) — artificially injected g-cue

The SAS *artificially injects* a g-load cue that doesn't exist naturally
on unboosted cyclic. This is the correct model for SAS-equipped aircraft
— the SAS system is specifically designed to provide manoeuvre stability.

```text
LoadForce_pitch = sas_g_gain * (g_nrml - 1.0)     # manoeuvre cue
                + sas_q_gain * Q_rad_s              # pitch rate damping

LoadForce_roll  = sas_p_gain * P_rad_s              # roll rate damping
```

When SAS is disengaged: `LoadForce = 0` (reverts to pure spring model).
SAS engage/disengage would need a new input signal (future work).

### Bell 206, H125 (boosted, irreversible)

`LoadForce = 0`. Pilot feels only spring + friction + trim system.


---

## 4. Pedal Spring Scaling

No `LoadForce` term needed for pedals — torque changes cause the pilot
to *reposition* the pedals to maintain yaw equilibrium, and the spring
force at the new position is what they feel.

However, **RPM-scaled spring stiffness** is new and important:

```text
SpringGain = base_spring * rpm_norm
```

Physical rationale: tail rotor blade loads scale with RPM (more RPM =
more aerodynamic force per degree of pitch = stiffer pedals). This
captures:

* **Normal flight**: full RPM → firm pedals
* **Overtorque / RPM droop**: RPM drops → pedals lighten
* **Shutdown**: RPM → 0 → pedals free

Works for both boosted and unboosted types.

`assist_loss` friction (hydraulic fade at low RPM) is already implemented
in the existing graphs.


---

## 5. Collective Load Force

On unboosted helicopters, the pilot holds the collective against
aerodynamic blade pitch loads through the pitch links. More collective
pitch = more torque = more force trying to push the lever down.

A torque-proportional `LoadForce` is already implemented in the existing
collective graph:

```text
LoadForce_collective = load_gain * torque_norm
```

This gives the "heavier when pulling power" feel. On boosted types
(Bell 206, H125), `LoadForce = 0` — the hydraulics absorb blade loads
and the pilot feels only the friction lock.

`assist_loss` friction (hydraulic fade at low RPM) is also already
implemented. No new collective work needed.


---

## 6. Implementation

1. Add cyclic load force computation to `heli_cyclic.json` sub-graph
2. Wire `LoadForce` outputs per axis in heli template
3. Add pedal RPM-scaled spring to `heli_pedal.json` sub-graph
4. Test MD 500E — verify speed stability cue builds with IAS
5. Tune default params


---

## 7. Open Questions

(None remaining — collective vibration moved to plan 08.)
