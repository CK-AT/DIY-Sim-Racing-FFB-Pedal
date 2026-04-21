# Helicopter Load Forces

Load force models for helicopter axes. Uses telemetry signals from plan 04.
All computation happens in the graph system — no ESP32 changes needed.

**Prerequisites:** Plan 04 (X-Plane signal extensions)


---

## 1. Overview

| Heli type | Cyclic load force | Pedal load force |
| --- | --- | --- |
| MD 500E (unboosted) | Aero hinge moments reach pilot | Tail rotor blade loads reach pilot |
| Bell 206 (boosted) | Zero — irreversible | Zero — irreversible |
| H125 (boosted) | Zero — irreversible | Zero — irreversible |
| Bell 222 (boosted + SAS) | SAS-injected forces | Zero — irreversible |


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
Drives collective/pedal coupling but not cyclic forces directly.


---

## 3. Cyclic Load Force

### MD 500E (unboosted) — two-component model

Cyclic load force requires two separate components:

1. **Speed stability** (from `blade_alph_pitch/roll`): stick gets heavier
   with airspeed. Physically correct — advancing blade asymmetry increases
   hinge moments at higher IAS. X-Plane's per-axis decomposition means
   `blade_alph_pitch` maps directly to longitudinal load force and
   `blade_alph_roll` to lateral — no axis swap needed.

2. **Manoeuvre stability** (from `g_nrml`): stick gets heavier when pulling
   g. Must be added as a separate term since blade alpha doesn't capture
   this effect (see plan 04 section 3 g-load analysis).

```
LoadForce_pitch = speed_gain * abs(blade_alph_pitch)   # lon: dominant, aft force
                + g_gain * (g_nrml - 1.0)               # manoeuvre cue

LoadForce_roll  = speed_gain * abs(blade_alph_roll)    # lat: weak
                + g_gain * (g_nrml - 1.0)               # manoeuvre cue
```

For OWL (one-way lock, longitudinal axis only): clamp pitch `LoadForce` to
negative values only (resist aft creep, don't resist forward input):

```
LoadForce_pitch = Min(0, LoadForce_pitch)
```

Graph params:

```
Cyclic.LoadSpeedGain    — N per degree of blade alpha
Cyclic.LoadGGain        — N per g increment above 1.0
```

### Bell 222 (SAS) — same two-component pattern

The SAS model uses `g_nrml` and body rates. The data analysis confirms
this is the right approach — `blade_alph_pitch` would not provide the
manoeuvre cue that the SAS is specifically designed to inject.

```
LoadForce_pitch = sas_g_gain * (g_nrml - 1.0)     # manoeuvre cue
                + sas_q_gain * Q_rad_s              # pitch rate damping

LoadForce_roll  = sas_p_gain * P_rad_s              # roll rate damping
```

When SAS is disengaged: `LoadForce = 0` (reverts to pure spring model).
SAS engage/disengage would need a new input signal (future work).

### Bell 206, H125 (boosted, irreversible)

`LoadForce = 0`. Pilot feels only spring + friction + trim system.


---

## 4. Pedal Load Force

### RPM-scaled spring (all types)

Rather than computing a tail rotor hinge moment (no good dataref), scale
the pedal `SpringGain` by normalized rotor RPM:

```
SpringGain = base_spring * rpm_norm
```

Physical rationale: tail rotor blade loads scale with RPM (more RPM = more
aerodynamic force per degree of pitch = stiffer pedals). This captures:

* **Normal flight**: full RPM → firm pedals
* **Autorotation**: RPM drops → pedals lighten (less TR authority)
* **Shutdown**: RPM → 0 → pedals free
* **Engine failure**: same as autorotation — immediate lightening

Works for both boosted and unboosted types — even hydraulically boosted
systems have an artificial spring that should feel lighter when the tail
rotor has less authority.

### MD 500E pedal load force (optional refinement)

For enhanced realism on the unboosted MD 500E, add a small `LoadForce`
component proportional to main rotor torque (more torque = more anti-torque
pedal demand = more force to hold position):

```
LoadForce_pedal = pedal_load_gain * torque_norm
```

This is a secondary effect on top of the RPM-scaled spring. Defer to tuning
phase — the RPM-scaled spring alone may be sufficient.


---

## 5. Collective Load Force

The collective on most helicopters has a friction lock — the pilot adjusts
friction to hold the collective in position. Model as constant friction
with no speed-dependent load. For the unboosted MD 500E, collective
forces scale with RPM and blade pitch, but the dominant feel is the
friction lock. `SpringGain = 0`, `Friction = user-adjustable`.

RPM-scaled friction is a possible refinement:

```
Friction = base_friction * rpm_norm
```


---

## 6. Implementation

1. Add cyclic load force computation to `heli_cyclic.json` sub-graph
2. Wire `LoadForce` outputs per axis in heli template
3. Add pedal RPM-scaled spring to `heli_pedal.json` sub-graph
4. Test MD 500E — verify speed stability cue builds with IAS
5. Test MD 500E — verify g-load cue in turns
6. Tune default params


---

## 7. Open Questions

1. **Collective coupling**: collective-axis vibration has physically distinct
   amplitudes (vertical blade-passing dominates). Defer to later or include
   in initial implementation?
