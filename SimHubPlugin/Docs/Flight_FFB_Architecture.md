# Flight FFB Architecture

This document describes the force feedback architecture for flight sim axes
(stick pitch, roll, pedals, collective). It is intended to provide enough
information for contributors to design FFB graphs and strategies for specific
aircraft without needing to read the full codebase.

---

## 1. System Overview

```
                        SimHub Plugin                              ESP32
                    (graph evaluation)                         (force loop)

 X-Plane UDP ──► telemetry signals ──► Graph ──► FlightFfbAction ──► FlightStickFunction
   (20 Hz)       (XPlane.*)            Evaluator   (protobuf)         (1 kHz)
                                         |
 Grip buttons ──► DirectInput ──► Grip.* inputs     per axis:         Force elements:
   (SPI 100Hz)    (binding UI)                      - SpringGain       ├ CenteringSpring
                                                    - DamperGain       ├ Damper
 Axis position ──► AxisState ──► Axis.* inputs      - Friction         ├ Friction
   (from ESP32)    (protobuf)                       - TrimOffset       ├ Buffet
                                                    - LoadForce        └ ConstForce
                                                    - BuffetAmplitude
```

The plugin evaluates a graph once per data update cycle (~50 ms) for each active
flight function. The graph reads telemetry and grip inputs and produces six FFB
output values. These are sent to the ESP32 as a `FlightFfbAction` protobuf message
over USB serial. The ESP32 applies them in a 1 kHz admittance-control force loop.

---

## 2. Graph Outputs

Each flight function (FlightStickPitch, FlightStickRoll, FlightPedals,
FlightStickCollective) produces these outputs:

| Output | Proto field | ESP32 element | Unit | Description |
| --- | --- | --- | --- | --- |
| `SpringGain` | `k_spring` | CenteringSpring | N/mm | Spring constant around trim center |
| `DamperGain` | `k_damper` | Damper | N-s/mm | Velocity-proportional resistance |
| `Friction` | `k_friction` | Friction | N | Constant Coulomb friction |
| `TrimOffset` | `trim_offset` | CenteringSpring offset | mm | Shifts spring center from baseline |
| `LoadForce` | `load_force` | ConstForce | N | Constant force (aero loads, SAS, etc.) |
| `BuffetAmplitude` | `buffet_amp` | Buffet | N | Amplitude of band-limited random force |
| `VibSlot1..5` | `vib_amp_slot1..5` | SyncVib (DDS 1) | N | Coherent vibration amplitude per harmonic slot |
| `Vib2Slot1..2` | `vib2_amp_slot1..2` | SyncVib (DDS 2) | N | Secondary oscillator amplitudes (engine, tail rotor) |

Output signal names use the function as prefix:
`FlightStickPitch.SpringGain`, `FlightStickRoll.TrimOffset`, etc.

DDS fundamentals are global, not per-function. They use a separate
`Shared.*` scope and are sent in their own `DdsFundamentals` message:

| Output | Wire path | Unit | Description |
| --- | --- | --- | --- |
| `Shared.VibFundamental` | `DdsFundamentals.dds1_fundamental_hz` → CAN `0x0F0` | Hz | DDS 1 master fundamental |
| `Shared.Vib2Fundamental` | `DdsFundamentals.dds2_fundamental_hz` → CAN `0x0F0` | Hz | DDS 2 master fundamental |

Wire format detail: amplitude fields are quantized to 8 bits at 0.01 N/LSB
(0..2.55 N range). The plugin pre-scales (×100) before sending; the ESP32
multiplies by 0.01 when applying to `SyncVib::set_amplitudes`.

The `FlightStickConfig.phase_offset` ConfigOut field is expressed in
**degrees** at the override / graph layer (more author-friendly) and
converted to radians by `FlightStickProcessor.ApplyOverrides` before the
proto is sent. The wire and firmware stay in radians.

### How the ESP32 applies them

The force loop runs at 1 kHz. Each cycle:

1. **CenteringSpring**: `F = -k_spring * (position - (center + trim_offset))`
   - `center` = `(pos_min + pos_max) / 2` from FlightStickConfig
   - `trim_offset` shifts the equilibrium point (positive = forward/right)
2. **Damper**: `F = -k_damper * velocity`
   - Takes the max of the graph value and the baseline config damping
3. **Friction**: Coulomb model with static/kinetic distinction
   - Opposes motion with a constant force
   - Acts as breakout force around the trim center
4. **ConstForce**: `F = load_force`
   - Constant force added to output; graph computes sign and magnitude
   - Positive = toward pos_max; negative = toward pos_min
5. **Buffet**: Band-limited noise (two-pole filter, 5-25 Hz band)
   - `F = filtered_noise * buffet_amp`
6. **Simulated mass**: `m_eff` from FunctionConfig (not graph-driven)
   - Provides inertia: `a = F_total / m_eff`
   - Position update via Verlet integration

If no `FlightFfbAction` arrives within 200 ms, the ESP32 reverts to baseline
config values (from `FlightStickConfig`).

---

## 3. Graph Inputs

### Telemetry (from X-Plane UDP)

| Signal | Unit | Description |
| --- | --- | --- |
| `XPlane.Speed.IAS` | knots | Indicated airspeed |
| `XPlane.Angle.Alpha` | degrees | Angle of attack |
| `XPlane.Angle.Beta` | degrees | Sideslip angle |
| `XPlane.Trim.Elevator` | normalized | Sim trim state (elevator) |
| `XPlane.Trim.Aileron` | normalized | Sim trim state (aileron) |
| `XPlane.Trim.Rudder` | normalized | Sim trim state (rudder) |
| `XPlane.Rate.Roll` | rad/s | Roll rate (P) |
| `XPlane.Rate.Pitch` | rad/s | Pitch rate (Q) |
| `XPlane.Rate.Yaw` | rad/s | Yaw rate (R) |
| `XPlane.G_Nrml` | g | Normal load factor |
| `XPlane.AeroTorque.Roll` | Nm | Aerodynamic torque, roll axis |
| `XPlane.AeroTorque.Pitch` | Nm | Aerodynamic torque, pitch axis |
| `XPlane.AeroTorque.Yaw` | Nm | Aerodynamic torque, yaw axis |
| `XPlane.MainRotor.Torque` | Nm | Main rotor torque |
| `XPlane.MainRotor.Speed` | RPM | Main rotor RPM |
| `XPlane.OnGround` | 0/1 | Weight on wheels |

### Grip buttons (from DirectInput, user-bound)

| Signal | Value | Description |
| --- | --- | --- |
| `Grip.TrimHat.Up` | 0 or 1 | Hat switch up |
| `Grip.TrimHat.Down` | 0 or 1 | Hat switch down |
| `Grip.TrimHat.Left` | 0 or 1 | Hat switch left |
| `Grip.TrimHat.Right` | 0 or 1 | Hat switch right |
| `Grip.ForceTrimRelease` | 0 or 1 | FTR button held |
| `Grip.TrimReset` | 0 or 1 | Trim reset button |

Bindings are configured in the plugin UI (System > Input tab). Any DirectInput
joystick button, hat direction, or keyboard key can be mapped to each signal.

### Axis position (from ESP32 AxisState)

| Signal | Unit | Description |
| --- | --- | --- |
| `Axis.FlightStickPitch.Position` | mm | Current contact point position |
| `Axis.FlightStickPitch.Center` | mm | `(pos_min + pos_max) / 2` |
| `Axis.FlightStickRoll.Position` | mm | Same for roll axis |
| `Axis.FlightStickRoll.Center` | mm | |
| `Axis.FlightPedals.Position` | mm | Same for pedals |
| `Axis.FlightPedals.Center` | mm | |
| `Axis.FlightStickCollective.Position` | mm | Same for collective |
| `Axis.FlightStickCollective.Center` | mm | |

Position is resolved via function -> linked axis -> cached AxisState. Center is
computed from the function's FlightStickConfig pos_min/pos_max.

---

## 4. Graph Node Types

### Arithmetic operations (Op nodes)

| Op | Ports | Computation |
| --- | --- | --- |
| `add` | `a, b [, c, ...]` | Sum of all inputs. Per-input `Negate` flag inverts before adding. |
| `sub` | `a, b` | `a - b` |
| `mul` | `a, b [, c, ...]` | Product of all inputs. Per-input `Negate` flag inverts. |
| `div` | `a, b` | `a / b` (0 if b ~ 0) |
| `min` | `a, b [, ...]` | Minimum of all inputs |
| `max` | `a, b [, ...]` | Maximum of all inputs |
| `abs` | `a` | `|a|` |
| `neg` | `a` | `-a` |
| `clamp` | `a, min, max` | Clamp a to [min, max] |
| `lerp` | `a, b, t` | `a + (b - a) * t` |

### Built-in functions (Func nodes)

#### Stateless

| Function | Inputs | Output | Description |
| --- | --- | --- | --- |
| `qhat_eff` | `ias_kts, vref_kts` | normalized q | `(IAS/Vref)^2` — dynamic pressure ratio. Use to scale forces with airspeed. |
| `torque_norm` | `trq, trq_ref` | ratio | `trq / trq_ref` — normalize torque to reference. |
| `rpm_norm` | `rpm, rpm_ref` | ratio | `rpm / rpm_ref` — normalize RPM to reference. |
| `assist_loss` | `rpm_norm` | 0..1 | `1 - rpm_norm` clamped — hydraulic assist loss factor. Returns 1.0 at zero RPM. |
| `buffet` | `alpha, start, full, gain, qhat_eff` | amplitude | Ramps linearly from 0 at `start` degrees to `gain * qhat_eff` at `full` degrees. |

#### Stateful (persist across evaluations)

| Function | Inputs | Output | Description |
| --- | --- | --- | --- |
| `accumulator` | `trigger, step, min, max [, reset]` | value | While `trigger > 0.5`, accumulates `step * dt` per second. `reset > 0.5` zeros the value. Clamped to [min, max]. Used for hat trim. |
| `sample_hold` | `input, trigger` | held value | Captures `input` on the **falling edge** of `trigger` (1->0). Holds last captured value. Used for FTR position capture. |
| `edge_detect` | `input` | 0 or 1 | Outputs 1.0 for one evaluation tick on the **rising edge** of input (0->1). Used to reset accumulators on button press. |

The `dt` (seconds since last evaluation) is tracked automatically and passed to
all stateful functions, including those inside Include sub-graphs.

### Other node types

| Type | Purpose |
| --- | --- |
| **Input** | Reads a telemetry/grip/axis signal. Has a SignalGroup (e.g., `XPlane`) and SignalSuffix (e.g., `Speed.IAS`). |
| **Output** | Writes a graph output. Has a SignalGroup (e.g., `FlightStickPitch`) and SignalSuffix (e.g., `SpringGain`). |
| **Param** | User-tunable parameter with default, min, max, and UI widget config. Grouped for the vehicle tab UI. |
| **Const** | Literal numeric constant. |
| **Include** | Embeds another graph file as a sub-graph. Has InputMap/OutputMap to wire ports. |

---

## 5. Include Sub-Graphs (Library Blocks)

Graphs can reference reusable sub-graphs via Include nodes. The sub-graph is
evaluated with the parent's wired inputs and parameters; its outputs are mapped
back to the parent.

### Available library blocks

| File | Title | Inputs | Outputs | Purpose |
| --- | --- | --- | --- | --- |
| `trim_hat.json` | Hat Trim | hat_pos, hat_neg, reset, trim_step, trim_min, trim_max | TrimOffset | Accumulator-based incremental trim via hat switch |
| `trim_ftr.json` | Force Trim Release | ftr_button, axis_position, axis_center, spring_in | TrimOffset, SpringGain | FTR: zeros spring while held, captures position-center on release |
| `trim_combined.json` | FTR + Hat Trim | ftr_button, axis_position, axis_center, hat_pos, hat_neg, spring_in, trim_step, trim_min, trim_max | TrimOffset, SpringGain | Combined: FTR sets baseline, hat adds incremental offset. Hat resets on each FTR release. |
| `heli_scale.json` | Heli Scale | in_torque, in_rpm, in_torque_ref, in_rpm_ref, in_rpm_blend | out_torque_norm, out_damp_scale, out_assist_loss | Normalizes rotor torque/RPM for helicopter force scaling |
| `heli_cyclic.json` | Heli Cyclic | torque, rpm, aero_trq, grip (hat_pos, hat_neg, ftr_button), axis_position, axis_center | spring, damper, friction, load, trim | Complete helicopter cyclic axis with FTR+hat trim |
| `heli_collective.json` | Heli Collective | torque, rpm | damper, friction, load | Helicopter collective with torque-based damping |
| `heli_pedals.json` | Heli Pedals | torque, rpm, aero_trq | spring, damper, friction, load | Helicopter anti-torque pedals |
| `plane_pitch.json` | Plane Pitch | ias, alpha, aero_trq | spring, damper, friction, load, buffet | Fixed-wing pitch axis with qhat scaling and stall buffet |
| `plane_roll.json` | Plane Roll | ias, alpha, aero_trq | spring, damper, friction, load, buffet | Fixed-wing roll axis |
| `plane_yaw.json` | Plane Yaw | ias, alpha, aero_trq | spring, damper, friction, load, buffet | Fixed-wing yaw/rudder axis |

### Creating a new library block

1. Create a `.json` file in `graphs/_embedded/`
2. Set `"IsLibraryGraph": true` in the JSON
3. Define Input nodes for signals the parent must provide
4. Define Output nodes for values the parent will use
5. Optionally define Param nodes — these surface in the vehicle UI
6. Reference from parent graphs via Include nodes with InputMap/OutputMap wiring

Parameters can be defined either as Param nodes inside the library block (shared
name across all instances) or as Input nodes (parent graph provides the value,
allowing per-instance naming).

---

## 6. Graph Templates

Templates are top-level graphs that wire telemetry inputs to sub-graph Includes
and map their outputs to the per-function FFB outputs.

| File | Game | Functions | Description |
| --- | --- | --- | --- |
| `heli_default.json` | X-Plane (helicopters) | Pitch, Roll, Pedals, Collective | Wires rotor telemetry + grip to heli_cyclic (x2), heli_pedals, heli_collective |
| `plane_default.json` | X-Plane (fixed-wing) | Pitch, Roll, Pedals | Wires IAS/alpha/aero torque to plane_pitch, plane_roll, plane_yaw |
| `vehicle_default.json` | Automotive | Per-pedal | Basic automotive template (not flight) |

When a vehicle is first encountered in a supported game, the plugin auto-assigns
the matching template (or prompts the user if multiple apply). The template's
params then appear in the Vehicle tab for per-aircraft tuning.

---

## 7. Per-Aircraft Profiles

Each aircraft gets an `AircraftFfbProfile` stored in plugin settings, keyed by
`gameId::aircraftId`. A profile contains:

- **GraphPath**: which graph template to use
- **GraphParamValues**: per-aircraft parameter overrides (e.g., spring rate,
  damping, load gain — anything defined as a Param node in the graph)
- **FunctionOverrides**: per-function hardware overrides (simulated mass,
  friction, motion range)
- **ActiveFunctionIds**: which functions are active for this aircraft

When switching aircraft, the plugin:
1. Saves the current graph state (trim accumulators, FTR positions)
2. Loads the new aircraft's profile and graph
3. Restores saved graph state for the new aircraft (if any)

Graph state persists across sessions (stored in settings).

---

## 8. Designing a New FFB Strategy

To implement FFB for a specific aircraft:

### Step 1: Choose a base template

- **Helicopter**: Start from `heli_default.json`. The heli_cyclic sub-graph
  provides spring, damper, friction, load, and trim for each axis.
- **Fixed-wing**: Start from `plane_default.json`. The plane_pitch/roll/yaw
  sub-graphs provide qhat-scaled spring/damper with stall buffet.

### Step 2: Tune parameters

All Param nodes in the graph (and included sub-graphs) appear as sliders in
the Vehicle tab. Common parameters to tune:

| Parameter | Effect | Typical range |
| --- | --- | --- |
| `SpringGain` | Centering force stiffness | 0.1 - 1.0 N/mm |
| `DamperGain` | Resistance to movement | 0.01 - 0.1 N-s/mm |
| `FrictionBase` | Breakout / Coulomb friction | 0.5 - 5.0 N |
| `LoadGain` | Aerodynamic force scaling | 0.1 - 2.0 |
| `TrimStep` | Hat trim rate | 1.0 - 20.0 mm/s |
| `TrimMin/Max` | Hat trim range | -30 to +30 mm |
| `Vref` | Reference speed for qhat | Aircraft-specific (kts) |

### Step 3: Modify graph structure (if needed)

For aircraft with unique force characteristics, create a new sub-graph:

1. **Pure spring-damper** (most GA planes, boosted helicopters):
   The default templates work. Just tune parameters.

2. **Aerodynamic force feedback** (unboosted helicopters, cable-controlled planes):
   Add `LoadForce` computation from `XPlane.AeroTorque.*` signals.
   Use `torque_norm` to scale, `Mul` with a gain param.

3. **One-way force limiting** (e.g., MD 500E one-way lock):
   Clamp `LoadForce` to one sign using `Min` or `Max` nodes.

4. **SAS / augmentation** (e.g., Bell 222):
   Compute `LoadForce` from rate/g-load signals. This adds forces on top of
   the spring model, providing manoeuvre cues.

5. **Speed-dependent forces** (airliners, jets):
   Use `qhat_eff(IAS, Vref)` to scale spring/damper/load with airspeed.

6. **Trim strategies**:
   - Electric/beep trim: use `trim_hat.json` (accumulator)
   - FTR (helicopter): use `trim_combined.json` (FTR + hat)
   - No trim needed: omit trim sub-graph, set `TrimOffset = 0`

### Step 4: Add vibration / transient effects

- **Stall buffet**: Already in plane_pitch/roll via `buffet` func node.
  Tune `Buffet.AlphaStart`, `Buffet.AlphaFull`, `BuffetGain`.
- **Rotor vibration**: Use `BuffetAmplitude` output driven by rotor RPM.
  A constant or RPM-scaled amplitude produces continuous vibration.
- **Ground rumble**: Gate `BuffetAmplitude` with `XPlane.OnGround`.
- **Custom patterns**: The `buffet` func produces band-limited noise.
  For periodic vibration (e.g., 2/rev rotor), this would need a new Func
  node or external input signal.

### Step 5: Share

Graph files (`.json`) are self-contained and portable. A new aircraft profile
can be shared as:
- The graph template (`.json`) — place in `graphs/templates/` or `graphs/_embedded/`
- A set of recommended parameter values (screenshot or exported profile)
- Any new library blocks used (place in `graphs/_embedded/`)

---

## 9. Extending the System

### Adding a new graph Func node

New functions are added in two places:

1. **Evaluator** (`GraphCompiledEvaluator.cs`): Add a `case` in `EvalFunc()`.
   For stateful functions, also add to `GetStateSlotsNeeded()`.
2. **Editor** (`GraphEditorControl.xaml.cs`): Add to `_funcChoices` array and
   `GetFuncInputNames()` for port definitions.

### Adding a new FFB output

1. Add the field to `FlightFfbAction` in `diy_ffb_protocol.proto`
2. Add the output signal to `GraphSignalCatalogData.OutputNames`
3. Read it in `TryGetGraphFlightOutputs()` in `DiyFfbPlugin.cs`
4. Pack it in `SendFlightFfb()`
5. Apply it in `FlightStickFunction::on_ffb_action()` on the ESP32

### Adding a new input signal

1. Add to `GraphSignalCatalogData.InputNames` (and `InputGroups` if new group)
2. Populate in `GraphSignalCatalog.BuildXPlaneInputs()` (or new builder method)
3. Call from `DiyFfbPlugin.BuildGraphInputs()`

---

## 10. Key Files Reference

| Component | Path |
| --- | --- |
| Protobuf definitions | `proto/diy_ffb_protocol.proto` |
| ESP32 flight stick force model | `ESP32/src/FlightStickFunction.cpp` |
| ESP32 physics engine | `ESP32/include/Physics.h`, `ESP32/src/Physics.cpp` |
| Graph compiled evaluator | `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs` |
| Graph signal catalog | `SimHubPlugin/GraphSignalCatalogData.cs` |
| Graph signal builders | `SimHubPlugin/GraphSignals.cs` |
| Plugin FFB send path | `SimHubPlugin/DiyFfbPlugin.cs` (`ProcessGraphFfb`, `SendFlightFfb`) |
| Graph editor model | `SimHubPlugin/GraphEditor/GraphModel.cs` |
| Graph templates | `SimHubPlugin/graphs/templates/` |
| Library sub-graphs | `SimHubPlugin/graphs/_embedded/` |
| Trim sub-graphs | `SimHubPlugin/graphs/_embedded/trim_*.json` |
| Button binding model | `SimHubPlugin/ButtonBinding.cs`, `ButtonInputReader.cs` |
| Per-aircraft profiles | `SimHubPlugin/DiyFfbPluginSettings.cs` (`AircraftFfbProfile`) |
| Grip SPI reader | `ESP32/src/GripReader.cpp`, `ESP32/include/GripReader.h` |
