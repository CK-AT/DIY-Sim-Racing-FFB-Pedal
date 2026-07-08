# FFB Graph Signal Catalog

Purpose: centralized list of graph input/output signal names for the editor and runtime.

## How it is used
- Graph input/param/output nodes use their port names as signal keys.
- The editor shows dropdowns for known signal names (input/output ports).
- Runtime mapping uses the same keys; unknown keys evaluate to zero.

## Input signals
These map to X-Plane telemetry and system values.
- `XPlane.Speed.IAS` (kts) 
- `XPlane.Angle.Alpha` (deg)
- `XPlane.Angle.Beta` (deg)
- `XPlane.Trim.Elevator` (normalized -1..1)
- `XPlane.Trim.Aileron` (normalized -1..1)
- `XPlane.Trim.Rudder` (normalized -1..1)
- `XPlane.Rate.Roll` (deg/s) (=p_rate)
- `XPlane.Rate.Pitch` (deg/s) (=q_rate)
- `XPlane.Rate.Yaw` (deg/s) (=r_rate)
- `XPlane.G_Nrml` (g multiples)
- `XPlane.AeroTorque.Roll` (Nm)
- `XPlane.AeroTorque.Pitch` (Nm)
- `XPlane.AeroTorque.Yaw` (Nm)
- `XPlane.MainRotor.Torque` (Nm)
- `XPlane.MainRotor.Speed` (1/min)
- `XPlane.Rotor.BladeAlphPitch` (deg)
- `XPlane.Rotor.BladeAlphRoll` (deg)
- `XPlane.Rotor.Slap` (normalized)
- `XPlane.Rotor.VRS` (normalized)
- `XPlane.Rotor.Propwash` (normalized)
- `XPlane.OnGround` (bool)

## MSFS input signals
These map to MSFS telemetry via the SimConnect bridge (plan 17). They mirror `XPlane.*` where possible; rotor-related signals are derived in the bridge (tier C — IAS-only baselines).

Airframe / motion:

- `MSFS.Speed.IAS` (kts)
- `MSFS.Speed.TAS` (kts)
- `MSFS.Angle.Alpha` (deg)
- `MSFS.Angle.Beta` (deg)
- `MSFS.Attitude.Pitch` (radians)
- `MSFS.Attitude.Bank` (radians)
- `MSFS.Rate.Roll` (radians/s)
- `MSFS.Rate.Pitch` (radians/s)
- `MSFS.Rate.Yaw` (radians/s)
- `MSFS.G_Nrml` (g multiples)
- `MSFS.VVI.World` (ft/min)
- `MSFS.Velocity.BodyX` (ft/s)
- `MSFS.Velocity.BodyY` (ft/s)
- `MSFS.Velocity.BodyZ` (ft/s)
- `MSFS.GroundSpeed` (kts) — ground speed over the surface, from SimConnect `GROUND VELOCITY`; drives the helicopter ground-rumble cue.
- `MSFS.Weight.Total` (lbs)
- `MSFS.Air.Density` (slug/ft^3)
- `MSFS.OnGround` (bool)

Rotorcraft:

- `MSFS.MainRotor.Speed` (1/min)
- `MSFS.TailRotor.Speed` (1/min)
- `MSFS.Eng.TorquePct` (%)
- `MSFS.Collective.Position` (%)
- `MSFS.Collective.BladePitchPct` (%)
- `MSFS.Cyclic.BladePitchPct` (%)
- `MSFS.Cyclic.MaxPitchAngle` (radians)
- `MSFS.TailRotor.PedalPosition` (%)
- `MSFS.TailRotor.BladePitchPct` (%)
- `MSFS.Disk.PitchAngle` (radians)
- `MSFS.Disk.BankAngle` (radians)
- `MSFS.Disk.ConingPct` (%)
- `MSFS.Rotor.LateralTrim` (%)
- `MSFS.Rotor.LongitudinalTrim` (%)
- `MSFS.Rotor.RotationAngle` (radians)

Trim:

- `MSFS.Trim.Elevator` (%)
- `MSFS.Trim.Aileron` (%)
- `MSFS.Trim.Rudder` (%)

Custom vars (plan 23): graph-declared `MsfsVarDef` nodes register additional SimVars at runtime; each is injected as an input signal named `MSFS.<alias>` (the node's alias), so graphs can read any SimVar not listed above.

## Grip / Axis input signals
- `Grip.TrimHat.Up` / `.Down` / `.Left` / `.Right` (bool)
- `Grip.ForceTrimRelease` (bool)
- `Grip.TrimReset` (bool)
- `Axis.{FlightStickPitch|FlightStickRoll|FlightPedals|FlightStickCollective}.Position` (normalized)
- `Axis.{FlightStickPitch|FlightStickRoll|FlightPedals|FlightStickCollective}.Center` (normalized)

## Parameter signals
These can be defined freely, but must be part of a defined set of groups (top level names):
- `<function name>` (like "FlightStickRoll")
- `Aircraft`
- `System`
- `Vehicle`
- `Cyclic`

Example Parameter Names:
- `Aircraft.Vref` (kts)
- `Aircraft.Rotor.SpeedNom` (1/min)
- `Aircraft.Rotor.TorqueNom` (Nm)
- `Aircraft.AeroTorque.RollNom` (Nm)
- `Aircraft.AeroTorque.PitchNom` (Nm)
- `Aircraft.AeroTorque.YawNom` (Nm)

## Output signals
These graph-level outputs are mapped to FFB function terms at runtime.
- `FlightStickPitch.SpringGain`
- `FlightStickPitch.DamperGain`
- `FlightStickPitch.Friction`
- `FlightStickPitch.LoadForce`
- `FlightStickPitch.TrimOffset`
- `FlightStickPitch.BuffetAmplitude`
- `FlightStickRoll.SpringGain`
- `FlightStickRoll.DamperGain`
- `FlightStickRoll.Friction`
- `FlightStickRoll.LoadForce`
- `FlightStickRoll.TrimOffset`
- `FlightStickRoll.BuffetAmplitude`
- `FlightPedals.SpringGain`
- `FlightPedals.DamperGain`
- `FlightPedals.Friction`
- `FlightPedals.LoadForce`
- `FlightPedals.TrimOffset`
- `FlightPedals.BuffetAmplitude`
- `FlightStickCollective.SpringGain`
- `FlightStickCollective.DamperGain`
- `FlightStickCollective.Friction`
- `FlightStickCollective.LoadForce`
- `FlightStickCollective.TrimOffset`
- `FlightStickCollective.BuffetAmplitude`

### DDS vibration outputs

Per-function vibration amplitudes (one set per flight function — wire via Scoped Output):

- `{FlightStickPitch|FlightStickRoll|FlightPedals|FlightStickCollective}.Vib1Ampl1..5` — DDS 1 amplitudes (mm), 0..2.55 mm range. SyncVib output is a position delta on the servo command path (plan 12).
- `{...}.Vib2Ampl1..2` — DDS 2 amplitudes (mm), same range.

Slot semantics are defined by `FlightStickConfig.vib_harmonic_ratios` (DDS 1, up to 5 ratios) and `vib2_harmonic_ratios` (DDS 2). Drive these from ConfigOut nodes using the field paths `flight_stick.vib_harmonic_ratios.0..4` and `flight_stick.vib2_harmonic_ratios.0..1`. Phase offset (for axis split / rotor handedness) goes via ConfigOut on `flight_stick.phase_offset`.

Shared (broadcast — wire via the plain Output node, not Scoped):

- `Shared.Vib1Fund` — DDS 1 fundamental in Hz. Plugin sends this once per FFB tick in a `DdsFundamentals` message; the gateway broadcasts it on CAN `0x0F0` to all axes.
- `Shared.Vib2Fund` — DDS 2 fundamental in Hz. Same path.

## Where to edit
- `SimHubPlugin/GraphSignalCatalogData.cs`
  - `GraphSignalCatalogData.InputNames`
  - `GraphSignalCatalogData.OutputNames`
  - (`SimHubPlugin/GraphSignals.cs` only re-exports these lists.)

## Conventions
- Use dot-notation with clear namespaces, e.g. `XPlane.*`, `FlightStickPitch.*`.
- Inputs are telemetry or system values.
- Outputs are per-function terms routed directly to the ESP32 FFB frames.
