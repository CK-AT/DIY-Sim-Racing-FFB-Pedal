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
- `XPlane.OnGround` (bool)

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
These are the graph-level outputs we plan to map to FFB function terms.
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

## Where to edit
- `SimHubPlugin/GraphSignals.cs`
  - `GraphSignalCatalog.InputNames`
  - `GraphSignalCatalog.OutputNames`

## Conventions
- Use dot-notation with clear namespaces, e.g. `XPlane.*`, `FlightStickPitch.*`.
- Inputs are telemetry or system values.
- Outputs are per-function terms (no direct hardware routing yet).
