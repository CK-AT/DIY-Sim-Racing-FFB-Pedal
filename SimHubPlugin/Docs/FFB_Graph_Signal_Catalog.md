# FFB Graph Signal Catalog

Purpose: centralized list of graph input/output signal names for the editor and runtime.

## How it is used
- Graph input/param/output nodes use their port names as signal keys.
- The editor shows dropdowns for known signal names (input/output ports).
- Runtime mapping uses the same keys; unknown keys evaluate to zero.

## Input signals (current)
These map to X-Plane telemetry and system values.
- `XPlane.IAS_kts`
- `XPlane.Alpha_deg`
- `XPlane.Beta_deg`
- `XPlane.PRate`
- `XPlane.QRate`
- `XPlane.RRate`
- `XPlane.GNrml`
- `XPlane.AeroTorque.RollNm`
- `XPlane.AeroTorque.PitchNm`
- `XPlane.AeroTorque.YawNm`
- `XPlane.Vref_kts`
- `XPlane.NominalRpm`
- `XPlane.MrTorqueRefNm`
- `XPlane.MainRotorTorqueNm`
- `XPlane.MainRotorRpm`
- `XPlane.OnGround`

## Output signals (current)
These are the graph-level outputs we plan to map to FFB function terms.
- `FlightStickPitch.SpringGain`
- `FlightStickPitch.DamperGain`
- `FlightStickPitch.Friction`
- `FlightStickPitch.LoadForce`
- `FlightStickPitch.TrimOffset`
- `FlightStickRoll.SpringGain`
- `FlightStickRoll.DamperGain`
- `FlightStickRoll.Friction`
- `FlightStickRoll.LoadForce`
- `FlightStickRoll.TrimOffset`
- `FlightPedals.SpringGain`
- `FlightPedals.DamperGain`
- `FlightPedals.Friction`
- `FlightPedals.LoadForce`
- `FlightPedals.TrimOffset`
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
