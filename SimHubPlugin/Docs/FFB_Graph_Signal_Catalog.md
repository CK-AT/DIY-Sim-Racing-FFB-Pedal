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

### DDS vibration outputs

Per-function vibration amplitudes (one set per flight function — wire via Scoped Output):

- `{FlightStickPitch|FlightStickRoll|FlightPedals|FlightStickCollective}.VibSlot1..5` — DDS 1 amplitudes (N), 0..2.55 N range.
- `{...}.Vib2Slot1..2` — DDS 2 amplitudes (N), same range.

Slot semantics are defined by `FlightStickConfig.vib_harmonic_ratios` (DDS 1, up to 5 ratios) and `vib2_harmonic_ratios` (DDS 2). Drive these from ConfigOut nodes using the field paths `flight_stick.vib_harmonic_ratios.0..4` and `flight_stick.vib2_harmonic_ratios.0..1`. Phase offset (for axis split / rotor handedness) goes via ConfigOut on `flight_stick.phase_offset`.

Shared (broadcast — wire via the plain Output node, not Scoped):

- `Shared.VibFundamental` — DDS 1 fundamental in Hz. Plugin sends this once per FFB tick in a `DdsFundamentals` message; the gateway broadcasts it on CAN `0x0F0` to all axes.
- `Shared.Vib2Fundamental` — DDS 2 fundamental in Hz. Same path.

## Where to edit
- `SimHubPlugin/GraphSignals.cs`
  - `GraphSignalCatalog.InputNames`
  - `GraphSignalCatalog.OutputNames`

## Conventions
- Use dot-notation with clear namespaces, e.g. `XPlane.*`, `FlightStickPitch.*`.
- Inputs are telemetry or system values.
- Outputs are per-function terms routed directly to the ESP32 FFB frames.
