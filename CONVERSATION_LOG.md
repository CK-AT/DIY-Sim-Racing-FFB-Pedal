# Conversation Log
Purpose: keep cross-machine continuity for this repo.
Update policy: append new entries at the top; include date/time, machine, request, summary, key files, and open items.

## 2026-01-01 19:04:06 +01:00 (DESKTOP-6KO022D)
Request: add more general kinematics test cases.
Summary:
- Expanded Python and C# tests to cover invalid configs (negative travel, duplicate pins, bad metering bars, non-collinear bars, shared collinear pins) and coefficient sanity checks.
Key files:
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`

## 2026-01-01 18:50:38 +01:00 (DESKTOP-6KO022D)
Request: tighten the general kinematics solvers.
Summary:
- Added adaptive LM damping, per-variable step caps, and Jacobian condition checks to both Python and C# solvers.
Key files:
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/GeneralKinematics.cs`

## 2026-01-01 18:09:50 +01:00 (DESKTOP-6KO022D)
Request: skip physics simulation when no function is active.
Summary:
- Physics loop now skips `sim.update` and holds the last contact position when `FunctionID_FUNCTION_ID_UNDEFINED`; sled target stays derived from the held contact position.
Key files:
- `ESP32/src/Main.cpp`

## 2026-01-01 17:54:14 +01:00 (DESKTOP-6KO022D)
Request: set firmware default function ID to undefined and guard SimHub for undefined active function.
Summary:
- Cleared default function-specific config so the firmware starts with no active function when `FunctionID_FUNCTION_ID_UNDEFINED`.
- SimHub now ignores undefined ActiveFunction updates and ignores undefined function configs without crashing.
Key files:
- `ESP32/src/ConfigManager.cpp`
- `ESP32/src/Main.cpp`
- `SimhubPlugin/DiyFfbPluginUI.xaml.cs`

## 2026-01-01 13:50:58 +01:00 (DESKTOP-6KO022D)
Request: set firmware default FunctionConfig function_id to undefined.
Summary:
- Changed default function_id to `FunctionID_FUNCTION_ID_UNDEFINED` in the ESP32 ConfigManager defaults.
Key files:
- `ESP32/src/ConfigManager.cpp`

## 2026-01-01 13:43:30 +01:00 (DESKTOP-6KO022D)
Request: implement option 2 collinear-bar handling for general kinematics.
Summary:
- Parameterized 3+ pin bars as rigid lines with bar pose variables in both Python and C# solvers; added fixed constraints for grounded/rail pins on those bars and updated force/Jacobian mapping.
- Updated the plotting script to use the new constraint API and draw collinear bars; added collinear-bar tests in Python and C#.
Key files:
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/plot_kinematic_polynomials.py`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`

## 2026-01-01 13:08:41 +01:00 (DESKTOP-6KO022D)
Request: fix convergence after adding another pin to the sample JSON and align protobuf outputs with updated field indices.
Summary:
- Adjusted the sample pin to avoid a collinear 3‑pin bar (degenerate triangle).
- Regenerated `SimhubPlugin/DiyFfbProtocol.cs` and `ESP32/sim/diy_ffb_protocol_pb2.py` after GeneralKinematicConfig field index changes.
Key files:
- `ESP32/sim/sample_general_kinematic.json`
- `SimhubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`

## 2026-01-01 13:00:28 +01:00 (DESKTOP-6KO022D)
Request: add a sample GeneralKinematicConfig JSON file.
Summary:
- Added `ESP32/sim/sample_general_kinematic.json` using the new rail_travel_negative/positive fields.

## 2026-01-01 12:57:20 +01:00 (DESKTOP-6KO022D)
Request: replace symmetric rail travel with separate negative/positive distances from zero.
Summary:
- Schema: removed `rail_travel`, added `rail_travel_negative`/`rail_travel_positive` in `proto/diy_ffb_protocol.proto`, regenerated protobuf outputs.
- Solvers/tests/plot: updated C# and Python kinematics, tests, and plotting/animation script to use the new fields.
Key files:
- `proto/diy_ffb_protocol.proto`
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/general_kinematics.py`
- `ESP32/sim/test_general_kinematics.py`
- `ESP32/sim/plot_kinematic_polynomials.py`
- `SimhubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`

## 2026-01-01 12:45:36 +01:00 (DESKTOP-6KO022D)
Request: add an animation of the kinematic layout to the plotting script.
Summary:
- Added layout animation support to the polynomial plotting script (with frame/interval options).
Key files:
- `ESP32/sim/plot_kinematic_polynomials.py`

## 2026-01-01 12:37:53 +01:00 (DESKTOP-6KO022D)
Request: add a Python script to plot the kinematic conversion polynomials.
Summary:
- Added a plotting helper that computes/loads KinematicParameters and plots sled/force conversion polynomials.
Key files:
- `ESP32/sim/plot_kinematic_polynomials.py`

## 2026-01-01 12:32:09 +01:00 (DESKTOP-6KO022D)
Request: add test cases, with C# coverage preferred.
Summary:
- Added a simple console-based C# test project covering GeneralKinematics centering and invalid config handling.
- Added a small Python test script mirroring the same checks.
Key files:
- `SimhubPlugin/DiyFfbPlugin.Tests/DiyFfbPlugin.Tests.csproj`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
- `ESP32/sim/test_general_kinematics.py`

## 2026-01-01 12:13:33 +01:00 (DESKTOP-6KO022D)
Request: make contact position zero at the initial pose (rail centered).
Summary:
- Shifted contact position by the center-rail pose using linear interpolation so zero aligns with rail centered.
Key files:
- `SimHubPlugin/GeneralKinematics.cs`
- `ESP32/sim/general_kinematics.py`
Open items:
- None.

## 2026-01-01 12:06:13 +01:00 (DESKTOP-6KO022D)
Request: add a general-purpose kinematic_config (pins/bars, rail travel, metering bar) with SimHub + Python solvers; have nanopb ignore pins/bars.
Summary:
- Protocol: added `GeneralKinematicPin`/`GeneralKinematicBar`/`GeneralKinematicConfig` (with `rail_travel`) and `AxisConfig.general_kinematic` in `proto/diy_ffb_protocol.proto`; nanopb options now ignore `GeneralKinematicConfig.pins` and `.bars` in `proto/diy_ffb_protocol.options`.
- SimHub: added `SimHubPlugin/GeneralKinematics.cs` solver (constraint solve + polynomial fit), wired `SimHubPlugin/AxisConfigControl.xaml.cs`, updated `SimHubPlugin/DiyFfbPlugin.csproj`, regenerated `SimHubPlugin/DiyFfbProtocol.cs`.
- Python: added `ESP32/sim/general_kinematics.py`, regenerated `ESP32/sim/diy_ffb_protocol_pb2.py`.
Notes:
- No tests run.

## 2025-12-31 15:30:33 +01:00 (DESKTOP-6KO022D)
Request: add dedicated FlightStickPitch/FlightStickRoll configs and matching sim scripts.
Summary:
- Protocol: added FunctionID FLIGHT_STICK_PITCH/ROLL and new FlightStickPitchConfig/FlightStickRollConfig in `proto/diy_ffb_protocol.proto`, with sizing hints in `proto/diy_ffb_protocol.options`.
- Firmware: added `ESP32/include/FlightStickFunction.h` and `ESP32/src/FlightStickFunction.cpp`, wired in `ESP32/src/Main.cpp`.
- SimHub: added `SimHubPlugin/FlightStickConfigControl.xaml` and `.xaml.cs`, wired in `SimHubPlugin/FunctionConfigControl.xaml` and `.xaml.cs`, extended function list in `SimHubPlugin/DiyFfbPluginUI.xaml.cs`, regenerated `SimHubPlugin/DiyFfbProtocol.cs`.
- Sim scripts: added `ESP32/sim/flight_stick_pitch.py` and `ESP32/sim/flight_stick_roll.py`; updated brake/accelerator sim scripts to *_PEDAL enum names; regenerated `ESP32/sim/diy_ffb_protocol_pb2.py` (untracked).
Defaults:
- Stick range -50..50, damping 0.5, centering 1.5, output mode TRAVEL.
- Pitch controller axis Y, roll controller axis X.
Axis IDs:
- `ESP32/sim/flight_stick_pitch.py` uses AXIS_ID_4.
- `ESP32/sim/flight_stick_roll.py` uses AXIS_ID_5.
Notes:
- No tests run.
- Unrelated modified/untracked files existed (DLL/PDB and various `DIY-FFB.srctrl*`/`ESP32/*` files), intentionally ignored.
