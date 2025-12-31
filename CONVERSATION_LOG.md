# Conversation Log
Purpose: keep cross-machine continuity for this repo.
Update policy: append new entries at the top; include date/time, machine, request, summary, key files, and open items.

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
