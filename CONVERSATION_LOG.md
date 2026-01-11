# Conversation Log
Purpose: keep cross-machine continuity for this repo.
Update policy: append new entries at the top; include date/time, machine, request, summary, key files, and open items.

## 2026-01-10 13:33:21 +01:00 (notebook-ckrenn)
Request: add per-folder pinned requirements for ESP32/sim.
Summary:
- Removed root requirements.txt and added pinned dependencies in ESP32/sim/requirements.txt.
Key files:
- `ESP32/sim/requirements.txt`
- `requirements.txt`
Open items:
- None.

## 2026-01-10 12:06:00 +01:00 (notebook-ckrenn)
Request: note OTA pull dependency removal and local header integration.
Summary:
- Removed ESP32OTAPull as a lib dependency and added the modified header to sources.
Key files:
- `include/ESP32OTAPull.h`
- `platformio.ini`
Open items:
- Ensure build includes the local header and no stale library references remain.

## 2026-01-10 12:03:53 +01:00 (notebook-ckrenn)
Request: assess FW versions on OTA CLI start and only update axes that responded.
Summary:
- Added a startup DeviceInfo assessment for all axes and gateways with a printed summary.
- OTA now targets/retries only axes that responded during assessment (skips missing axes or sends per-axis messages as needed).
Key files:
- `ESP32/sim/ota_update_cli.py`
Open items:
- Validate OTA flow on hardware with some axes offline.

## 2026-01-09 10:38:43 +01:00 (notebook-ckrenn)
Request: add DeviceInfo with unique device identifier.
Summary:
- Added DeviceInfo/DeviceInfoRequest to the protocol, including a device UID from eFuse MAC.
- Implemented boot-time DeviceInfo broadcast and request handling in CommManager.
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/src/CommManager.cpp`
- `ESP32/include/CommManager.h`
Open items:
- Regenerate protobuf outputs (C#/Python) if needed by tools.

## 2026-01-08 19:07:49 +01:00 (notebook-ckrenn)
Request: add MD5 verification to OTA updates.
Summary:
- Added `MD5` parsing in ESP32-OTA-Pull and wired it into Update.setMD5.
- Updated OTA JSON and CLI generator to emit the MD5 hash.
Key files:
- `ESP32/.pio/libdeps/a6-servo-ffb-axis-controller-v10-ck-at/ESP32-OTA-Pull/src/ESP32OTAPull.h`
- `OTA/update_info.json`
- `ESP32/sim/ota_update_cli.py`
Open items:
- Validate OTA succeeds with correct MD5 and fails on mismatch.

## 2026-01-08 00:43:21 +01:00 (notebook-ckrenn)
Request: extend OTA CLI to host JSON + firmware binary.
Summary:
- OTA CLI now spins up a local HTTP server that serves `update_info.json` and `firmware.bin`, and sends the generated URL to the device.
Key files:
- `ESP32/sim/ota_update_cli.py`
Open items:
- None.

## 2026-01-08 00:32:18 +01:00 (notebook-ckrenn)
Request: add CLI for OTA updates.
Summary:
- Added an OTA CLI that sends StartOtaUpdate over USB and optionally tails log messages.
Key files:
- `ESP32/sim/ota_update_cli.py`
Open items:
- None.

## 2026-01-07 19:23:24 +01:00 (DESKTOP-PUK6UGO)
Request: optimize ShifterFunction lane selection and fix native PI build error.
Summary:
- Added fallback `PI` definition in `Physics.h` to fix native builds.
- Added seg-to-lane lookup tables and inside-mask reuse (bitmask) in `ShifterGateRuntime` to reduce per-update scans.
Key files:
- `ESP32/include/Physics.h`
- `ESP32/include/ShifterFunction.h`
Open items:
- None.

## 2026-01-07 19:02:48 +01:00 (DESKTOP-PUK6UGO)
Request: shifter detent plotting and cam tuning; fix shifter test signature; add centering to plots.
Summary:
- Updated `test_shifter_native` to call the new `ShifterFunction::update_config` signature (with detect config) and set a usable centering spring in the simple gate config.
- Enhanced detent visualizer `plot_shifter_detents.py` with centering spring support, per-lane force curves, and robust config loading for sparse configItems.
- Added `cam_test.py` to explore lane cam profiles, roller follower effects, centering spring, and more realistic fork-style pocket tuning.
Key files:
- `ESP32/test/test_shifter_native/test_shifter.cpp`
- `ESP32/sim/plot_shifter_detents.py`
- `ESP32/sim/cam_test.py`
Open items:
- Native tests not run here (PlatformIO/gcc unavailable); run `pio test -e native -f test_shifter_native` after installing toolchain.

## 2026-01-03 21:01:21 +01:00 (DESKTOP-6KO022D)
Request: debug USB gateway receive; add logging on ISOTP errors.
Summary:
- Increased PacketSerial receive buffer to 1024 bytes to allow larger USB packets.
- Added throttled SerialManager overflow logging for oversized packets.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `ESP32/include/SerialManager.h`
- `ESP32/src/SerialManager.cpp`
Open items:
- Verify gateway now receives shifter function configs over USB.

## 2026-01-03 20:46:38 +01:00 (DESKTOP-6KO022D)
Request: add ISOTP error logging for CAN path.
Summary:
- Added throttled ISOTP send/receive error logs with return codes and payload lengths.
- Logged CAN ISOTP errors for gateway/axis traffic and log-ack messages.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `ESP32/src/CANManager.cpp`
Open items:
- Check SimHub logs for CAN ISOTP errors during shifter uploads.

## 2026-01-03 16:28:43 +01:00 (DESKTOP-6KO022D)
Request: shifter UI polish (live marker + axis range limits) and fix build error.
Summary:
- Added live shifter position marker driven by AxisState (with sequential fallback to X midpoint).
- Unlocked X range in sequential mode and clamped X/Y range sliders to selected axis travel ranges.
- Fixed CS0206 by avoiding ref on protobuf properties in range clamp helper.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-03 16:10:28 +01:00 (DESKTOP-6KO022D)
Request: shifter detection/visualization updates, sequential demo, and X-midpoint centering.
Summary:
- Renamed shifter detection schema to gear-based slots (ShifterGear enum + gear_slots) and updated firmware/tests/UI + demo configs.
- Fixed empty gear dropdown by setting the DataGridComboBoxColumn ItemsSource in code-behind.
- Added shifter force-field plotter, sequential demo config, and sequential X-midpoint sampling derived from the X motion range.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/src/ShifterDetect.cpp`
- `ESP32/test/test_shifter_native/test_shifter.cpp`
- `ESP32/include/ShifterFunction.h`
- `ESP32/src/ShifterFunction.cpp`
- `ESP32/sim/plot_shifter_force_fields.py`
- `SimhubPlugin/ShifterConfigControl.xaml`
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
- `SimhubPlugin/shifter_hpattern_demo_config.json`
- `SimhubPlugin/shifter_hpattern_demo_config_modified.json`
- `SimhubPlugin/shifter_sequential_demo_config.json`
Open items:
- None.

## 2026-01-03 02:42:55 +01:00 (DESKTOP-6KO022D)
Request: remove shifter controller-axis output UI (buttons only).
Summary:
- Removed controller axis selector from shifter config UI and force the output axis to Undefined.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/ShifterConfigControl.xaml`
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-03 02:38:05 +01:00 (DESKTOP-6KO022D)
Request: add SimHub UI for shifter config editing.
Summary:
- Added ShifterConfigControl with axis selection, range inputs, force settings, and editable gate/detent tables.
- Wired shifter tab into FunctionConfigControl and function list population.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/ShifterConfigControl.xaml`
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
- `SimhubPlugin/FunctionConfigControl.xaml`
- `SimhubPlugin/FunctionConfigControl.xaml.cs`
- `SimhubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimhubPlugin/DiyFfbPlugin.csproj`
Open items:
- None.

## 2026-01-03 02:14:03 +01:00 (DESKTOP-6KO022D)
Request: implement shifter physics + config (gate geometry, detents, sequential).
Summary:
- Protocol: added shifter function ID and ShifterConfig with gate segments and detent points.
- Firmware: added ShifterFunction with 2D LUT-based force field and axis role handling.
- Regenerated C# and Python protobuf outputs.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `proto/diy_ffb_protocol.proto`
- `ESP32/include/ShifterFunction.h`
- `ESP32/src/ShifterFunction.cpp`
- `ESP32/src/Main.cpp`
- `proto/diy_ffb_protocol.options`
- `SimHubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`
Open items:
- None.

## 2026-01-03 01:54:23 +01:00 (DESKTOP-6KO022D)
Request: add shifter detection config + 8 joystick buttons.
Summary:
- Protocol: added shifter slot detection config and wired it into AuxFunctionConfig; updated nanopb options.
- Firmware: added ShifterDetect aux function to set joystick buttons from slot regions.
- Regenerated C# and Python protobuf outputs.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `proto/diy_ffb_protocol.proto`
- `ESP32/src/ShifterDetect.cpp`
- `ESP32/include/ShifterDetect.h`
- `ESP32/src/Main.cpp`
- `proto/diy_ffb_protocol.options`
- `SimHubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`
Open items:
- None.

## 2026-01-03 01:42:04 +01:00 (DESKTOP-6KO022D)
Request: increase joystick output to 8 buttons for shifter detection.
Summary:
- Firmware joystick now advertises 8 buttons and applies button states alongside axis outputs.
- Added CommManager helper to set button values for aux functions.
Key files:
- `ESP32/src/CommManager.cpp`
- `ESP32/include/CommManager.h`
Open items:
- None.

## 2026-01-02 21:40:09 +01:00 (DESKTOP-6KO022D)
Request: grounded pins enhanced using a ground-symbol.
Summary:
- Consolidated grounded-pin marker iterations into a single update for a mechanical ground symbol style.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:32:37 +01:00 (DESKTOP-6KO022D)
Request: make rail end caps more visible and add clearer grounded pin cues.
Summary:
- Rail end caps are now longer with a soft halo line behind them for visibility through pin overlays.
- Grounded pins now draw a diamond outline marker behind the pin dot for quick identification.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:27:40 +01:00 (DESKTOP-6KO022D)
Request: improve rail end caps visibility.
Summary:
- Replaced rail end dots with short vertical end cap lines for better contrast.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:22:55 +01:00 (DESKTOP-6KO022D)
Request: drop Config pose mode and improve the rail travel visualization.
Summary:
- Removed Config from pose mode selector; only Live and Test remain.
- Rail travel line now renders as a thicker track with a highlighted guide line and end caps.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:06:45 +01:00 (DESKTOP-6KO022D)
Request: apply 0.1mm resolution to all kinematic inputs.
Summary:
- Rail travel and test position inputs now round to 0.1mm; rail values normalize to non-negative.
- Test position wheel now supports Shift for 0.1mm steps; pin rounding already enforces 0.1mm.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:01:36 +01:00 (DESKTOP-6KO022D)
Request: limit pin X/Y resolution to 0.1mm.
Summary:
- Pin X/Y values now round to 0.1mm on update to keep stored coordinates at the desired resolution.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:56:01 +01:00 (DESKTOP-6KO022D)
Request: mouse wheel stepping for pin X/Y and rail inputs, with 0.1mm steps on shift.
Summary:
- Added mouse wheel adjustments on PinGrid X/Y cells and Rail +/- inputs (1mm steps, 0.1mm with Shift).
- Rail inputs clamp to >= 0; pin inputs apply deltas directly.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:30:58 +01:00 (DESKTOP-6KO022D)
Request: clamp Test pose position to the valid range and add mouse wheel 1mm/1N steps.
Summary:
- Test position now clamps to the cached contact range; switching to Test or rebuilding clamps it too.
- Added mouse wheel adjustments for test position/force (1mm/1N steps) and kept live update in Test mode.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:24:02 +01:00 (DESKTOP-6KO022D)
Request: fix NullReferenceException during plugin settings load in PoseModeCombo_SelectionChanged.
Summary:
- Guarded the pose mode selection handler during XAML initialization to avoid nulls and set isLoading before InitializeComponent.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:18:33 +01:00 (DESKTOP-6KO022D)
Request: add a Test pose mode with force and position inputs.
Summary:
- Added a Test pose mode with position/force inputs; the pose interpolation and force labels now use the selected mode.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:02:47 +01:00 (DESKTOP-6KO022D)
Request: center the contact force label above the arrow shaft.
Summary:
- Contact force label now anchors to the arrow shaft midpoint so it stays centered above the arrow.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:00:43 +01:00 (DESKTOP-6KO022D)
Request: move force labels above the arrow and sensor icon.
Summary:
- Contact force label now sits above the arrow tip; metered force label is centered above the sensor icon.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 18:41:28 +01:00 (DESKTOP-6KO022D)
Request: set the metering sensor icon size to 35x25 mm.
Summary:
- Updated the metering sensor icon to use fixed world dimensions of 35mm by 25mm (scaled by the canvas zoom).
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 18:15:04 +01:00 (DESKTOP-6KO022D)
Request: show contact force and measured force values next to the arrow and metering sensor.
Summary:
- Added force labels: contact point force from AxisState and measured force computed via the kinematic force factor.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 16:07:47 +01:00 (DESKTOP-6KO022D)
Request: align the contact force arrow with the contact point path direction.
Summary:
- Updated the contact arrow to follow the path tangent derived from the pose cache (positive travel direction) instead of bar geometry.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 16:02:17 +01:00 (DESKTOP-6KO022D)
Request: add a contact force arrow and a metering bar sensor icon in the kinematics view.
Summary:
- Added a contact point force arrow that points toward the contact pin based on nearby bar geometry.
- Added a stylized force sensor icon centered on the metering bar with rotation matching bar direction; ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 13:15:36 +01:00 (DESKTOP-6KO022D)
Request: reverse zoom direction, remove scale controls, and rename Recalc to Fit.
Summary:
- Reversed mouse wheel zoom direction and removed the scale buttons/label from the kinematics UI; renamed the Recalc button to Fit.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 13:07:39 +01:00 (DESKTOP-6KO022D)
Request: add zoom/pan to the general kinematics canvas and repurpose Recalc to reset view.
Summary:
- Added mouse wheel zoom and left-drag pan; updated scale limits and recalc button now resets the view to the auto-fit.
- Enabled canvas clipping so graphics do not draw outside the visualization area; ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 12:36:36 +01:00 (DESKTOP-6KO022D)
Request: add a migration check for a legacy DIY pedal config example.
Summary:
- Added a C# test that parses `axis1_diy_pedal_config.json` and verifies the DIY→General migration produces a valid kinematic config.
- Added `ESP32/sim/README.md` documenting the legacy example; ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
- `ESP32/sim/README.md`
- `ESP32/sim/axis1_diy_pedal_config.json`
- `ESP32/sim/test_general_kinematics.py`
Open items:
- None.

## 2026-01-02 12:20:43 +01:00 (DESKTOP-6KO022D)
Request: assume the rail is to the right in the DIY->General migration.
Summary:
- Updated the migration helper to place the rail interface pin at the minimum rail position and set travel as 0..stroke (no centering).
- Ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 12:16:24 +01:00 (DESKTOP-6KO022D)
Request: delete legacy DiyPedal kinematics files and add a migration helper.
Summary:
- Removed `DiyPedalKinematics` XAML/control files from the plugin; existing configs now migrate via a DIYPedal -> GeneralKinematic conversion.
- Conversion centers the rail travel, computes the link mount by circle intersection, and builds pedal + metering bars; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `SimhubPlugin/DiyPedalKinematics.xaml`
- `SimhubPlugin/DiyPedalKinematics.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 12:05:54 +01:00 (DESKTOP-6KO022D)
Request: remove DIY pedal kinematics, move general kinematics into the main tab control, and rename it to kinematics.
Summary:
- Removed the DIY pedal kinematics selector/control and placed the general kinematics UI as the first tab ("Kinematics").
- Default axis configs now build a basic GeneralKinematicConfig; DiyPedalKinematics removed from the SimHub plugin build.
- Renamed the GeneralKinematicsControl header label; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/AxisConfigControl.xaml`
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/DiyFfbPlugin.csproj`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:41:50 +01:00 (DESKTOP-6KO022D)
Request: add per-bar pin rings and color the bar list entries.
Summary:
- Added per-pin color rings for every bar (stacked when a pin belongs to multiple bars) and use the same palette to color bar outlines.
- Colored the Bar list "Pins" column to match bar colors; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:31:29 +01:00 (DESKTOP-6KO022D)
Request: emphasize zero axes and color bar outlines/first pins.
Summary:
- Added emphasized grid lines at x=0/y=0 and drew bar outlines with per-bar colors; first pin of each bar now gets a matching highlight ring.
- Bars now draw all pins (polyline closed for 3+ pins) instead of only farthest pair; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:21:01 +01:00 (DESKTOP-6KO022D)
Request: auto-fit the kinematic visualization to all poses.
Summary:
- Added auto-fit logic to compute scale/offset from pose bounds (or config fallback) and update on cache rebuild and canvas resize.
- Ran Python and C# test suites (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:09:35 +01:00 (DESKTOP-6KO022D)
Request: run Python and C# tests after kinematics changes.
Summary:
- Python: `ESP32/.venv/Scripts/python.exe ESP32/sim/test_general_kinematics.py` (17 tests, 0 failures).
- C#: built `SimhubPlugin/DiyFfbPlugin.Tests` with `BuildProjectReferences=false` and ran the exe (17 tests, 0 failures); full plugin build via `dotnet run` still fails due to missing XAML-generated code in this environment.
Key files:
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:03:24 +01:00 (DESKTOP-6KO022D)
Request: add edge-case topology tests and verify extra collinear pins do not change polynomials.
Summary:
- Added Python/C# tests for unknown pin references, zero-length bars, and a regression check that adding a collinear bar pin leaves the polynomials unchanged.
Key files:
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`

## 2026-01-02 10:54:38 +01:00 (DESKTOP-6KO022D)
Request: represent rigid bars as pose variables (x,y,theta) with fixed pin offsets.
Summary:
- Non-collinear multi-pin bars now use pose variables with per-pin local offsets instead of rigid distance constraints in the Python and C# solvers.
- Updated bar position, Jacobian, and force projection math to use local offsets; the plot animation path uses the new constraint outputs.
Key files:
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/plot_kinematic_polynomials.py`

## 2026-01-02 03:54:48 +01:00 (DESKTOP-6KO022D)
Request: derive bar type from pin geometry (collinear vs rigid).
Summary:
- 3+ pin bars are now treated as collinear only if the pins lie on a line; otherwise they become rigid bars via distance constraints.
- Updated Python/C# solvers and tests for the new behavior.
Key files:
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
- `ESP32/sim/test_general_kinematics.py`

## 2026-01-02 02:33:29 +01:00 (DESKTOP-6KO022D)
Request: preserve GeneralKinematicConfig pins/bars when the FW returns axis config.
Summary:
- Implemented raw round-trip of axis_config protobuf payloads so ignored GeneralKinematicConfig.pins/bars are preserved on return.
- Cached raw axis_config bytes on update/load and used them when replying to return-axis-config; raw buffer is now dynamically sized.
Key files:
- `ESP32/src/ConfigManager.cpp`
- `ESP32/include/ConfigManager.h`
- `ESP32/src/CommManager.cpp`

## 2026-01-01 21:08:21 +01:00 (DESKTOP-6KO022D)
Request: fix missing bar segment in Config mode.
Summary:
- Config-mode bars now render using the farthest pin pair so collinear multi-pin bars show their full span.
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`

## 2026-01-01 20:59:13 +01:00 (DESKTOP-6KO022D)
Request: add a Live/Config toggle for the general kinematics canvas.
Summary:
- Added pose mode selector and update gating so the canvas can show config positions or live axis state.
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`

## 2026-01-01 20:27:14 +01:00 (DESKTOP-6KO022D)
Request: add an active switch between DIY pedal and general kinematics.
Summary:
- Added a kinematic model selector to AxisConfigControl and wired switching to preserve per-mode configs.
Key files:
- `SimhubPlugin/AxisConfigControl.xaml`
- `SimhubPlugin/AxisConfigControl.xaml.cs`

## 2026-01-01 19:58:42 +01:00 (DESKTOP-6KO022D)
Request: SimHub general kinematics UI with pin picker and cached pose interpolation.
Summary:
- Added GeneralKinematicsControl with pin/bar lists, pin picker context menu, rail travel inputs, and cached pose visualization driven by axis state.
- Added GeneralKinematics.BuildPoseCache for sample pose caching and wired AxisConfigControl to show the new control.
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `SimhubPlugin/GeneralKinematics.cs`
- `SimhubPlugin/AxisConfigControl.xaml`
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `SimhubPlugin/DiyFfbPlugin.csproj`

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
- Adjusted the sample pin to avoid a collinear 3-pin bar (degenerate triangle).
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
