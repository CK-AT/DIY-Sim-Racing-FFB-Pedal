# DIY Sim Racing FFB Pedal – ESP32 Firmware

Firmware for the force‑feedback pedal axis used in the DIY Sim Racing project. It runs on several ESP32 / ESP32‑S3 control boards, drives the motor/servo, samples the load cell via an ADS1256, exposes the axis as a USB HID device, and synchronises multiple axes over CAN/ESP‑Now. Config and telemetry are exchanged with the PC host/gateway using the `diy_ffb_protocol` protobuf schema.

## Hardware Targets
- Boards: ESP32 DevKit, ESP32‑S3 DevKit C, SpeedCrafter, CK-AT prototypes (see `platformio.ini` environments and `include/Main.h` pinouts).
- Sensors/actuation: ADS1256 load cell ADC, A6-series servo (or stepper in older builds), optional WS2812 status LED.
- Connectivity: USB HID joystick, CAN bus, optional ESP‑Now; OTA updates via Wi‑Fi + GitHub (see `CommManager`).

## Firmware Architecture (key modules)
- `src/Main.cpp` – bootstraps hardware, launches the physics task at the ADC rate, pushes motor setpoints, and forwards FFB/axis actions.
- `ConfigManager.*` – loads/saves axis and function configs to NVS, maintains kinematic/force lookup tables, exposes the active function (e.g., automotive pedal vs. flight rudder/brake).
- `CommManager.*` – bridges USB serial, CAN, and optional wireless links; reports state as a HID joystick; handles OTA pull updates.
- `Physics.*` and `ForceCurve.*` – simple physics simulation (mass/friction) plus tunable force curves.
- `LoadCell.*`, `SignalFilter*` – ADS1256 acquisition and Kalman-based filtering.
- `A6Servo.*` / `FastNonAccelStepper.*` – motor/servo abstraction with lock-in, homing, and position commands.
- `AutomotivePedalFunction.*`, `FlightPedalFunction.*`, `RudderBrake.*` – behaviour for different axis roles and auxiliary functions.

## Building & Flashing
1) Install PlatformIO (VS Code extension or CLI).  
2) Pick an environment from `platformio.ini` that matches your board; the default is `a6-servo-ffb-axis-controller-v10-ck-at` (ESP32-S3 + A6 servo, PCB v13).  
3) Build and flash:
```sh
pio run -e a6-servo-ffb-axis-controller-v10-ck-at
pio run -e a6-servo-ffb-axis-controller-v10-ck-at -t upload
```
4) Open a serial monitor (3,000,000 baud for S3 builds, 115,200 for early boards) to watch logs:
```sh
pio device monitor -b 3000000
```

## OTA Update Helper
- The OTA helper (`ESP32/sim/ota_update_cli.py`) can host a `.ffbota` container (manifest + firmware) over HTTP or point devices to a public `update_info.json`.
- The CLI derives the expected firmware version from the `.ffbota` manifest or public JSON and verifies the `.ffbota` MD5 before serving.

Local OTA with `.ffbota`:

```sh
./ESP32/sim/ota_update_cli.py COM3 --ssid YOUR_SSID --password YOUR_PASS --firmware OTA/firmware_a6-servo-ffb-axis-controller-v10-ck-at.ffbota
```

Public OTA JSON:

```sh
./ESP32/sim/ota_update_cli.py COM3 --ssid YOUR_SSID --password YOUR_PASS --url http://host/update_info.json
```

## Configuration & Operation
- Axis/function configs are stored in NVS; defaults are applied on first boot (see `ConfigManager::set_*_defaults`).  
- Update configs over USB serial or via a gateway using the protobuf messages (use the SimHub plugin/host tools from the root project).  
- Axis mode vs. gateway mode is selected automatically based on the uploaded config; gateways forward CAN/ESP‑Now traffic and present combined HID output.  
- The physics loop runs at the ADC sample rate (default 1 kHz). Filter choice (`kf_const_vel`, `kf_const_accel`, or none), friction, simulated mass, and force curves are all configurable per axis.  
- RGB LED (if fitted) signals status: red = error/disabled, yellow = unlocked servo, green = ready/locked.

## Useful Paths
- Pin mappings and PCB variants: `include/Main.h`, `include/Version_Board.h`.  
- Protocol definitions: `../proto/diy_ffb_protocol.proto` (referenced by `platformio.ini`).  
- OTA setup and Wi‑Fi credentials: see `CommManager` OTA states and `ConfigManager` storage.

## Development Tips
- Keep cycle times deterministic; heavy debug output can starve the physics task—use `DEBUG_INFO_*` flags sparingly.  
- If you change PCB revision, adjust the `PCB_VERSION` build flag or pick the matching PlatformIO environment.  
- Use `pio run -t compiledb` to refresh `compile_commands.json` for clangd.  
- When testing multi-axis setups, ensure unique axis IDs in the uploaded configs to avoid CAN/joystick collisions.
