[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](http://creativecommons.org/licenses/by-nc-sa/4.0/)

# DIY-FFB — Force-Feedback Actuator Platform

> **Fork notice** — This is a substantially extended fork of
> [ChrGri/DIY-Sim-Racing-FFB-Pedal](https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal).
> It started as a DIY active *pedal* and has since grown into a general force-feedback
> actuator platform with native flight-sim support (MSFS, X-Plane), a node-based signal-processing
> engine, multiple host plugins, and reworked ESP32 firmware. See [Credits](#credits) for the
> original authors, and [License](#license) for how attribution is handled.

# Disclaimer

An FFB actuator is a robot and can be dangerous. If not interacted with carefully it may cause harm.
The authors are not responsible for any harm caused by this design. Use responsibly and at your own risk.

# What this project is

A force-feedback "ecosystem" (originally a sim-racing pedal) built around DIY actuators, each controlled by an ESP32-S3
driving an A6-series servo (via a servo abstraction to make adding future support for other servo types easier), sensing force via a load cell measured by an ADS1256 ADC.
A SimHub plugin provides the UI to configure the axes and functions and derives FFB cues from live simulator telemetry.

The project now covers three broad use cases from the same firmware and config model:

- **Sim racing** — throttle/brake/clutch pedals with force curves, ABS/TC/RPM effects (via SimHub).
- **Flight simulation** — rudder pedals, emulated toe brakes, cyclic/collective sticks and helicopter/aircraft
  load & vibration cues, driven from **Microsoft Flight Simulator (2020/2024)** and **X-Plane**.
- **Input devices** — up to eight axes present as a single USB-HID device via a gateway (same hardware and firmware) to ease input mapping within simulators

# Highlights of this fork

Relative to the upstream project, this repository adds:

| Area | What's new |
| :--- | :--- |
| **Flight sims** | Native MSFS SimConnect integration (pure-C#), X-Plane DataRef provider, helicopter & aircraft force models (load, vibration, trim). |
| **Signal processing** | A node-based **graph engine** in the SimHub plugin: telemetry → DSP nodes → actuator force, editable in a visual graph editor with reusable templates and embedded subgraphs. |
| **Config model** | **Axes** assigned to **Functions**, Tiered **Hardware → Profile → User** configuration with per-function overrides |
| **New subprojects** | `XPlanePlugin` and `GripController` — none of which exist upstream. |
| **Firmware** | Multi-axis sync over CAN, admittance control based on a custom physics engine, A6 servo protocol work, sensorless homing and Wi-Fi OTA. |

# Repository layout

| Path | Description |
| :--- | :--- |
| [`ESP32/`](ESP32/) | Actuator firmware (PlatformIO). See [`ESP32/README.md`](ESP32/README.md). |
| [`SimHubPlugin/`](SimHubPlugin/) | SimHub plugin: config UI, graph engine, effects, in-process MSFS/X-Plane bridges. |
| [`XPlanePlugin/`](XPlanePlugin/) | X-Plane DataRef data provider + DataRefLogger. |
| [`GripController/`](GripController/) | Standalone USB-HID grip firmware (ESP32-S3, AS5600 + buttons). |
| [`proto/`](proto/) | `diy_ffb_protocol` protobuf schema (host ↔ firmware). |
| [`docs/`](docs/) | Design plans and architecture notes. |
| [`SimHubPlugin/graphs/`](SimHubPlugin/graphs/) | Graph templates (automotive, heli, plane). |

# Software

## ESP32 firmware

The firmware runs on a custom ESP32-S3 based control board for now, drives the motor/servo, samples the
load cell via an ADS1256, exposes the axis as a **USB HID** device, and synchronises multiple axes
over CAN. Config and telemetry are exchanged with the host using the `diy_ffb_protocol`
protobuf schema.

**Build & flash (PlatformIO):**

```sh
# pick the environment matching your board (see ESP32/platformio.ini)
pio run   -e a6-ffb-v10-ck-at            # build (default: ESP32-S3 + A6 servo, PCB v10)
pio run   -e a6-ffb-v10-ck-at -t upload  # flash
pio device monitor -b 3000000            # 3,000,000 baud for S3 builds; 115200 for early boards
```

Full details in [`ESP32/README.md`](ESP32/README.md) and [`ESP32/BUILD.md`](ESP32/BUILD.md).
OTA updates are supported via Wi-Fi + a `.ffbota` container (see the OTA helper in `ESP32/sim/`).

Prebuilt binaries can be flashed with an [ESP web flasher](https://esp.huhn.me/):

| ESP32 | ESP32-S3 | File |
| :---: | :---: | :--- |
| 0x1000 | 0x0000 | bootloader.bin |
| 0x8000 | 0x8000 | partitions.bin |
| 0xe000 | 0xe000 | boot_app0.bin |
| 0x10000 | 0x10000 | firmware.bin |

## SimHub plugin

The SimHub plugin is the primary tuning surface. It communicates with the actuator over USB to
modify configuration (per-axis and per-function parameters) and to stream derived effects
(ABS oscillations, flight cues).

Its core is a **node-based graph engine**: simulator telemetry enters as signals, flows through DSP
nodes, and produces the actuator force. Graphs are edited visually, saved as reusable templates,
and support embedded subgraphs. Design docs: [`SimHubPlugin/Docs/FFB_Graph_Design.md`](SimHubPlugin/Docs/FFB_Graph_Design.md),
[`FFB_Graph_Signal_Catalog.md`](SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md), and the flight
architecture in [`Flight_FFB_Architecture.md`](SimHubPlugin/Docs/Flight_FFB_Architecture.md).

To install, copy the built `DiyFfbPlugin.dll` into your SimHub directory
(e.g. `C:/Program Files (x86)/SimHub`).

## Flight-sim data providers

- **MSFS (2020/2024)** — talks to MSFS **directly from the SimHub plugin** via an in-process,
  pure-C# SimConnect client over the SimConnect named pipe (no external EXE, no `SimConnect.dll`).
  See `SimHubPlugin/Msfs/`.
- **X-Plane** — [`XPlanePlugin/`](XPlanePlugin/) exposes DataRefs to the host (plus a `DataRefLogger`
  for capture/analysis). See [`SimHubPlugin/Docs/XPlane_FFB.md`](SimHubPlugin/Docs/XPlane_FFB.md).

## A6 servo tuning

You can find the initial provisioning procedure in [`ESP32/A6ServoSetup.md`](ESP32/A6ServoSetup.md); anything else is set by the ESP32-S3 via RS485. See [`ESP32/A6_SERVO.md`](ESP32/A6_SERVO.md) for the integration details.

# Credits

This project would not exist without the upstream work it forks. Original project and contributors:

- [ChrGri](https://github.com/ChrGri) — upstream maintainer; carried the project to the form this fork branched from.
- [tjfenwick](https://github.com/tjfenwick) — started the original project with the initial implementation.
- [tcfshcrw](https://github.com/tcfshcrw) — elevated the SimHub plugin, added many effects, hardware and Discord support.
- [MichaelJFr](https://github.com/MichaelJFr) — early refactoring and control-loop strategy discussions.
- [Ibakha](https://github.com/Ibakha) — Discord community.

Fork development and the flight-sim / graph-engine / multi-plugin extensions in this repository by
[CK-AT](https://github.com/CK-AT).

# License

This work is licensed under the
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-nc-sa/4.0/)
(**CC BY-NC-SA 4.0**), inherited from the upstream project.

[![CC BY-NC-SA 4.0](https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png)](http://creativecommons.org/licenses/by-nc-sa/4.0/)

**Copyright:**

- © ChrGri, tjfenwick and the original contributors — original work.
- © CK-AT — modifications and new subprojects in this fork.

**Attribution & modification notice (CC BY-NC-SA 4.0 §3):** This repository is a *modified* fork of
[ChrGri/DIY-Sim-Racing-FFB-Pedal](https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal). Changes have
been made from the original, including new firmware, host plugins, and the graph engine described above.

The **ShareAlike** term applies: because parts of this project (notably the ESP32 firmware) are
derivative works of the CC BY-NC-SA-licensed upstream, the combined work remains under
CC BY-NC-SA 4.0. The **NonCommercial** term prohibits commercial use — the license was chosen
upstream to prevent others from reselling the sources/binaries or mass-producing the design for
profit without contributing back.
