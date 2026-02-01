# ESP32 Firmware Architecture

## Overview

The ESP32 firmware is a PlatformIO/Arduino project that implements a **force-feedback controller** for DIY sim racing peripherals. It supports multiple device types including pedals (brake, throttle, clutch), flight controls, and gear shifters.

The architecture follows a **physics-based force feedback model** where a virtual mass-spring-damper simulation determines motor position based on user input force (from a load cell).

## Directory Structure

```
ESP32/
├── src/                    # Source files
│   ├── Main.cpp           # Entry point, setup/loop, physics task
│   ├── Physics.cpp        # Physics simulation elements
│   ├── ConfigManager.cpp  # Configuration storage/loading
│   ├── CommManager.cpp    # Serial/CAN communication
│   ├── A6Servo.cpp        # A6-series servo motor driver
│   ├── LoadCell.cpp       # ADS1256 load cell ADC
│   └── *Function.cpp      # Device-specific functions
├── include/               # Header files
├── proto/                 # Protocol buffer definitions (via ../proto/)
├── platformio.ini         # Build configurations
└── test/                  # Unit tests (native platform)
```

## Core Concepts

### 1. Physics Simulation (`Physics.h/cpp`)

The firmware simulates a **virtual mass** that the user pushes against. This creates realistic force feedback.

```
                     ┌─────────────────┐
  User Force ──────► │   Sim Engine    │ ──────► Motor Position
  (Load Cell)        │ (mass-spring-   │         (Servo)
                     │  damper)        │
                     └─────────────────┘
```

**Key Classes:**

| Class | Description |
|-------|-------------|
| `Sim` | Main simulation engine - manages mass, position limits, integrates forces |
| `SimElement` | Base class for force contributions |
| `CompoundElement` | Container for multiple `SimElement`s |
| `Spring` | Linear spring: `F = -k * (x - offset)` |
| `Damper` | Velocity-dependent damping: accumulates `k_damp` |
| `Friction` | Static/kinetic friction model |
| `ForceMap` | Position-dependent force lookup table |
| `DampingMap` | Position-dependent damping lookup table |
| `Cam` | Sinusoidal detent effect |
| `Buffet` | Band-limited noise for flight stick buffet |

**Simulation Update (`Sim::update`):**
1. Calculate velocity from position delta
2. Collect forces from all `SimElement`s via `SimAccumulators`
3. Apply damping (velocity-proportional, clamped for stability)
4. Apply friction (static stiction, kinetic sliding)
5. Integrate acceleration → velocity → position (Verlet-like)
6. Clamp position to limits

### 2. Function Types (`IFunction.h`)

Each device type has a corresponding "Function" that configures the physics simulation:

| Function | Description |
|----------|-------------|
| `AutomotivePedalFunction` | Brake/throttle/clutch with force curve, ABS, damping |
| `FlightPedalsFunction` | Centered flight rudder pedals |
| `FlightStickFunction` | Pitch/roll/collective axes |
| `ShifterFunction` | H-pattern/sequential shifter with gate constraints |

Functions inherit from `IFunction` (which extends `CompoundElement`) and provide:
- Position limits (`get_x_contact_point_min/max`)
- FFB action handlers (game telemetry effects)
- Configuration updates from protobuf messages

### 3. Configuration System (`ConfigManager.h`)

**Two-layer configuration:**

1. **AxisConfig** - Hardware-specific:
   - Kinematics (pedal geometry, linkage)
   - Motor settings (steps/mm, direction)
   - Load cell calibration
   - Kalman filter parameters
   - Oscillation guard tuning

2. **FunctionConfig** - Function-specific:
   - Force curves and damping
   - Position limits
   - Effect configurations (ABS, vibration, etc.)
   - Controller output mapping

Configurations are stored in ESP32 flash (`Preferences`) and communicated via protobuf.

### 4. Communication (`CommManager.h`)

**Multi-channel architecture supporting:**

- **Serial** - USB communication with SimHub plugin (host)
- **CAN Bus** - Inter-axis communication for multi-axis setups
- **ESP-NOW** - Wireless (legacy, being phased out)

**Roles:**
- **Axis** - A single force-feedback axis (pedal, stick, shifter axis)
- **Gateway** - Aggregates multiple axes, provides USB/HID output

**Protocol:** Nanopb-encoded protobuf messages (see `proto/diy_ffb_protocol.proto`)

### 5. Motor Control (`Servo.h`, `A6Servo.h`)

Supports A6-series industrial servo drives via:
- **Step/Direction** pulses (FastNonAccelStepper library)
- **Modbus RTU** for configuration and status

Key features:
- Sensorless homing (torque-limited)
- Torque limiting for safety
- Position tracking

## Task Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Core 0 (Arduino)                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │   loop()    │  │  CommMgr    │  │   Servo     │     │
│  │  (1Hz LED)  │  │  periodic   │  │  periodic   │     │
│  └─────────────┘  └─────────────┘  └─────────────┘     │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│                    Core 1 (Physics)                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │              physics_task_func                   │   │
│  │  ISR-triggered @ 1kHz (ADC ready)               │   │
│  │  ┌─────────────────────────────────────────┐    │   │
│  │  │ 1. Read load cell (ADS1256)             │    │   │
│  │  │ 2. Kalman filter                        │    │   │
│  │  │ 3. Static balance correction            │    │   │
│  │  │ 4. CAN message processing               │    │   │
│  │  │ 5. Physics simulation (Sim::update)     │    │   │
│  │  │ 6. Send motor command (servo->move_to)  │    │   │
│  │  │ 7. Broadcast force/position via CAN     │    │   │
│  │  └─────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

## PCB Versions

The firmware supports multiple hardware revisions via `PCB_VERSION` define:

| Version | Description |
|---------|-------------|
| 3 | Dev PCB for regular ESP32 |
| 6 | ESP32-S3 with USB HID |
| 7 | Gilphilbert PCBA |
| 12 | CK-AT prototype (ESP32-S3, A6 servo) |
| **13** | CK-AT HW V1.0 (current default) |

## Build Environments

Default: `a6-ffb-v10-ck-at` (ESP32-S3 with A6 servo, 8MB flash)

Key environments:
- `native` - For unit testing on host machine
- `esp32s3usbotg` - Generic ESP32-S3 with USB
- `a6-ffb-v10-ck-at` - Production A6-series pedal controller

## Protocol Overview

Messages are defined in `proto/diy_ffb_protocol.proto`:

| Message | Direction | Purpose |
|---------|-----------|---------|
| `AxisConfig` | Host→Axis | Hardware configuration |
| `FunctionConfig` | Host→Axis | Function parameters |
| `FFBAction` | Host→Axis | Game telemetry (ABS, G-force, etc.) |
| `AxisState` | Axis→Host | Position/force feedback |
| `GatewayState` | Gateway→Host | Multi-axis presence |
| `AxisAction` | Host→Axis | Commands (restart, homing, etc.) |

## Key Algorithms

### Force Curve (Automotive Pedal)

Spline-based force mapping with:
- 6 configurable control points (0%, 20%, 40%, 60%, 80%, 100% travel)
- Cubic spline interpolation for smooth force transitions
- Direction flag (subtractive for brake pedal feel)

### Oscillation Guard

Detects and suppresses mechanical oscillations:
1. Track zero-crossings of velocity
2. Measure half-period timing
3. If frequency and amplitude exceed thresholds → engage damping
4. Smooth ramp-in/hold/ramp-out to avoid jerks

### Static Balance Compensation

Compensates for gravity-induced force offset across travel:
1. Calibration: sweep position, record force offset
2. Fit polynomial to offset curve
3. Runtime: subtract polynomial(x) from measured force

### Shifter Gate Logic

H-pattern shifter with:
- Corridor segments (lanes) with centering springs
- Detent wells at gear positions
- Lane hysteresis to prevent jitter at intersections
- Dynamic soft limits based on current position
