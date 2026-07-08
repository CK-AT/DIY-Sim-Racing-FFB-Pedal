# A6 Servo Integration

Technical documentation for the A6-RS servo drive integration.

## Overview

The A6Servo class controls an A6-RS industrial servo drive via Modbus RTU over RS485. It combines:
- **Modbus RTU** for configuration, homing commands, and status reads
- **Step/Dir pulses** for real-time position commands (via FastNonAccelStepper)

## Files

| File | Purpose |
|------|---------|
| [include/A6Servo.h](include/A6Servo.h) | Class definition, Modbus helpers |
| [src/A6Servo.cpp](src/A6Servo.cpp) | Implementation |
| [include/Servo.h](include/Servo.h) | Abstract base class |
| [A6ServoSetup.md](A6ServoSetup.md) | Initial commissioning steps |

## Class Hierarchy

```
Servo (abstract)
  ├─ State: Disabled, Enabled, Homing
  ├─ HomingState: HomeUnknown, Pending, Homed, LockedIn, LockingError, LockingBlocked
  └─ A6Servo (concrete)
       ├─ ModbusClientRTU* _modbus
       └─ FastNonAccelStepper* _stepper_engine
```

## Communication

### Modbus RTU

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Format | 8N1 |
| Slave ID | 1 |
| Timeout | 10ms |
| Retries | 3 |

### Step/Dir

Position commands sent as stepper pulses at up to 2 MHz. The A6-RS follows these pulses in position mode.

## Key Registers

### Required Settings (checked at startup)

| Register | Name | Required | Purpose |
|----------|------|----------|---------|
| 0x0A05 | EEPROM Save | 0 | Don't persist RS485 writes |
| 0x0A06 | Word Order | 1 | High word first for 32-bit |
| 0x0022 | Pulse Channel | 1 | High-speed input |

### Configuration Registers

| Register | Name | Notes |
|----------|------|-------|
| 0x0122 | Position LPF | 50 = 5.0ms filter |
| 0x0304 | Gear Denom | steps_per_mm × mm_per_rev |
| 0x0306 | Gear Numer | 131072 (encoder CPR) |
| 0x0343 | Neg Torque Limit | 0.1%/LSB |
| 0x0344 | Pos Torque Limit | 0.1%/LSB |
| 0x0411 | Enable | 1=on, 0=off |
| 0x0600 | Deviation Threshold | Relaxed: 1000000, Locked: 30000 |
| 0x0607 | Limit Mode | 2 = active after homing |
| 0x0608 | Pos Limit | counts (32-bit) |
| 0x060A | Neg Limit | counts (32-bit) |

### Homing Registers

| Register | Name | Notes |
|----------|------|-------|
| 0x1000 | Homing Command | 1=start, 0=stop |
| 0x1001 | Homing Mode | -1=reverse, -2=forward to mech limit |
| 0x1002 | Homing Speed | RPM — manual/panel parameter, not configured by firmware; homing speed is applied via the stepper (set_speed) |
| 0x1030 | Homing Torque | 0.1%/LSB |

### Status Registers (read-only)

| Register | Name | Notes |
|----------|------|-------|
| 0x4001 | Actual Speed | RPM (int16) |
| 0x4016 | Actual Position | counts (int32) |
| 0x4100 | Fault Code | 0 = no fault, else ErXX.Y or ALFx.Y |

### Fault Control Registers

| Register | Name | Notes |
|----------|------|-------|
| 0x3E00 | Fault Reset | Write 1 to clear resettable faults |

## Operating Parameters

| Parameter | Open Loop | Locked In |
|-----------|-----------|-----------|
| Torque Limit | 10% | 300% |
| Speed | 200 RPM | 6000 RPM |
| Deviation Threshold | 1,000,000 | 30,000 |

## State Machine

```
                    ┌──────────────────┐
                    │    Disabled      │
                    └────────┬─────────┘
                             │ enable()
                             ▼
                    ┌──────────────────┐
       ┌───────────▶│    Enabled       │◀────────────┐
       │            │ (HomeUnknown)    │             │
       │            └────────┬─────────┘             │
       │                     │ home() or autohome    │
       │                     ▼                       │
       │            ┌──────────────────┐             │
       │            │    Homing        │             │
       │            │   (Pending)      │             │
       │            └────────┬─────────┘             │
       │                     │ do_homing()           │
       │                     ▼                       │
       │  fail      ┌──────────────────┐             │
       ├────────────│     Homed        │             │
       │            └────────┬─────────┘             │
       │                     │ lock_onto_curr_pos()  │
       │                     ▼                       │
       │  fail      ┌──────────────────┐   success   │
       └────────────│    LockedIn      │─────────────┘
                    │ (normal operation)│
                    └──────────────────┘
```

`pause()` moves `LockedIn` → `LockingBlocked` (config/OTA/calibration guards);
`resume()` moves `LockingBlocked` → `Homed`, from which the background task
re-runs `lock_onto_curr_pos()` to return to `LockedIn`.

## Homing Sequence

The homing process uses mode 35 plus stepper verification for reliability.

`home()` requests homing: if a fault is currently latched (`_last_fault_code != 0`),
it first resets any resettable faults (F31.00 = 1), then sets the homing state to
`Pending`. The background task then runs `do_homing()`, which performs the phases
below. `do_homing()` itself does not reset faults — the autohome/`Pending` path
reaches it without a fault reset.

**Phase 1: Set Home Position**
1. Set homing mode to 35 (current position as home)
2. Enable homing command (0x1000 = 1)
3. This avoids Er47.1 that can occur with mechanical limit + Z pulse modes

**Phase 2: Stepper Endstop Detection**
1. Command stepper to run toward first endstop (based on `_homing_direction`)
2. Wait for servo to stall (speed ~0 for 10 readings)
3. Record first endstop position
4. Reverse direction, run to second endstop
5. Validate travel > 20mm
6. Set position limits with 2500-count margins

**Phase 3: Lock-In**
1. Command position repeatedly until servo follows
2. Verify position error < 10 counts
3. Tighten deviation threshold (0x0600 = 30000)
4. Increase torque limit to 300%
5. Increase speed limit to 6000 RPM

## Position Command Flow

```
Physics Task (100Hz)
    │
    ▼
move_to(position_mm)
    │
    ├─ Constrain to [0, max_pos_mm]
    ├─ Store as _curr_pos
    ├─ Check state == Enabled && homing_state == LockedIn
    │
    ▼
logical_to_counts(_curr_pos)
    │
    ├─ Convert mm to counts
    ├─ Apply _reverse_motion if set
    │
    ▼
_stepper_engine->move_to(counts)
    │
    ▼
Step/Dir pulses to A6-RS
```

## Safety Features

### Hardware Protection
- Mechanical endstops detected during homing
- Position limits stored in servo registers
- 2500-count margin from physical endstops

### Software Protection
- Position deviation threshold prevents servo runaway
- Torque limits cap motor output
- State machine prevents commands in wrong state
- 3-retry Modbus communication with error logging

### Operational Guards
- Servo disabled during load cell calibration
- Servo paused during config updates
- Servo paused during OTA updates
- `move_to()` only executes when LockedIn

## Background Task

A FreeRTOS task runs every 100ms on Core 0:

```cpp
void periodic_task_func(void) {
    // Handle pause timeout
    if (ti_pause_end && esp_timer_get_time() > ti_pause_end) {
        resume();
    }
    // Trigger homing if pending
    if (_homing_state == Pending && _state == Enabled) {
        do_homing();
    }
    // Trigger lock-in after homing
    if (_homing_state == Homed && _state == Enabled) {
        lock_onto_curr_pos();
    }
    // Poll fault register and log changes
    poll_fault_register();
}
```

## Fault Monitoring

The servo's fault register (0x4100) is polled every 100ms. When a fault is detected:

1. Fault code is looked up in a table of 25+ known codes
2. Human-readable description is logged with resettability status
3. When fault clears, "Fault cleared" is logged

**Example output:**
```
A6Servo: FAULT Er47.1 Position deviation overflow (running) [resettable]
A6Servo: Fault cleared
```

### Fault Classes

| Class | Code Range | Resettable | Examples |
|-------|------------|------------|----------|
| Class 1 | Er0x-Er2x | No | Hardware failures, encoder errors |
| Class 2 | Er4x-Er8x | Yes | Overload, over-temp, position deviation |
| Class 3 | ALFx | Yes | Overtravel, homing timeout, warnings |

### Auto-Reset on Homing

When `home()` is called and a fault exists, resettable faults are automatically cleared by writing F31.00=1 before the homing sequence begins. This allows recovery from Er47.1 (position deviation) without manual intervention.

See [A6-RS_MANUAL_REFERENCE.md](A6-RS_MANUAL_REFERENCE.md) for complete fault code reference.

## Initial Commissioning

Before first use, configure the A6-RS drive panel:

1. **Commissioning** — A6-RS manual chapter 5.3
2. **Inertia auto-tuning** — A6-RS manual chapter 6.2
3. **C00.05** = 20 (stiffness level)
4. **C00.02** = 0 (disable simple gear ratio)
5. **C0A.05** = 0 (don't save RS485 writes to EEPROM)
6. **C0A.06** = 1 (high word first)
7. **C00.22** = 1 (high-speed pulse channel) — requires power cycle
8. Connect controller board

## Homing Modes Reference

From A6-RS manual table 4-14 (register 0x1001):

| Mode | Description |
|------|-------------|
| -2 | Forward to mechanical limit, then Z pulse |
| -1 | Reverse to mechanical limit, then Z pulse |
| 1-8 | Various limit switch + Z pulse combinations |
| 17-30 | Same as 1-14 but stop at limit (no Z search) |
| 33-34 | Nearest Z pulse (forward/reverse) |
| 35 | Use current position as home |

This implementation uses **mode 35** (current position as home) to avoid Er47.1 during Z pulse search, then performs stepper-based endstop detection.

## Troubleshooting

### Startup Failures

**"Register 0x0A05 is not 0"**
- Set C0A.05 = 0 on drive panel

**"Register 0x0A06 is not 1"**
- Set C0A.06 = 1 on drive panel

**"Register 0x0022 is not 1"**
- Set C00.22 = 1, then power cycle drive

### Homing Failures

**"Sled did not move far enough"**
- Check mechanical binding
- Verify endstops are reachable
- Check torque limit (may be too low)

**"Failed to lock in"**
- Target position may be out of bounds
- Servo may not be following commands
- Check Modbus communication

### Runtime Issues

**Servo not responding**
- Check RS485 wiring (TX, RX, DE pins)
- Verify baud rate matches (115200)
- Check slave ID = 1

**Position drift**
- Increase deviation threshold temporarily
- Check for mechanical binding
- Verify encoder counts match configuration
