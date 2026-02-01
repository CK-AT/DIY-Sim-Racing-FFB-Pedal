# A6 Servo Error Logging Plan

Date: 2026-02-01
Status: Implemented

## Goal

Add runtime logging for A6 servo faults with human-readable descriptions to aid diagnostics.

## Problem Statement

- A6 servo fault codes (register 0x4100) are never monitored
- When faults occur, users have no visibility into what happened
- Debugging requires connecting the servo directly to read fault registers
- No way to see transient faults that clear automatically

## Scope

- Poll fault register in existing periodic task (100ms interval)
- Log fault with human-readable description when detected
- Log when fault clears
- Include lookup table for common fault codes
- **Out of scope**: Fault history registers (0x4101-0x4105), automatic fault reset

## Design

### Fault Code Format

A6 fault codes are stored as `(group << 8) | offset`:
- `Er47.1` → group=47 (0x2F), offset=1 → `0x2F01`
- `ALF2.0` → group=0xF2, offset=0 → `0xF200`

### Logging Output

```
A6Servo: FAULT Er47.1 Position deviation overflow (running) [resettable]
A6Servo: FAULT Er06.0 Runaway protection [non-resettable]
A6Servo: Fault cleared
A6Servo: FAULT 0x1234 (unknown)   # For unmapped codes
```

### Fault Classes

| Class | Code Range | Resettable | Examples |
|-------|------------|------------|----------|
| Class 1 | Er0x-Er2x | No | Hardware failures, encoder errors |
| Class 2 | Er4x-Er8x | Yes | Overload, over-temp, position deviation |
| Class 3 | ALFx | Yes | Overtravel, homing timeout, warnings |

### Fault Lookup Table

| Code | Description | Resettable |
|------|-------------|------------|
| 0x0600 | Er06.0 Runaway protection | No |
| 0x1400 | Er20.0 Encoder disconnected | No |
| 0x1401 | Er20.1 Encoder internal fault | No |
| 0x2800 | Er40.0 Drive overload | Yes |
| 0x2900 | Er41.0 Motor overload | Yes |
| 0x2901 | Er41.1 Motor over-temp (locked rotor) | Yes |
| 0x2A00 | Er42.0 IGBT temp too high | Yes |
| 0x2A02 | Er42.2 Heatsink temp too high | Yes |
| 0x2B00 | Er43.0 Main circuit overvoltage | Yes |
| 0x2B01 | Er43.1 Main circuit undervoltage | Yes |
| 0x2E00 | Er46.0 Motor overspeed | Yes |
| 0x2F00 | Er47.0 Position deviation overflow (static) | Yes |
| 0x2F01 | Er47.1 Position deviation overflow (running) | Yes |
| 0x3100 | Er49.0 Output phase loss | Yes |
| 0xF200 | ALF2.0 Forward overtravel | Yes |
| 0xF201 | ALF2.1 Reverse overtravel | Yes |
| 0xF400 | ALF4.0 Homing timeout | Yes |
| 0xFA00 | ALFA.0 Drive high temp warning | Yes |
| ... | (40+ codes total) | |

## File Structure

```
ESP32/
  include/
    A6Servo.h           # Add _last_fault_code member
  src/
    A6Servo.cpp         # Add get_fault_description(), fault polling
```

## Implementation Steps

1. Add `uint16_t _last_fault_code = 0;` member to A6Servo class in `A6Servo.h`
2. Add static `get_fault_description(uint16_t code)` function in `A6Servo.cpp`
3. Add fault polling logic in `periodic_task_func()` after existing code
4. Build and verify no errors

## Verification

1. Build: `pio run` in ESP32 directory
2. Flash: `pio run -t upload`
3. Monitor: `pio device monitor`
4. Trigger a fault (obstruct movement to cause Er47.1) - verify log shows fault description
5. Clear fault (power cycle or remove obstruction) - verify "Fault cleared" log appears
6. Verify no repeated log spam for same fault

## Reference

- Full fault code list: [A6-RS_MANUAL_REFERENCE.md](../../A6-RS_MANUAL_REFERENCE.md)
- Fault register: U41.00 (0x4100)
