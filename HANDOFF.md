# Session Handoff

Date: 2026-02-01
Last commit: `acddc868` — Fix Vehicle tab params not syncing on profile load

## What Was Done This Session

### Vehicle Tab Param Sync Bug Fix (Committed)

Fixed bug where Vehicle tab graph parameters showed stale values after profile load, while Function tab showed correct values.

**Root Cause**: `ResolveActiveGraph()` fired `ActiveGraphChanged` event before `graphParams` was updated, so the Vehicle tab's `RefreshVehicleParams()` read stale values from `graphParams` dictionary.

**Fix**: Added `BuildGraphParams()` call immediately after every `ResolveActiveGraph()` call:

- `HandleGameChange()` — when game changes (e.g., X-Plane → MSFS)
- `HandleAircraftChange()` — when vehicle changes
- `SetVehicleGraphPath()` — when graph path is changed

**Files Changed**: `SimHubPlugin/DiyFfbPlugin.cs`

## Previous Session Work

### Progressive Spring Plan (Planned)

Created comprehensive plan for adding non-linear spring behavior to flight controls at [docs/plans/01_progressive-spring.md](docs/plans/01_progressive-spring.md).

**Features**:

- Spring exponent parameter: `F = -k * sign(x) * |x|^n`
- n=1.0 linear (default), n>1 progressive, n<1 degressive
- Graph output signals for all flight functions
- CAN transport via renamed FlightFfbPayload1/2 structs
- `fast_powf` approximation for performance (~10 cycles vs ~100)

**Status**: Plan complete, ready for implementation

### A6 Servo Fault Monitoring (Implemented)

Implemented servo fault logging and auto-reset per plan at [ESP32/docs/plans/01_a6-servo-error-logging.md](ESP32/docs/plans/01_a6-servo-error-logging.md).

**Features**:

- Polls fault register (0x4100) every 100ms
- Logs faults with human-readable descriptions and resettable status
- Auto-resets resettable faults when `home()` is called (writes F31.00=1)

**Status**: Implemented, build verified (+1.6KB flash, +0 RAM)

## Build & Test Commands

```bash
# SimHub Plugin (use MSBuild, not dotnet)
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Debug /v:minimal /nologo

# ESP32 Firmware
PIO="C:\Users\Christian\.platformio\penv\Scripts\pio.exe"
cd ESP32
"$PIO" run                    # Build default env
"$PIO" run -t upload          # Flash to device
"$PIO" test -e native         # Run unit tests
```

## Next Steps

1. Test Vehicle tab param sync fix with profile switching
2. Implement progressive spring feature per plan
3. Flash firmware and test fault logging

## Related Documents

- [docs/plans/01_progressive-spring.md](docs/plans/01_progressive-spring.md) — Progressive spring feature plan
- [ESP32/docs/plans/01_a6-servo-error-logging.md](ESP32/docs/plans/01_a6-servo-error-logging.md) — Error logging plan
- [SimHubPlugin/Docs/Plugin_Design.md](SimHubPlugin/Docs/Plugin_Design.md) — Plugin architecture and build instructions
