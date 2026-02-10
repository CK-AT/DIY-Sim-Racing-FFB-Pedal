# Plan 33: Consolidate FlightStick Config Types

## Goal

Replace three identical protobuf messages (`FlightStickPitchConfig`, `FlightStickRollConfig`,
`FlightStickCollectiveConfig`) with a single `FlightStickConfig`. Differentiate by `FunctionID`
alone (already in `FunctionBase`). Migrate stored baselines and file imports.

## Motivation

All three messages have identical fields (`pos_min`, `pos_max`, `damping`,
`centering_spring_const`). The codebase already papers over this with
`IFlightStickSubConfig` (C#) and `FlightStickConfigCommon` (ESP32). This cleanup
eliminates ~200 lines of dispatch boilerplate.

**What stays**: The 3 `FunctionID` values (Pitch=5, Roll=6, Collective=8) remain.
Signal groups (`FlightStickPitch.SpringGain` etc.) remain. Only the **config message type** unifies.

---

## Phase 1: Protobuf Definition

**File: `proto/diy_ffb_protocol.proto`**
- Delete `FlightStickRollConfig` and `FlightStickCollectiveConfig` messages
- Rename `FlightStickPitchConfig` to `FlightStickConfig`
- In `FunctionConfig.oneof specific`: replace 3 fields with `FlightStickConfig flight_stick = 7`
- Reserve field numbers 8, 11: `reserved 8, 11;`
- Note: field 7 reuse is binary-compatible since message structure is identical

**File: `proto/diy_ffb_protocol.options`** (nanopb)
- Replace 3 lines (`FlightStickPitchConfig.pos_*`, `FlightStickRollConfig.pos_*`,
  `FlightStickCollectiveConfig.pos_*`) with single `FlightStickConfig.pos_* int_size:IS_16`

**Regenerate code**:
- C#: Run `protoc` to regenerate `DiyFfbProtocol.cs`
- ESP32: nanopb auto-regenerates via PlatformIO build

### Generated API changes (C#)
- `FunctionConfig.FlightStickPitch` / `.FlightStickRoll` / `.FlightStickCollective` → `FunctionConfig.FlightStick`
- `SpecificOneofCase.FlightStickPitch` etc. → `SpecificOneofCase.FlightStick`
- Types `FlightStickPitchConfig`, `FlightStickRollConfig`, `FlightStickCollectiveConfig` → `FlightStickConfig`

---

## Phase 2: ESP32 Firmware

**File: `ESP32/include/FlightStickFunction.h`**
- Replace 3 `update_config()` overloads with single `void update_config(const FlightStickConfig &config)`
- Remove `FlightStickConfigCommon` struct — `FlightStickConfig` IS the common struct now

**File: `ESP32/src/FlightStickFunction.cpp`**
- Replace 3 overloads + `update_config_common` with single method that reads directly
  from `FlightStickConfig` fields (same field names as `FlightStickConfigCommon`)
- Rename `update_config_common` body → `update_config` body

**File: `ESP32/src/Main.cpp`**
- Collapse 3 switch cases into 1:
  ```cpp
  case FunctionConfig_flight_stick_tag:
      // Dispatch to per-FunctionID instance
      switch (function_cfg->base.function_id) {
          case FunctionID_FUNCTION_ID_FLIGHT_STICK_PITCH:
              flight_stick_pitch_function.update_config(function_cfg->specific.flight_stick);
              active_function = &flight_stick_pitch_function; break;
          case FunctionID_FUNCTION_ID_FLIGHT_STICK_ROLL:
              flight_stick_roll_function.update_config(function_cfg->specific.flight_stick);
              active_function = &flight_stick_roll_function; break;
          case FunctionID_FUNCTION_ID_FLIGHT_STICK_COLLECTIVE:
              flight_stick_collective_function.update_config(function_cfg->specific.flight_stick);
              active_function = &flight_stick_collective_function; break;
      }
      break;
  ```

---

## Phase 3: C# Plugin — Core Config Layer

### 3a: Delete `IFlightStickSubConfig`
- **Delete file**: `SimHubPlugin/IFlightStickSubConfig.cs`
- No longer needed — single concrete type everywhere

### 3b: Simplify `FlightStickProcessor.cs`
- `ReconcileDerivedFields`: No switch needed, just read `config.FlightStick?.PosMin/PosMax`
- `ApplyOverrides`: No switch needed, just `if (merged.FlightStick == null) merged.FlightStick = new FlightStickConfig();`
  then apply to `merged.FlightStick` directly
- Delete 3 private `ApplyTo*` methods → inline into single apply block
- ~119 lines → ~35 lines

### 3c: Simplify `ConfigLayerProvider.cs`
- `GetFieldValueFromConfig`: Replace `config.FlightStickPitch` → `config.FlightStick`
  (also fixes existing bug where it always read Pitch regardless of FunctionId)
- `WriteFieldToFunctionConfig`: Replace dispatch helpers with direct access
- **Delete**: `GetFlightStickSubConfig()` and `EnsureFlightStickSubConfig()` helpers
- All flight stick cases simplify to `config.FlightStick` / `new FlightStickConfig()`

### 3d: Update `ConfigMerger.cs`
- `ReconcileDerivedFields` switch: collapse 3 cases → 1 that calls `FlightStickProcessor`

### 3e: Update `TieredConfigOrchestrator.cs`
- `IsFlightStick()` helper: references FunctionID enum values (no change needed)
- Any direct references to config types → `FlightStickConfig`

---

## Phase 4: C# Plugin — UI Controls

### 4a: Simplify `FlightStickConfigControl.xaml.cs`
- Replace 3 fields (`pitch_config`, `roll_config`, `collective_config`) with single
  `FlightStickConfig stick_config`
- Delete `FlightStickMode` enum and `GetMode()` method
- Delete `GetActiveSubConfig()`, `GetSubConfig()`, `EnsureConfigInitialized()` —
  all replaced by direct `function_config.FlightStick` access
- Replace 3 `GetDefault*Config()` methods with single `GetDefaultConfig()` returning `FlightStickConfig`
- Update `SwitchFunction()`: `EnsureConfigInitialized()` → simple null-check on `function_config.FlightStick`
- Update `OnBadgeOverrideCleared()`: read from `config.FlightStick` directly
- ~33 references in this file

### 4b: Update `FunctionConfigControl.xaml.cs`
- `GetDefaultConfig()` switch cases: all 3 flight stick cases set `new_config.FlightStick = FlightStickConfigControl.GetDefaultConfig()`
- `SwitchFunction()`: references FunctionID values (no change needed)

### 4c: Naming collision note
- XAML element `x:Name="FlightStickConfig"` (type `FlightStickConfigControl`) in
  `FunctionConfigControl.xaml` will shadow the proto class name `FlightStickConfig`
- Resolution: rename XAML element to `x:Name="uc_flight_stick"` to match `uc_` prefix
  convention, or use `global::FlightStickConfig` where the proto type is needed

---

## Phase 5: Baseline & Import Migration

### 5a: Baseline JSON migration
Stored baselines use protobuf JSON format. The oneof discriminator is the field name:
- Old: `"flightStickPitch": {...}` or `"flightStickRoll": {...}` or `"flightStickCollective": {...}`
- New: `"flightStick": {...}`

**Add `MigrateFlightStickBaselines()` in `TieredConfigOrchestrator`**:
- Called from `InitializeManagerFromSettings()` before baseline deserialization
- For function IDs 5, 6, 8: string-replace old field names → `"flightStick"` in stored JSON
- Re-persist updated JSON strings
- Run once on first load after upgrade; idempotent (no-op if already migrated)

### 5b: File import migration
**File: `DiyFfbPluginUI.xaml.cs`** — config import code
- Before parsing `ConfigItemsList` JSON, apply same string replacements
- Handles old export files that still use the 3-field format

---

## Phase 6: Tests

### 6a: Update `FlightStickProcessorTests.cs`
- Replace all `FlightStickPitchConfig`/`RollConfig`/`CollectiveConfig` with `FlightStickConfig`
- Replace `config.FlightStickPitch`/`.FlightStickRoll`/`.FlightStickCollective` with `config.FlightStick`
- Helper methods collapse from 3 to 1

### 6b: Add baseline migration tests
- Test migration of each old field name → new
- Test idempotency (migrating already-migrated JSON is no-op)
- Test file import with old-format JSON

### 6c: Verify existing tests pass
- Build with MSBuild
- Run all 240+ existing unit tests

---

## Files Modified

| File | Change |
|------|--------|
| `proto/diy_ffb_protocol.proto` | Consolidate 3 messages → 1, update oneof |
| `proto/diy_ffb_protocol.options` | Consolidate 3 nanopb lines → 1 |
| `SimHubPlugin/DiyFfbProtocol.cs` | Regenerated (protoc) |
| `ESP32/include/FlightStickFunction.h` | 3 overloads → 1, drop CommonConfig |
| `ESP32/src/FlightStickFunction.cpp` | 3 overloads → 1 method |
| `ESP32/src/Main.cpp` | 3 switch cases → 1 with inner FunctionID dispatch |
| `SimHubPlugin/IFlightStickSubConfig.cs` | **DELETE** |
| `SimHubPlugin/TieredConfig/FlightStickProcessor.cs` | Remove all switch/dispatch |
| `SimHubPlugin/TieredConfig/ConfigLayerProvider.cs` | Remove helpers, direct access |
| `SimHubPlugin/TieredConfig/ConfigMerger.cs` | Collapse 3 cases → 1 |
| `SimHubPlugin/TieredConfig/TieredConfigOrchestrator.cs` | Add migration, minor type updates |
| `SimHubPlugin/FlightStickConfigControl.xaml.cs` | Remove mode dispatch, single config field |
| `SimHubPlugin/FunctionConfigControl.xaml.cs` | Update default config creation |
| `SimHubPlugin/FunctionConfigControl.xaml` | Rename x:Name to avoid collision |
| `SimHubPlugin/DiyFfbPlugin.cs` | Minor: FunctionID string helper (no config type refs) |
| `SimHubPlugin/DiyFfbPluginUI.xaml.cs` | Import migration |
| `SimHubPlugin/TieredConfigTests/FlightStickProcessorTests.cs` | Update all config types |

---

## Verification

1. `MSBuild` — clean build of SimHubPlugin with no errors
2. Run all unit tests (240+ existing + new migration tests)
3. Manual: launch SimHub, verify flight stick config tab loads correctly for all 3 modes
4. Manual: verify existing baselines load after migration
5. ESP32: `pio build` succeeds for all targets
