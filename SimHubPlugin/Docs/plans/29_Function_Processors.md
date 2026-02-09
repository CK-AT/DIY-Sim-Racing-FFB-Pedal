# Function Processors

**Status: IN PROGRESS** — AutomotivePedal first

## Problem

Function-type-specific config logic is duplicated between `ConfigMerger` (backend
reconciliation during merge) and UI controls (real-time updates during editing).
The same derived-field rules exist in two places and can drift:

| Function Type | ConfigMerger method | UI control method |
|---|---|---|
| AutomotivePedal | `ReconcileAutomotivePedal` | `OnRangeSettingsChanged`, `cb_controller_output_mode_SelectionChanged` |
| FlightPedals | `ReconcileFlightPedals` | `SwitchFunction` |
| FlightStick | `ReconcileFlightStick` | `SwitchFunction` + mode accessors |
| Shifter | `ReconcileShifter` | `UpdateOutputRange` |

Override application logic (`ApplyAutomotivePedalOverrides`, etc.) is also
function-type-specific and lives inline in `ConfigMerger`.

## Solution

Extract per-function-type **processor classes** — pure static classes (no WPF
dependency) that are the single source of truth for:

1. **Derived field reconciliation** — computing posIdle/posEnd/outputMin/outputMax
   from force curve, motion range, or shifter geometry
2. **Override application** — merging deltas into configs (damper field-by-field,
   force curve full replacement, etc.)

Both `ConfigMerger` and UI controls call the same processor methods.

## Design

- **Static classes** in `TieredConfig/` alongside ConfigMerger (matches existing
  pattern: ConfigMerger, ConfigComparer, FieldRouter are all static)
- **No interfaces** — the switch dispatch in `ReconcileDerivedFields` stays; each
  arm delegates to the corresponding processor
- **Scope**: Reconciliation + override application only. Default config creation
  and UI-specific logic (labels, plots, `is_updating` guards) stay in controls.

## Phase 1: AutomotivePedalProcessor

### New file: `TieredConfig/AutomotivePedalProcessor.cs`

**`ReconcileDerivedFields(FunctionConfig config)`**

Moved from `ConfigMerger.ReconcileAutomotivePedal`:

- `PosIdle` = `ForceCurveConfig.PosMin`
- `PosEnd` = `ForceCurveConfig.PosMax`
- `OutputMin`/`OutputMax` depend on `OutputMode`:
  - Force → `FMin`/`FMax`
  - Travel → `PosMin`/`PosMax`

**`ApplyOverrides(AutomotivePedalConfig config, FunctionConfigOverrides delta)`**

Moved from `ConfigMerger.ApplyAutomotivePedalOverrides`:

- DamperConfig: field-by-field merge (PositiveFactor, NegativeFactor)
- ForceCurve: full replacement (clone)

### ConfigMerger changes

- `ReconcileDerivedFields` switch arm delegates to
  `AutomotivePedalProcessor.ReconcileDerivedFields(config)`
- `MergeFunctionConfig` delegates to
  `AutomotivePedalProcessor.ApplyOverrides(merged.AutomotivePedal, delta)`
- Private methods `ReconcileAutomotivePedal` and
  `ApplyAutomotivePedalOverrides` deleted

### AutomotivePedalConfigControl changes

- `OnRangeSettingsChanged`: Replace inline OutputMin/Max + PosIdle/PosEnd
  computation with `AutomotivePedalProcessor.ReconcileDerivedFields(function_config)`
- `cb_controller_output_mode_SelectionChanged`: After setting OutputMode, call
  `AutomotivePedalProcessor.ReconcileDerivedFields(function_config)` instead of
  inline OutputMin/Max assignment

### Tests: `TieredConfigTests/AutomotivePedalProcessorTests.cs`

11 test cases covering reconciliation (force mode, travel mode, null guards) and
override application (field-by-field damper, force curve replacement, null/empty
delta, DamperConfig auto-creation).

## Files touched (Phase 1)

| File | Change |
|---|---|
| `TieredConfig/AutomotivePedalProcessor.cs` | New — processor class |
| `TieredConfig/ConfigMerger.cs` | Delete 2 private methods, delegate to processor |
| `AutomotivePedalConfigControl.xaml.cs` | Replace inline derived-field logic with processor calls |
| `DiyFfbPlugin.csproj` | Add Compile Include |
| `TieredConfigTests/AutomotivePedalProcessorTests.cs` | New — 11 test cases |
| `TieredConfigTests/TieredConfigTests.csproj` | Add Compile Include |
| `TieredConfigTests/Program.cs` | Register test suite |

## Phase 2: FlightPedalsProcessor

### New file: `TieredConfig/FlightPedalsProcessor.cs`

**`ReconcileDerivedFields(FunctionConfig config)`**

Moved from `ConfigMerger.ReconcileFlightPedals`:
- `OutputMin` = `PosNearLim`
- `OutputMax` = `PosFarLim`

**`ApplyOverrides(FlightPedalsConfig config, AuxFunctionConfig auxConfig, FunctionConfigOverrides delta)`**

Moved from `ConfigMerger.ApplyFlightPedalsOverrides`:
- MotionRange: field-by-field (NearLim, FarLim)
- Damping: scalar replacement
- CenteringSpringConst: scalar replacement
- RudderBrakeForceRange: field-by-field on AuxFunctionConfig (FMin, FMax)

### Files touched (Phase 2)

| File | Change |
|---|---|
| `TieredConfig/FlightPedalsProcessor.cs` | New — processor class |
| `TieredConfig/ConfigMerger.cs` | Delete 2 private methods, delegate to processor |
| `FlightPedalsConfigControl.xaml.cs` | Replace inline OutputMin/Max with processor calls |
| `DiyFfbPlugin.csproj` | Add Compile Include |
| `TieredConfigTests/FlightPedalsProcessorTests.cs` | New — 14 test cases |
| `TieredConfigTests/TieredConfigTests.csproj` | Add Compile Include |
| `TieredConfigTests/Program.cs` | Register test suite |

## Future phases

| Phase | Processor | Source methods |
|---|---|---|
| 3 | `FlightStickProcessor` | `ReconcileFlightStick`, `ApplyFlightStickOverrides` |
| 4 | `ShifterProcessor` | `ReconcileShifter`, inline shifter override in `MergeFunctionConfig` |

After all 4 phases, `ConfigMerger` contains only the generic merge orchestration
(clone, apply scalars, delegate to processors, reconcile) with zero
function-type-specific logic inline.
