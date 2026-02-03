# Session Handoff

Date: 2026-02-03
Branch: `ck_tiered_config`
Last commit: `f7d68181` — Add function selector UI for axis parameter overrides

## Current State

**Phase 5: AxisConfig Override UI** — COMMITTED
**Unit Tests** — ADDED (uncommitted)

## What Was Implemented

Added function selector dropdown to the Axis tab allowing users to edit axis parameters (kinematics, static balance) either for axis base config OR as per-function override.

### Files Modified

**DiyFfbPlugin.cs** — Added API methods:
- `FunctionAxisLink` class for function-axis linking info
- `GetFunctionsLinkingToAxis(int axisId)` — Returns functions linking to an axis
- `HasAxisParameterOverride()` / `GetAxisParameterOverride()` — Query overrides
- `SetAxisParameterOverride()` / `UpdateAxisParameterOverride()` — Set overrides
- `ClearAxisParameterOverride()` / `ClearAllAxisParameterOverrides()` — Remove overrides

**AxisConfigControl.xaml** — Added UI elements:
- `BooleanToVisibilityConverter` resource
- `FunctionSelectorPanel` with ComboBox and Clear Override button
- `[F]` badge indicator for functions with overrides

**AxisConfigControl.xaml.cs** — Added code-behind:
- `FunctionSelectorItem` class, `AxisEditingMode` enum
- `RefreshFunctionSelector()` — Populate dropdown with linked functions
- Mode switching and override save logic

### UI Behavior

1. Function Selector appears on Axis tab when functions link to the axis
2. "Axis N (base)" edits the hardware axis config (normal behavior)
3. Function name options enable override editing mode
4. `[F]` badge indicates functions with existing overrides
5. "Clear Override" button removes the override

## Commit History (this branch)

| Commit | Description |
|--------|-------------|
| `f7d68181` | Add function selector UI for axis parameter overrides |
| `38603843` | Add override value editor UI for active functions |
| `a2625da0` | Add Active Functions UI for vehicle profile overrides |
| `f77294c8` | Add tiered config override system with profile integration |

## Build Command

```bash
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Debug /v:minimal /nologo
```

## Unit Tests Added

Created `TieredConfigTests` project with 80 tests covering:

| Test Suite | Tests | Description |
|------------|-------|-------------|
| ConfigMergerTests | 18 | Axis override merge, function config delta merge, three-layer merge |
| ConfigComparerTests | 28 | Equality checks for configs, overrides, floats, lists |
| ConflictDetectorTests | 14 | Axis conflict detection, partial overlap, locked axes |
| AxisConfigManagerTests | 20 | Base config lifecycle, apply/clear overrides, diff checking |

### Test Project Files

- `SimHubPlugin/TieredConfigTests/TieredConfigTests.csproj`
- `SimHubPlugin/TieredConfigTests/Program.cs`
- `SimHubPlugin/TieredConfigTests/ConfigMergerTests.cs`
- `SimHubPlugin/TieredConfigTests/ConfigComparerTests.cs`
- `SimHubPlugin/TieredConfigTests/ConflictDetectorTests.cs`
- `SimHubPlugin/TieredConfigTests/AxisConfigManagerTests.cs`

### Run Tests

```bash
cd SimHubPlugin/TieredConfigTests/bin/Debug
./TieredConfigTests.exe
```

## Reference

Design doc: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
