# Tiered Config System — Implementation Complete

Branch: `ck_tiered_config`
Last commit: `4b21b196` — Update HANDOFF.md with latest commit

## Status: Ready for testing

All core phases from the design doc are implemented and tested, plus per-field layer badges and user profile UI.

### Recent Bug Fixes

1. **User profile key mismatch**: `GetCurrentUserOverrides()` now uses same key logic as write path (added `IsNullOrWhiteSpace` check).

2. **Clear behavior**: Clearing an override removes only the top layer (User first, then Profile on second click). If no Profile value exists, field returns to hardware default after clearing User.

3. **UI collapse on edit**: Removed `RefreshVehicleParams()` calls after field edits to preserve the expanded editor panel state. Per-field badges update via `RefreshOverrideFieldRow()` without rebuilding the entire UI.

4. **Badge not updating after edit**: `ApplyProfileOverridesToFunction()` was throwing `InvalidOperationException` when no base config existed (ESP32 not connected), which silently prevented `RefreshOverrideFieldRow()` from running. Fixed by adding `HasBaseConfig` check before calling `ApplyProfileOverrides` — matches the guard already used in `ApplyCurrentProfileOverrides()`. Overrides are still stored; they just won't apply to hardware until base config arrives.

5. **Function-level badge not updating**: The `[U]`/`[P]` badge next to the function name wasn't refreshing when all field overrides were cleared. Fixed by always creating the badge element and calling `RefreshFunctionLevelBadge()` after any field change.

## Implementation Summary

### Core Classes (`SimHubPlugin/TieredConfig/`)

| File                       | Purpose                                                                                    |
| -------------------------- | ------------------------------------------------------------------------------------------ |
| `TieredConfigTypes.cs`     | `ConfigLayer` enum, `UserPreferences`, `FunctionConfigOverrides`, `AxisParameterOverrides` |
| `ConfigMerger.cs`          | Pure merge functions: `MergeAxisOverrides`, `MergeFunctionConfig`, `MergeAllLayers`        |
| `ConfigComparer.cs`        | Deep equality checks with float tolerance for diff-checking                                |
| `ConflictDetector.cs`      | Detects when multiple active functions override the same axis                              |
| `AxisConfigManager.cs`     | Tracks base configs, applies/clears function overrides, fires events                       |
| `FunctionConfigManager.cs` | Manages profile + user override merging for function configs                               |
| `FieldRouter.cs`           | Routes field changes to User/Profile/Hardware layer by default                             |
| `ChangeTracker.cs`         | Tracks pending unsaved changes per layer                                                   |
| `ConfigLayerProvider.cs`   | Determines which layer a field value comes from for UI badges                              |

### Plugin API (`DiyFfbPlugin.cs`)

```csharp
// Query functions linking to an axis
List<FunctionAxisLink> GetFunctionsLinkingToAxis(int axisId)

// Check/get axis parameter overrides
bool HasAxisParameterOverride(int functionId, int axisId)
AxisParameterOverrides GetAxisParameterOverride(int functionId, int axisId)

// Set/update axis parameter overrides
void SetAxisParameterOverride(int functionId, int axisId, AxisParameterOverrides overrides)
void UpdateAxisParameterOverride(int functionId, int axisId, Action<AxisParameterOverrides> updateAction)

// Clear overrides
void ClearAxisParameterOverride(int functionId, int axisId)
void ClearAllAxisParameterOverrides(int functionId)

// Function config overrides (profile and user layers)
FunctionConfigOverrides GetFunctionOverrides(int functionId)       // Profile layer
FunctionConfigOverrides GetUserFunctionOverrides(int functionId)   // User layer

// Create layer provider for UI badges
ConfigLayerProvider CreateConfigLayerProvider()

// User profile selection
void SetCurrentUserProfile(string userProfile)
void ApplyCurrentProfileOverrides()
```

### UI Components

**Axis Tab — Function Selector** (`AxisConfigControl.xaml/.cs`)

- Dropdown to switch between "Axis N (base)" and function override modes
- `[F]` badge indicates functions with existing overrides
- "Clear Override" button to remove per-function axis overrides
- Kinematics and static balance editors work in both modes

**Vehicle Profile — Active Functions** (`DiyFfbPluginUI.xaml.cs`)

- Checkbox list of functions per vehicle profile
- Expandable override editor for each active function
- Sliders for OutputMin/Max, SimulatedMass, Friction
- Static balance tuning controls (Enabled, Gain)
- **Per-field layer badges**: `[P]` (green) for Profile overrides, `[U]` (blue) for User overrides
- **System → User tab**: create/select/delete user profiles for user-layer overrides

### Unit Tests (132 tests, all passing)

| Suite                  | Count | Coverage                                                                      |
| ---------------------- | ----- | ----------------------------------------------------------------------------- |
| ConfigMergerTests      | 18    | Axis override merge, function delta merge, three-layer merge, mutation safety |
| ConfigComparerTests    | 28    | Equality checks, float tolerance, null handling, list comparison              |
| ConflictDetectorTests  | 14    | No-conflict cases, conflict detection, partial overlap, locked axes           |
| AxisConfigManagerTests | 20    | Base config lifecycle, apply/clear overrides, diff-checking, events           |
| ChangeTrackerTests     | 25    | Track/commit/discard changes, layer routing, reroute, pending state queries   |
| FieldRouterTests       | 27    | User/Hardware/Profile routing, case insensitivity, nested fields, helpers     |

Run tests:

```bash
cd SimHubPlugin/TieredConfigTests/bin/Debug
./TieredConfigTests.exe
```

## Commit History

| Commit     | Description                                               |
| ---------- | --------------------------------------------------------- |
| `9ef65069` | Add user profile UI and fix function badge refresh        |
| `8f89eded` | Add ChangeTracker and FieldRouter unit tests              |
| `95c17a92` | Add unit tests for tiered config system                   |
| `f7d68181` | Add function selector UI for axis parameter overrides     |
| `38603843` | Add override value editor UI for active functions         |
| `a2625da0` | Add Active Functions UI for vehicle profile overrides     |
| `f77294c8` | Add tiered config override system with profile integration |

## Build

```bash
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Debug /v:minimal /nologo
```

## Next Tasks

Optional improvements if continuing development:

1. **Add Save/Discard Review Dialog** — UI for triaging pending changes before save (re-route, discard individual changes)
2. **Implement `DeltaExtractor`** — Extract changed fields from full config for cleaner delta generation
3. **Create single source of truth for override parameters** — Currently override field definitions are scattered across 5+ files (TieredConfigTypes.cs, FieldRouter.cs, ConfigLayerProvider.cs, DiyFfbPlugin.cs, DiyFfbPluginUI.xaml.cs). Consider:
   - JSON schema defining all overridable parameters with metadata (name, type, layer routing, min/max, tooltip)
   - Code generation or runtime loading from this schema
   - Developer documentation on how to add new override parameters

## Deferred Items Reference

These were lower-priority items from the design doc, not blocking merge:

| Item                         | Priority | Notes                                                                 |
| ---------------------------- | -------- | --------------------------------------------------------------------- |
| `DeltaExtractor` class       | Medium   | Extract changed fields from full config (not needed for current flow) |
| Per-field `[P]`/`[U]` badges | Done     | Live refresh now works correctly                                      |
| Save/Discard Review Dialog   | Low      | UI polish — triage pending changes before save                        |

## Design Reference

Full design doc: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
