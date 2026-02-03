# Session Handoff

Date: 2026-02-03
Last commit: `05171481` — Add staged imports feature to Profile Browser

## What Was Done This Session

### Phase 3: Profile Integration — COMPLETE ✅

Implemented the profile override integration flow that auto-applies FunctionConfig and AxisConfig overrides on vehicle change.

#### Files Created

| File | Purpose |
|------|---------|
| `TieredConfig/FunctionConfigManager.cs` | Manages function config lifecycle for profile/user overrides |

#### Files Modified

| File | Changes |
|------|---------|
| `DiyFfbPlugin.cs` | Added `_functionConfigManager`, `_axisConfigManager` fields and public properties; Added `ApplyProfileFunctionOverrides()`, `ApplyProfileOverridesToFunction()`, `GetCurrentUserOverrides()`, `ShouldApplyProfileOverride()` methods; Modified `ApplyAircraftProfile()` to call override logic |
| `DiyFfbPluginUI.xaml.cs` | Added event handlers `OnMergedFunctionConfigChanged()`, `OnMergedAxisConfigChanged()`; Wired manager events in constructor; Modified `OnFunctionConfigUpdate()` to integrate with FunctionConfigManager |
| `DiyFfbPlugin.csproj` | Added `TieredConfig\FunctionConfigManager.cs` to compilation |

#### Build Status

**COMPILING** ✅ — All Phase 3 changes integrated.

#### Architecture

```
Vehicle Change
     │
     ▼
ApplyAircraftProfile()
     │
     ▼
ApplyProfileFunctionOverrides(profile)
     │
     ├── ClearAllProfileOverrides()      ← Restore base configs
     │
     ▼
For each function in profile.ActiveFunctionIds:
     │
     ├── FunctionConfigManager.ApplyProfileOverrides()
     │        │
     │        ├── ConfigMerger.MergeAllLayers(base, profile, user)
     │        │
     │        └── Fire FunctionConfigChanged event
     │                   │
     │                   ▼
     │              OnMergedFunctionConfigChanged()
     │                   │
     │                   ├── Update functions[] cache
     │                   ├── EnqueueFunctionConfigUpload(merged, store:false)
     │                   └── Update UI if selected
     │
     └── AxisConfigManager.ApplyFunctionOverrides()
              │
              └── Fire AxisConfigChanged event
                         │
                         ▼
                    OnMergedAxisConfigChanged()
                         │
                         ├── Update axes[] cache
                         ├── EnqueueAxisConfigUpload(axisId, merged, store:false)
                         └── Update UI if selected
```

#### New Config Reception Flow

When ESP32 sends a FunctionConfig:
1. `OnFunctionConfigUpdate()` receives config
2. Stores as base via `FunctionConfigManager.SetBaseConfig()`
3. Checks `ShouldApplyProfileOverride()` — is function in active profile?
4. If yes: calls `ApplyProfileOverridesToFunction()` → merged config sent to ESP32
5. If no: uses base config directly in UI

---

### Tiered Config Override Implementation — Foundation (Phase 1-2) ✅

Created foundation files for the tiered configuration override system in `SimHubPlugin/TieredConfig/`:

#### Files Created

| File | Purpose |
|------|---------|
| `TieredConfigTypes.cs` | Core data types: `ConfigLayer` enum, `UserPreferences`, `FunctionConfigOverrides`, `AxisParameterOverrides` |
| `ConfigMerger.cs` | Merge logic for config overlays (User > Profile > Hardware) |
| `ConfigComparer.cs` | Equality checks for diff-based config sending |
| `ConflictDetector.cs` | Detects when multiple functions override same axis |
| `FieldRouter.cs` | Routes field changes to appropriate layer (User/Profile/Hardware) |
| `ChangeTracker.cs` | Tracks pending unsaved changes per layer |
| `AxisConfigManager.cs` | Manages axis config lifecycle for function overrides |

#### Settings Changes (`DiyFfbPluginSettings.cs`)

Added to `DiyFfbPluginSettings`:

- `CurrentUserProfile` — user identity (defaults to Windows username)
- `UserPreferencesProfiles` — per-user preferences storage
- `FunctionAxisOverrides` — per-function axis parameter overrides

Added to `AircraftFfbProfile`:

- `FunctionOverrides` — vehicle-specific function config deltas
- `ActiveFunctionIds` — which functions are active for this profile

## Key Concepts

**StaticBalanceTuning vs StaticBalanceConfig:**

- `FunctionConfig.Types.StaticBalanceTuning` — Part of FunctionConfig, user-tunable (Enabled, Gain)
- `AxisConfig.Types.StaticBalanceConfig` — Part of AxisConfig, function-dependent axis override (XCenter, XHalfRange, Coeffs)

## Build & Test Commands

```bash
# SimHub Plugin
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Debug /v:minimal /nologo
```

## Next Steps

### Phase 4: UI for Override Configuration

Add UI controls to:
- View/edit function config overrides per profile
- Manage ActiveFunctionIds for each profile
- Configure user preferences

### Phase 5: AxisConfig Override UI

Add controls for axis parameter overrides:
- Kinematic parameters per function
- Static balance config per function

### Alternative

Implement progressive spring feature first (simpler, self-contained).
