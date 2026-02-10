# TieredConfig Orchestrator Extraction

**Status: COMPLETE**

## Problem

`DiyFfbPlugin.cs` is 4,685 lines carrying 7+ responsibilities. The tiered
config orchestration section (lines 2222-3338, ~1,116 lines, 38 methods) handles
baseline management, override application, user preferences, axis parameter
overrides, and field-level override operations. This is tightly coupled to
profile CRUD and settings persistence but has **zero UI references** — a clean
extraction target.

Additionally, `DiyFfbPluginUI.xaml.cs` contains two methods
(`OnFunctionConfigUpdate` at line 2651 and `OnAxisConfigUpdate` at line 2711)
that embed authority-decision logic (ESP32 vs stored baseline) that belongs in
the orchestrator, not the UI layer.

### Why extract

- **Testability**: The orchestrator's 38 methods can't be unit-tested without a
  full plugin instance. After extraction, mock the managers and settings.
- **Discoverability**: A developer looking for "how does clearing an override
  work" must search a 4,685-line file. After extraction: one file, one concern.
- **Change safety**: Adding a new override source (e.g., game-specific presets)
  requires edits scattered across the plugin file. After extraction: one class.
- **Prerequisite for plan 30**: Badge/graph/travel helper extraction (plan 30
  phases 2-4) will be easier once the orchestration API surface is clear.

## Design

### New class: `TieredConfigOrchestrator`

A non-static class in `TieredConfig/` that owns the merge-and-apply lifecycle.
Not a WPF control — no Dispatcher, no UI references.

```csharp
namespace DiyFfbPedal.TieredConfig
{
    public class TieredConfigOrchestrator
    {
        private readonly FunctionConfigManager _functionConfigManager;
        private readonly AxisConfigManager _axisConfigManager;
        private readonly ISettingsAccessor _settings;
        private readonly Func<AircraftFfbProfile> _getActiveProfile;
        private readonly Func<string> _getActiveGraphCategory;
        private readonly Func<string, string, string> _buildProfileKey;
        private readonly Action _persistSettings;

        // Events (moved from DiyFfbPlugin)
        public event EventHandler ContextChanged;
        public event EventHandler<OverrideFieldChangedEventArgs> OverrideFieldChanged;
    }
}
```

### Dependency injection strategy

The orchestrator needs access to plugin state without depending on the plugin
class. Two approaches were considered:

**Option A — Interface (`ISettingsAccessor`)**: Define an interface exposing only
the settings properties the orchestrator needs. `DiyFfbPluginSettings` implements
it (or a thin wrapper does). Cleanest but requires an interface + implementation.

**Option B — Direct `DiyFfbPluginSettings` reference + callbacks**: Pass the
settings object directly and use `Func<>` delegates for the 3 cross-boundary
methods. Simpler, no new interface, settings object is already a POCO.

**Decision: Option B.** The settings object is a simple data container with no
behavior. The 3 cross-boundary calls (`GetCurrentAircraftProfile`,
`GetActiveGraphCategory`, `BuildProfileKey`) become constructor-injected
`Func<>` delegates. `SaveCommonSettings` becomes an `Action` delegate.

### What moves

**38 methods + 2 events move from `DiyFfbPlugin.cs` to the orchestrator:**

#### A. Override Application
| Method | Current Lines | Notes |
|--------|-------------|-------|
| `ApplyProfileFunctionOverrides` | 2226-2300 | Master orchestrator |
| `ApplyProfileOverridesToFunction` | 2306-2326 | Single-function apply |
| `ReapplyMergedOverrides` | (new, from plan 30 §1d) | Extracted pattern |
| `ApplyCurrentProfileOverrides` | 3106-3110 | Convenience wrapper |

#### B. User Override Retrieval
| Method | Current Lines |
|--------|-------------|
| `GetCurrentUserOverrides` | 2331-2342 |

#### C. Function Activity
| Method | Current Lines |
|--------|-------------|
| `ShouldApplyProfileOverride` | 2348-2352 |
| `IsFunctionActive` | 2362-2368 |
| `IsDefaultActiveFunction` | 2370-2391 |
| `SeedDefaultActiveFunctionIds` | 2397-2414 |
| `SetFunctionActive` | 2420-2458 |

#### D. Profile/Override Accessors
| Method | Current Lines |
|--------|-------------|
| `GetOrCreateCurrentProfile` | 2464-2483 |
| `GetFunctionOverrides` | 2489-2497 |
| `GetUserFunctionOverrides` | 2503-2511 |
| `CreateConfigLayerProvider` | 2516-2523 |

#### E. Baseline Management — Function
| Method | Current Lines |
|--------|-------------|
| `GetFunctionBaseline` | 2529-2547 |
| `SetFunctionBaseline` | 2553-2568 |
| `HasFunctionBaseline` | 2573-2576 |
| `ClearFunctionBaseline` | 2582-2587 |

#### F. Baseline Management — Axis
| Method | Current Lines |
|--------|-------------|
| `GetAxisBaseline` | 2593-2610 |
| `SetAxisBaseline` | 2616-2633 |
| `HasAxisBaseline` | 2638-2641 |
| `ClearAxisBaseline` | 2647-2652 |

#### G. Manager Initialization
| Method | Current Lines |
|--------|-------------|
| `InitializeManagerFromSettings` | 2658-2705 |
| `InitializeAxisManagerFromSettings` | 2710-2736 |
| `GetInitialFunctionConfig` | 2742-2749 |
| `GetInitialAxisConfig` | 2755-2762 |

#### H. Override Field Operations
| Method | Current Lines |
|--------|-------------|
| `GetOrCreateFunctionOverrides` | 2767-2783 |
| `UpdateFunctionOverride` | 2788-2801 |
| `UpdateFunctionOverrideField` | 2806-2820 |
| `ClearFunctionOverrideField` | 2825-2839 |
| `ClearProfileFunctionOverrideField` | 2841-2872 |
| `UpdateUserFunctionOverride` | 2874-2897 |
| `ClearUserFunctionOverrideField` | 2899-2930 |
| `ClearAllFunctionOverrides` | 2936-2957 |
| `ClearOverrideFieldValue` | 2959-3033 |

#### I. User Preference Management
| Method | Current Lines |
|--------|-------------|
| `GetOrCreateUserFunctionOverrides` | 3035-3051 |
| `GetOrCreateCurrentUserOverrides` | 3053-3072 |
| `SetCurrentUserProfile` | 3077-3101 |

#### J. Utility
| Method | Current Lines |
|--------|-------------|
| `NormalizeFunctionOverrideFieldPath` | 3112-3125 |
| `GetFunctionOverrideTargetLayer` | 3127-3131 |

#### K. Axis Parameter Override API
| Method | Current Lines |
|--------|-------------|
| `GetFunctionsLinkingToAxis` | 3150-3182 |
| `HasAxisParameterOverride` | 3187-3199 |
| `GetAxisParameterOverride` | 3205-3215 |
| `GetOrCreateAxisParameterOverride` | 3220-3241 |
| `SetAxisParameterOverride` | 3247-3268 |
| `UpdateAxisParameterOverride` | 3273-3286 |
| `ClearAxisParameterOverride` | 3291-3319 |
| `ClearAllAxisParameterOverrides` | 3324-3336 |

#### L. Events
| Member | Current Lines |
|--------|-------------|
| `ContextChanged` event | 3989 |
| `OverrideFieldChanged` event | 3995 |
| `OnContextChanged()` | 4001-4007 |
| `OnOverrideFieldChanged()` | 4009-4012 |

### What stays in DiyFfbPlugin

The plugin keeps a `TieredConfigOrchestrator` field and exposes thin forwarding
properties/methods for the public API that UI controls call. The forwarding
layer is mechanical — no logic, just delegation:

```csharp
// In DiyFfbPlugin:
private TieredConfigOrchestrator _configOrchestrator;

public TieredConfigOrchestrator ConfigOrchestrator => _configOrchestrator;
// UI controls can call plugin.ConfigOrchestrator.HasFunctionBaseline(id)
// or we keep forwarding methods for backward compat during migration
```

Profile CRUD stays in the plugin:
- `GetCurrentAircraftProfile()` (line 2045)
- `HandleAircraftChange()` (line 2066)
- `SaveCurrentAircraftProfile()` (line 2151)
- `StoreCurrentProfile()` (line 2177)
- `ApplyAircraftProfile()` (line 2188) — calls into orchestrator

Profile comparison stays:
- `BuildCurrentAircraftProfile()` (line 3340)
- `HasUnsavedProfileChanges()` (line 3364)
- `AreProfilesEqual()` (line 3386)

Graph management, X-Plane UDP, FFB processing, serial communication all stay.

### ESP32 authority logic migration

`OnFunctionConfigUpdate` (DiyFfbPluginUI.xaml.cs line 2651) and
`OnAxisConfigUpdate` (line 2711) contain authority-decision logic:

```csharp
// Current (in UI code-behind):
if (plugin.HasFunctionBaseline(functionId))
{
    var merged = plugin.FunctionConfigManager.GetCurrentConfig(functionId);
    // push merged back to ESP32
}
else
{
    plugin.FunctionConfigManager.SetBaseConfig(functionId, config);
    plugin.ApplyProfileOverridesToFunction(functionId);
}
```

This decision belongs in the orchestrator. Add:

```csharp
// In TieredConfigOrchestrator:
public (bool authorityOverride, FunctionConfig mergedConfig)
    HandleIncomingFunctionConfig(int functionId, FunctionConfig incoming, bool fromEsp32)
{
    if (fromEsp32 && HasFunctionBaseline(functionId))
    {
        return (true, _functionConfigManager.GetCurrentConfig(functionId));
    }
    _functionConfigManager.SetBaseConfig(functionId, incoming);
    ApplyProfileOverridesToFunction(functionId);
    return (false, _functionConfigManager.GetCurrentConfig(functionId));
}
```

The UI handler becomes a thin dispatcher:
```csharp
var (override, merged) = plugin.ConfigOrchestrator.HandleIncomingFunctionConfig(functionId, config, fromEsp32: true);
if (override) { /* enqueue push-back to ESP32 */ }
function.Config = merged;
// update UI
```

Same pattern for `HandleIncomingAxisConfig`.

### Constructor

```csharp
public TieredConfigOrchestrator(
    FunctionConfigManager functionConfigManager,
    AxisConfigManager axisConfigManager,
    DiyFfbPluginSettings settings,
    IReadOnlyDictionary<int, Function> functions,
    Func<AircraftFfbProfile> getActiveProfile,
    Func<string> getActiveGraphCategory,
    Func<string, string, string> buildProfileKey,
    Action persistSettings)
```

### Nested type migration

- `FunctionAxisLink` (line 3138) moves into the orchestrator
- `OverrideFieldChangedEventArgs` (line 4670) moves to its own file or stays
  as a nested type in the orchestrator

## Phases

### Phase 1: Create orchestrator shell, move baseline management

Lowest-risk first cut. Baseline get/set/has/clear methods are self-contained
with minimal cross-dependencies.

**Move (12 methods):**
- Baseline — Function: `GetFunctionBaseline`, `SetFunctionBaseline`,
  `HasFunctionBaseline`, `ClearFunctionBaseline`
- Baseline — Axis: `GetAxisBaseline`, `SetAxisBaseline`, `HasAxisBaseline`,
  `ClearAxisBaseline`
- Initialization: `InitializeManagerFromSettings`,
  `InitializeAxisManagerFromSettings`, `GetInitialFunctionConfig`,
  `GetInitialAxisConfig`

**DiyFfbPlugin forwarding:** Keep public methods on the plugin that delegate to
the orchestrator. This avoids touching all 20+ call sites in phase 1.

**Files:**
| File | Change |
|------|--------|
| `TieredConfig/TieredConfigOrchestrator.cs` | New — shell + 12 methods |
| `DiyFfbPlugin.cs` | Create orchestrator in `Init()`, forward 12 methods |
| `DiyFfbPlugin.csproj` | Add Compile Include |

**Tests:** Existing test suite passes. Add 4-6 unit tests for baseline
round-trip (serialize → store → retrieve → deserialize).

### Phase 2: Move override application and activity queries

**Move (10 methods):**
- Override application: `ApplyProfileFunctionOverrides`,
  `ApplyProfileOverridesToFunction`, `ReapplyMergedOverrides` (from plan 30),
  `ApplyCurrentProfileOverrides`
- User override retrieval: `GetCurrentUserOverrides`
- Activity: `ShouldApplyProfileOverride`, `IsFunctionActive`,
  `IsDefaultActiveFunction`, `SeedDefaultActiveFunctionIds`, `SetFunctionActive`

**Key dependency:** These methods call `GetCurrentAircraftProfile()` which stays
in the plugin. This is the `Func<AircraftFfbProfile>` constructor parameter.

**Files:**
| File | Change |
|------|--------|
| `TieredConfig/TieredConfigOrchestrator.cs` | Add 10 methods |
| `DiyFfbPlugin.cs` | Move methods out, add forwarding |

**Tests:** Add 6-8 tests for `IsFunctionActive` (category defaults), override
application (profile + user layers), `SetFunctionActive` (activate/deactivate).

### Phase 3: Move override field operations and user preferences

**Move (15 methods):**
- Profile/override accessors: `GetOrCreateCurrentProfile`,
  `GetFunctionOverrides`, `GetUserFunctionOverrides`,
  `CreateConfigLayerProvider`
- Override field ops: `GetOrCreateFunctionOverrides`, `UpdateFunctionOverride`,
  `UpdateFunctionOverrideField`, `ClearFunctionOverrideField`,
  `ClearProfileFunctionOverrideField`, `UpdateUserFunctionOverride`,
  `ClearUserFunctionOverrideField`, `ClearAllFunctionOverrides`,
  `ClearOverrideFieldValue`
- User prefs: `GetOrCreateUserFunctionOverrides`,
  `GetOrCreateCurrentUserOverrides`, `SetCurrentUserProfile`
- Utility: `NormalizeFunctionOverrideFieldPath`,
  `GetFunctionOverrideTargetLayer`
- Events: `ContextChanged`, `OverrideFieldChanged`, `OnContextChanged`,
  `OnOverrideFieldChanged`

**Files:**
| File | Change |
|------|--------|
| `TieredConfig/TieredConfigOrchestrator.cs` | Add 15 methods + 2 events |
| `DiyFfbPlugin.cs` | Move methods out, add forwarding |

### Phase 4: Move axis parameter override API

**Move (9 methods + 1 nested type):**
- All axis parameter methods: `GetFunctionsLinkingToAxis`,
  `HasAxisParameterOverride`, `GetAxisParameterOverride`,
  `GetOrCreateAxisParameterOverride`, `SetAxisParameterOverride`,
  `UpdateAxisParameterOverride`, `ClearAxisParameterOverride`,
  `ClearAllAxisParameterOverrides`
- `FunctionAxisLink` nested class

**Files:**
| File | Change |
|------|--------|
| `TieredConfig/TieredConfigOrchestrator.cs` | Add 9 methods + nested type |
| `DiyFfbPlugin.cs` | Move methods out, add forwarding |

### Phase 5: Move ESP32 authority logic

Extract the authority-decision logic from `DiyFfbPluginUI.xaml.cs` into the
orchestrator.

**Add to orchestrator:**
- `HandleIncomingFunctionConfig(int functionId, FunctionConfig config, bool fromEsp32)`
- `HandleIncomingAxisConfig(int axisId, AxisConfig config, bool fromEsp32)`

**Simplify in UI:**
- `OnFunctionConfigUpdate` becomes a thin dispatcher (authority decision +
  manager manipulation removed, UI update remains)
- `OnAxisConfigUpdate` same treatment

**Files:**
| File | Change |
|------|--------|
| `TieredConfig/TieredConfigOrchestrator.cs` | Add 2 methods |
| `DiyFfbPluginUI.xaml.cs` | Simplify `OnFunctionConfigUpdate` and `OnAxisConfigUpdate` |

### Phase 6: Replace forwarding with direct orchestrator access

Once all methods are moved, update call sites to use
`plugin.ConfigOrchestrator.Method()` directly instead of
`plugin.Method()` forwarding. Remove forwarding methods from `DiyFfbPlugin`.

This phase touches many files but each change is mechanical:

**Call sites to update:**
| File | Approximate call count |
|------|----------------------|
| `DiyFfbPluginUI.xaml.cs` | ~30 |
| `AutomotivePedalConfigControl.xaml.cs` | ~15 |
| `FlightPedalsConfigControl.xaml.cs` | ~15 |
| `FlightStickConfigControl.xaml.cs` | ~15 |
| `ShifterConfigControl.xaml.cs` | ~15 |
| `FunctionConfigControl.xaml.cs` | ~10 |
| `AxisConfigControl.xaml.cs` | ~8 |
| `Controls/LayerBadgeWrapper.xaml.cs` | ~4 |

**Files:**
| File | Change |
|------|--------|
| `DiyFfbPlugin.cs` | Remove ~40 forwarding methods |
| All UI files above | `plugin.X()` → `plugin.ConfigOrchestrator.X()` |

## Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| Settings persistence timing | `_persistSettings` delegate calls `SaveCommonSettings()` on plugin — same thread, same timing |
| Profile key mismatch | `_buildProfileKey` delegate ensures orchestrator uses same key logic as plugin |
| Thread safety | No change — current code is single-threaded (UI dispatcher). Orchestrator inherits this. |
| Large diff in phase 6 | Mechanical find-replace. Can be done with IDE refactoring tools. |
| Forwarding overhead during migration | Negligible — single method call indirection. Removed in phase 6. |

## What This Enables

After extraction, `DiyFfbPlugin.cs` drops from ~4,685 to ~3,550 lines with
these remaining responsibilities:
- SimHub plugin lifecycle (`Init`, `End`, `DataUpdate`)
- FFB data processing and game event handling
- Graph management and compilation
- X-Plane UDP telemetry
- Profile CRUD and dirty detection
- Serial communication

Each of these could be further extracted in future plans (graph management is
the next-largest section at ~800 lines).

## Relationship to Plan 30

Plan 30 (Duplication Cleanup) phase 1 should be done **before** this plan.
Specifically:
- §1d extracts `ReapplyMergedOverrides()` — this method then moves into the
  orchestrator in phase 2 here
- §1a-c consolidate registry delegation — the simplified methods move cleanly

Plans 30 phases 2-5 (badge/graph/travel/accessor extraction) are independent
of this plan and can be done in parallel or after.
