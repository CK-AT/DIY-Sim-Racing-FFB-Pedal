# Duplication & Separation of Concerns Cleanup

**Status: COMPLETE**

## Problem

After the function processor extractions (plan 29), several categories of
duplication and poor separation remain:

1. **Registry bypass** — `DiyFfbPlugin.cs` and `ConfigLayerProvider.cs` have
   hand-coded switch statements duplicating what `OverrideFieldRegistry` already
   provides (field clearing, field checking, path normalization)
2. **Scattered orchestration** — the "gather profile + user deltas, call
   ApplyProfileOverrides" sequence appears 5 times in `DiyFfbPlugin.cs`
3. **Copy-pasted UI infrastructure** — badge refresh, graph parameter UI, travel
   display, and kinematic bounds logic are duplicated across 3-5 controls
4. **God class** — `DiyFfbPlugin.cs` is 4,685 lines with 7+ responsibilities

## Phases

### Phase 1: Registry Consolidation

Low-effort, high-impact. Eliminate 3 hand-coded switch statements by delegating
to `OverrideFieldRegistry`, and extract the repeated orchestration pattern.

#### 1a. Replace `ClearOverrideFieldValue` with registry delegation

`DiyFfbPlugin.cs` lines 2959-3033 is a 70-line switch with 18 cases that
manually nulls override fields. `OverrideFieldRegistry` already has `ClearValue`
delegates for every field (registered at lines 85-441).

**Before:**
```csharp
private static void ClearOverrideFieldValue(FunctionConfigOverrides overrides, string fieldName)
{
    switch (fieldName)
    {
        case "OutputMin":
        case "output_min":
            overrides.OutputMin = null; break;
        // ... 17 more cases
    }
}
```

**After:**
```csharp
private static void ClearOverrideFieldValue(FunctionConfigOverrides overrides, string fieldName)
{
    OverrideFieldRegistry.ClearValue(overrides, fieldName);
}
```

Prerequisite: `OverrideFieldRegistry` needs a public static `ClearValue(overrides, fieldName)`
convenience method that looks up the field and calls its `ClearValue` delegate.
Currently the `ClearValue` delegate exists per-field (line 65) but there's no
lookup-by-name wrapper.

**Files:**
| File | Change |
|------|--------|
| `TieredConfig/OverrideFieldRegistry.cs` | Add `public static void ClearValue(FunctionConfigOverrides, string)` |
| `DiyFfbPlugin.cs` | Replace lines 2959-3033 with one-liner |

#### 1b. Replace `ConfigLayerProvider.HasFieldValue` with registry delegation

`ConfigLayerProvider.cs` lines 58-129 is a 60-line switch checking `HasValue`
per field. `OverrideFieldRegistry.HasValue` (lines 572-582) already does this.

**Before:**
```csharp
private static bool HasFieldValue(FunctionConfigOverrides overrides, string fieldName)
{
    switch (fieldName)
    {
        case "OutputMin":
        case "output_min":
            return overrides.OutputMin.HasValue;
        // ... 17 more cases
    }
}
```

**After:**
```csharp
private static bool HasFieldValue(FunctionConfigOverrides overrides, string fieldName)
{
    return OverrideFieldRegistry.HasValue(overrides, fieldName);
}
```

**Files:**
| File | Change |
|------|--------|
| `TieredConfig/ConfigLayerProvider.cs` | Replace lines 58-129 with one-liner |

#### 1c. Replace `NormalizeFunctionOverrideFieldPath` with registry

`DiyFfbPlugin.cs` lines 3112-3125 is a hand-rolled switch for PascalCase →
snake_case. `OverrideFieldRegistry.NormalizeFieldPath` (lines 590-597) does
the same thing using the registry's field definitions.

**Files:**
| File | Change |
|------|--------|
| `DiyFfbPlugin.cs` | Replace lines 3112-3125 with delegation to `OverrideFieldRegistry.NormalizeFieldPath` |

#### 1d. Extract `ReapplyMergedOverrides(functionId)` method

The "gather deltas + apply" pattern appears 5 times:

| Method | Lines |
|--------|-------|
| `ApplyProfileOverridesToFunction` | 2308-2320 |
| `OnStartConfigLoad` (baseline restore) | 2688-2699 |
| `UpdateProfileFunctionOverride` | 2863-2870 |
| `UpdateUserFunctionOverride` | 2887-2895 |
| `ClearProfileFunctionOverrideField` | 2920-2928 |

Each instance does:
1. `GetCurrentAircraftProfile()` → `TryGetValue(functionId, out profileDelta)`
2. `GetCurrentUserOverrides()` → `TryGetValue(functionId, out userDelta)`
3. `_functionConfigManager.ApplyProfileOverrides(functionId, profileDelta, userDelta)`

Extract:
```csharp
private void ReapplyMergedOverrides(int functionId, bool diffCheck = true)
{
    var profile = GetCurrentAircraftProfile();
    FunctionConfigOverrides profileDelta = null;
    profile?.FunctionOverrides?.TryGetValue(functionId, out profileDelta);

    var userOverrides = GetCurrentUserOverrides();
    FunctionConfigOverrides userDelta = null;
    userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);

    _functionConfigManager.ApplyProfileOverrides(functionId, profileDelta, userDelta, diffCheck);
}
```

Replace all 5 call sites with `ReapplyMergedOverrides(functionId)` (or with
`diffCheck: false` where applicable).

**Files:**
| File | Change |
|------|--------|
| `DiyFfbPlugin.cs` | Add method, replace 5 call sites |

#### 1e. Use shared `ProtobufJsonHelper` instances

6 locations in `DiyFfbPlugin.cs` create `new JsonParser(...)` or
`new JsonFormatter(...)` instead of using the shared static instances in
`TieredConfigTypes.cs` lines 9-27.

| Line | Instance |
|------|----------|
| 2540 | `new JsonParser(...)` |
| 2562 | `new JsonFormatter(...)` |
| 2603 | `new JsonParser(...)` |
| 2624 | `new JsonFormatter(...)` |
| 2664 | `new JsonParser(...)` |
| 2715 | `new JsonParser(...)` |

Replace with `ProtobufJsonHelper.Parser` and `ProtobufJsonHelper.Formatter`.

**Files:**
| File | Change |
|------|--------|
| `DiyFfbPlugin.cs` | Replace 6 `new` calls with shared instances |

#### Phase 1 tests

Existing tests should continue to pass — these are pure refactorings that don't
change behavior. Run full test suite after each sub-step.

---

### Phase 2: Badge Infrastructure Extraction

Extract the copy-pasted badge refresh infrastructure from 5 controls into a
reusable base class or helper.

#### Duplicated methods (identical across all 5 controls)

| Method | Auto Pedal | Flight Pedals | Flight Stick | Shifter | FunctionConfig |
|--------|-----------|---------------|-------------|---------|----------------|
| `OnContextChanged` | 124-127 | 106-109 | 107-110 | 405-408 | 396-399 |
| `OnOverrideFieldChanged` | 129-135 | 111-117 | 112-118 | 410-416 | 401-408 |
| `RefreshAllBadges` | 137-143 | 180-186 | 205-211 | 418-424 | 410-417 |
| `RefreshBadgeForField` | 145-154 | 188-197 | 213-222 | 426-435 | 419-429 |
| `FindVisualChildren` | 156-165 | 199-208 | 224-233 | 437-446 | 431-447 |
| `InitializeBadges` | 167-176 | 210-219 | 235-244 | 448-457 | 449-461 |

Only `OnBadgeOverrideCleared` varies per control (field-specific revert logic).

#### Approach

Create a `BadgeHelper` class that encapsulates the generic infrastructure:

```csharp
internal class BadgeHelper
{
    private readonly FrameworkElement _root;
    private readonly Func<DiyFfbPlugin> _getPlugin;
    private readonly Func<Function> _getFunction;
    private readonly Action<string> _onBadgeOverrideCleared;

    public void InitializeBadges() { ... }
    public void RefreshAllBadges() { ... }
    public void RefreshBadgeForField(string fieldPath) { ... }
    public void OnContextChanged(object sender, EventArgs e) { ... }
    public void OnOverrideFieldChanged(object sender, OverrideFieldChangedEventArgs e) { ... }

    public static IEnumerable<T> FindVisualChildren<T>(DependencyObject parent)
        where T : DependencyObject { ... }
}
```

Each control creates a `BadgeHelper` in its constructor, passing `this` as the
root and a delegate for `OnBadgeOverrideCleared`. The control's `Loaded`/
`Unloaded` handlers wire up the helper's event handlers.

**Files:**
| File | Change |
|------|--------|
| `BadgeHelper.cs` | New — extracted generic badge infrastructure |
| `AutomotivePedalConfigControl.xaml.cs` | Replace lines 124-176 with helper usage |
| `FlightPedalsConfigControl.xaml.cs` | Replace lines 106-219 with helper usage |
| `FlightStickConfigControl.xaml.cs` | Replace lines 107-244 with helper usage |
| `ShifterConfigControl.xaml.cs` | Replace lines 405-457 with helper usage |
| `FunctionConfigControl.xaml.cs` | Replace lines 396-461 with helper usage |
| `DiyFfbPlugin.csproj` | Add Compile Include |

Estimated savings: ~200 lines of production code across 5 files.

---

### Phase 3: Graph Parameter UI Extraction

Extract the duplicated graph parameter slider/label generation from
FlightPedalsConfigControl and FlightStickConfigControl into a reusable helper.

#### Duplicated methods

| Method | FlightPedals | FlightStick |
|--------|-------------|-------------|
| `OnActiveGraphChanged` | 740-743 | 246-249 |
| `OnGraphParamChanged` | 745-789 | 251-297 |
| `FormatParamLabel` | 791-805 | 897-914 |
| `RefreshGraphParams` | 807-892 | 780-867 |
| `OrderParamsByGraph` | 894-920 | 869-895 |
| `MatchesGroup` | 931-938 | 931-938 |

Total: ~300 lines duplicated.

#### Approach

Create a `GraphParamHelper` class:

```csharp
internal class GraphParamHelper
{
    private readonly StackPanel _targetPanel;
    private readonly Func<IReadOnlyList<GraphParam>> _getParams;
    private readonly Action<string, double> _onParamChanged;
    private readonly Dictionary<string, Slider> _controls = new();
    private readonly Dictionary<string, TextBlock> _labels = new();
    private bool _isUpdating;

    public void Refresh() { ... }          // was RefreshGraphParams
    public void HandleParamChanged() { ... } // was OnGraphParamChanged
    public static string FormatLabel(GraphParam param, double value) { ... }
    public static IEnumerable<GraphParam> OrderByGraph(IEnumerable<GraphParam> ps) { ... }
}
```

**Files:**
| File | Change |
|------|--------|
| `GraphParamHelper.cs` | New — extracted graph parameter UI logic |
| `FlightPedalsConfigControl.xaml.cs` | Replace lines 740-938 with helper |
| `FlightStickConfigControl.xaml.cs` | Replace lines 246-938 with helper |
| `DiyFfbPlugin.csproj` | Add Compile Include |

Estimated savings: ~200 lines.

---

### Phase 4: Kinematic Bounds & Travel Display Extraction

#### 4a. Extract `KinematicBoundsHelper`

`OnKinematicParametersChanged` is near-identical in 3 controls:

| Control | Lines |
|---------|-------|
| FlightPedalsConfigControl | 236-264 |
| FlightStickConfigControl | 314-351 |
| SplineForceCurve | 62-91 |

Extract:
```csharp
internal static class KinematicBoundsHelper
{
    public static void UpdateBounds(
        KinematicParameters parameters,
        RangeSlider slider,
        Func<double> getLower, Func<double> getUpper,
        ref bool isUpdating, ref bool hasAxisRange,
        Dispatcher dispatcher)
    { ... }
}
```

#### 4b. Extract `TravelDisplayHelper`

Near-identical travel marker and axis state display in FlightPedals and
FlightStick:

| Method | FlightPedals | FlightStick |
|--------|-------------|-------------|
| `OnAxisStateUpdate` | 266-284 | 353-371 |
| `UpdateTravelMarkers` | 685-720 | 1022-1057 |
| `ApplyFallbackTravelRange` | 727-738 | 1064-1075 |

Extract into a helper that takes canvas/slider references and config accessors.

**Files:**
| File | Change |
|------|--------|
| `KinematicBoundsHelper.cs` | New |
| `TravelDisplayHelper.cs` | New |
| `FlightPedalsConfigControl.xaml.cs` | Delegate to helpers |
| `FlightStickConfigControl.xaml.cs` | Delegate to helpers |
| `SplineForceCurve.xaml.cs` | Delegate to KinematicBoundsHelper |
| `DiyFfbPlugin.csproj` | Add Compile Includes |

Estimated savings: ~150 lines.

---

### Phase 5: FlightStick Mode-Dispatch Cleanup

`FlightStickConfigControl.xaml.cs` lines 451-565 has 8 accessor methods, each
with an identical 3-way switch on `GetMode()`:

- `GetPosMin` (451-462), `GetPosMax` (464-475)
- `SetPosMin` (477-491), `SetPosMax` (493-507)
- `GetDamping` (509-520), `SetDamping` (535-549)
- `GetCenteringSpringConst` (522-533), `SetCenteringSpringConst` (551-565)

#### Approach

The three protobuf config types (`FlightStickPitchConfig`,
`FlightStickRollConfig`, `FlightStickCollectiveConfig`) have identical property
sets but don't share an interface. Options:

**Option A (recommended):** Add a `GetActiveSubConfig()` method returning
`dynamic` or a wrapper struct with the 4 properties. Reduces 8 methods to 1.

**Option B:** Define `IFlightStickSubConfig` via partial class extensions on
the protobuf types. Cleaner but more files.

Either way, the 8 methods collapse to direct property access on the returned
sub-config.

**Files:**
| File | Change |
|------|--------|
| `FlightStickConfigControl.xaml.cs` | Replace 8 accessors with `GetActiveSubConfig()` |

Estimated savings: ~80 lines.

---

## Not Worth Refactoring

| Issue | Reason |
|-------|--------|
| `FlightStickProcessor` triple Apply methods | Forced by protobuf type system; ~33 lines |
| `FunctionConfigManager` / `AxisConfigManager` parallel | Semantically different enough; ~300 lines each |
| `SwitchFunction` lifecycle scaffolding | Substantial variation in the middle; extract is awkward |
| INotifyPropertyChanged row boilerplate | Standard WPF; stable, rarely changes |
| Sine wave plot methods | Stable, localized; 2 of 4 are unique |

## Summary

| Phase | Effort | Lines Saved | Risk | Key Benefit |
|-------|--------|-------------|------|-------------|
| 1. Registry consolidation | ~2 hours | ~180 | Low | Single source of truth for field operations |
| 2. Badge extraction | ~3 hours | ~200 | Low | Badge changes in one place, not 5 |
| 3. Graph param extraction | ~3 hours | ~200 | Low | Graph UI changes in one place, not 2-3 |
| 4. Kinematic/travel helpers | ~3 hours | ~150 | Low | WPF interaction fixes in one place |
| 5. FlightStick accessors | ~1 hour | ~80 | Low | New properties added once, not 8× |

Total: ~810 lines eliminated, ~12 hours across all phases.
