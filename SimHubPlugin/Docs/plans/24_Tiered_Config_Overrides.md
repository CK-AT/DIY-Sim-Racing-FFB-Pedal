# Tiered Configuration Override System

## Problem Statement

Current architecture has flat configs with no hierarchy:
- Function configs are loaded as complete units
- No distinction between hardware defaults, vehicle-specific settings, and user preferences
- Shared hardware (e.g., shifter axes used for flight stick) requires separate config files
- No user identity concept

**Real-world examples:**
1. **Shifter** - Gate positions vary by car (Miata vs Porsche H-pattern)
2. **Pedals** - Travel and force preferences are personal (short vs tall driver)
3. **Flight stick** - Same physical axes as shifter, but different kinematics
4. **Flight controls** - Trim positions and spring feel vary by aircraft

## Proposed Hierarchy

```
┌─────────────────────────────────────────────────────────┐
│  User Layer (highest priority)                          │
│  Personal preferences that follow the user              │
│  Examples: pedal travel, force scaling, dead zones      │
├─────────────────────────────────────────────────────────┤
│  Profile Layer (vehicle-specific)                       │
│  Settings tied to a specific game/vehicle               │
│  Examples: shifter gate pattern, aircraft trim, detents │
├─────────────────────────────────────────────────────────┤
│  Default Layer (base config)                            │
│  Hardware fundamentals, rarely changed                  │
│  Examples: kinematics, motor orientation, force limits  │
└─────────────────────────────────────────────────────────┘
```

**Resolution:** User > Profile > Default (first non-null wins)

## What Lives Where?

### Default Layer
Hardware-dependent values that rarely change:
- `AxisConfig.steps_per_mm`, `mm_per_rev`
- `AxisConfig.b_motor_inverted`, `b_loadcell_inverted`
- `AxisConfig.f_max_loadcell`
- `AxisConfig.KinematicParameters` (base geometry)
- `FunctionConfig.linked_axes`
- `FunctionConfig.controller_output_axis`

### Profile Layer
Vehicle/aircraft-specific tuning:
- `ShifterConfig` gate positions (H-pattern varies by car)
- FFB graph parameter values (already exists in `AircraftFfbProfile`)
- Force curves (maybe)

### User Layer
Personal preferences:
- Pedal travel range (`output_min`, `output_max`)
- Seat position compensations

## Design Options

### Option A: Delta Overlay (Recommended)

Store only differences at each layer. Merge at runtime.

```
Default:  { k_spring: 100, k_damper: 50, trim: 0 }
Profile:  { trim: -0.1 }                          # only what's different
User:     { k_damper: 30 }                        # only what's different
─────────────────────────────────────────────────
Merged:   { k_spring: 100, k_damper: 30, trim: -0.1 }
```

**Pros:**
- Clear what each layer contributes
- Small override files
- Easy to "reset" (delete the override)

**Cons:**
- Complex merge logic
- Need schema for which fields are mergeable
- Nested messages (oneof types) harder to merge

### Option B: Full Replacement

Each layer stores complete config. Higher replaces lower entirely.

**Pros:**
- Simple implementation
- What you see is what you get

**Cons:**
- Duplication
- Hard to track "what did profile change vs default?"
- Updating default doesn't flow through

### Option C: Tagged Parameters

Each parameter carries source metadata.

```csharp
class ConfigValue<T> {
    T Value;
    ConfigSource Source;  // Default, Profile, User
}
```

**Pros:**
- UI can show provenance
- Flexible reset per-parameter

**Cons:**
- Invasive change to all data structures
- Protobuf doesn't support this natively

## Recommended Approach: Option A with Constraints

Use delta overlay, but with practical constraints:

1. **Axis configs** - No direct overlays; kinematics can be overridden per-axis via function (plugin-side merge)
2. **Function configs** - Delta merge for scalar fields, full replacement for complex nested types
3. **Graph params** - Already work this way (dictionary merge)

### Merge Rules

| Field Type | Merge Behavior |
|------------|----------------|
| Scalar (float, int, bool) | Override if present |
| String | Override if non-empty |
| Repeated (arrays) | Full replacement |
| Oneof (type-specific config) | Full replacement |
| Map (dictionary) | Key-level merge |

## User Identity

Options considered:

| Approach | Pros | Cons |
|----------|------|------|
| Windows username | Automatic, no setup | Multi-user PC issues |
| Plugin profile name | Explicit, portable | Extra setup step |
| Hardware fingerprint | Follows rig | Can't distinguish users on shared rig |

**Recommendation:** Named user profiles in plugin settings, with Windows username as default.

## Storage Structure

No separate config files needed. All layers stored in existing infrastructure:

| Layer | Storage Location | Notes |
|-------|------------------|-------|
| Default (hardware) | ESP32 EEPROM | Already exists |
| Profile (vehicle) | `AircraftFfbProfile` | Extend with function config deltas |
| User (preferences) | `DiyFfbPluginSettings` | New `UserPreferences` section |

```csharp
class DiyFfbPluginSettings {
    // Existing
    Dictionary<string, AircraftFfbProfile> AircraftFfbProfiles;

    // Add: User preferences
    string CurrentUserProfile;  // default: Environment.UserName
    Dictionary<string, UserPreferences> UserPreferencesProfiles;

    // Add: Per-function axis parameter overrides (function_id → axis_id → overrides)
    // Defined globally here; only APPLIED when function is in ActiveFunctionIds for current profile
    Dictionary<int, Dictionary<int, AxisParameterOverrides>> FunctionAxisOverrides;
}

class UserPreferences {
    // Function config overrides keyed by FunctionID
    Dictionary<int, FunctionConfigOverrides> FunctionOverrides;
}

class FunctionConfigOverrides {
    // Only user-tunable fields, all nullable
    float? OutputMin;
    float? OutputMax;
    // ... other preferences
}

class AircraftFfbProfile {
    // Existing
    string GraphPath;
    Dictionary<string, object> GraphParamValues;

    // Add: Vehicle-specific function config deltas
    Dictionary<int, FunctionConfigOverrides> FunctionOverrides;
}
```

**Import/Export:** Serialize `UserPreferences` or `AircraftFfbProfile` to JSON file on demand.

**Function config export** includes axis parameter overrides from `FunctionAxisOverrides` for that function.

**Import behavior:** Loading a config from JSON marks all modified fields as pending changes (routed to User or Profile layer based on field category). Changes remain pending until explicitly saved or discarded.

**Hardware layer:** "Upload and Persist" (Shift+Click Upload) writes axis config to ESP32 EEPROM. This is the mechanism for saving hardware-layer changes.

## Shared Hardware Problem

**Scenario:** Shifter hardware (axes 2-3) also used for flight stick roll + collective.

**Current approach:** Separate config files, manual switching.

**Proposed solution:** Plugin-side axis parameter overrides per function, merged into AxisConfig before sending to ESP32.

```
┌──────────────────────────────┐     ┌─────────────────────────────────┐
│ AxisConfig (hardware truth)  │     │ FunctionConfig (behavior)       │
├──────────────────────────────┤     ├─────────────────────────────────┤
│ Axis 2:                      │     │ ShifterFunction                 │
│   kinematics: ShifterLinkage │────▶│   linked_axes: [2,3]            │
│   steps_per_mm: 200          │     │   kinematic_overrides: null     │
│   motor_inverted: false      │     │   → uses axis kinematics        │
│                              │     ├─────────────────────────────────┤
│ Axis 3:                      │     │ FlightStickRollFunction         │
│   kinematics: ShifterLinkage │     │   linked_axes: [2]              │
│   steps_per_mm: 200          │     │   kinematic_overrides: {        │
│   motor_inverted: false      │     │     2: {...}                    │
│                              │     │   }                             │
│                              │     │   → plugin merges into axis 2   │
│                              │     ├─────────────────────────────────┤
│                              │     │ CollectiveFunction              │
│                              │     │   linked_axes: [3]              │
│                              │     │   kinematic_overrides: {        │
│                              │     │     3: {...}                    │
│                              │     │   }                             │
│                              │     │   → plugin merges into axis 3   │
└──────────────────────────────┘     └─────────────────────────────────┘
```

Key insight: **Axis physics (kinematics, static balance) live in AxisConfig. Functions can override per-axis, but merge happens plugin-side.**

### Implementation Pattern (Plugin-Side Merge)

**Plugin storage** — Extend function config with per-axis overrides for axis physics:

```csharp
class AxisParameterOverrides {
    // All nullable — only non-null fields override the axis base
    KinematicParameters? Kinematics;
    StaticBalanceConfig? StaticBalance;
}

class FunctionConfigExtended {
    // Existing function config fields...

    // Per-axis parameter overrides (plugin-side only, not in protobuf)
    Dictionary<int, AxisParameterOverrides> AxisOverrides;
}
```

**Function Activation Lifecycle:**

| Trigger | Behavior |
|---------|----------|
| Plugin startup / axis connect | Read configs from ESP32 → working "base" state |
| Profile change (vehicle switch) | Send axis and/or function configs as needed (diff-checked) |
| Manual upload (click Upload) | Send config for current tab only (axis OR function, no diff check) |
| Manual upload (Shift+Upload) | Same as above + sets "store" flag → writes to EEPROM |
| "Restart all Axes" | ESP32 reloads from EEPROM → plugin reads clean base |

**Crash recovery:** On reconnect, plugin reads current ESP32 state. If clean slate needed, user clicks "Restart all Axes" to reload from EEPROM.

**Runtime flow:**

```
Plugin                                    ESP32
──────                                    ─────
FunctionConfig.AxisOverrides[2]
  (Kinematics, StaticBalance)
       │
       ▼
  On function activation:
  - Merge overrides into AxisConfig[2]
  - Diff against last-sent config
  - If changed: send AxisConfig ─────────▶ AxisConfig (merged)
       │
  On function deactivation:
  - Restore base AxisConfig[2]
  - Diff against last-sent config
  - If changed: send AxisConfig ─────────▶ AxisConfig (restored)
```

**Config change tracking:** Plugin maintains hash/snapshot of last-sent config per axis and function.

| Send type             | Diff check | Scope                                 |
|-----------------------|------------|---------------------------------------|
| Auto (profile change) | Yes        | Axis and/or function as needed        |
| Manual Upload         | No         | Current tab only (axis OR function)   |

Benefits:

- Reduces USB traffic on automatic profile switches
- Cleaner debugging (fewer spurious config messages)

Note: EEPROM writes only happen on Shift+Upload (explicit user action with "store" flag).

**No ESP32 changes required** — device just receives normal AxisConfig updates.

### Benefits

- Conceptual integrity: axis = hardware reality
- Backwards compatible: existing configs work unchanged
- No protobuf changes for FunctionConfig
- No ESP32 memory bloat from kinematics in function configs
- Multi-axis functions naturally supported (each axis can have different override)
- Plugin is single source of truth for layer semantics

## Migration Path

### Phase 1: Foundation
1. Add `CurrentUserProfile` to settings
2. Create delta config data structures
3. Implement merge logic for FunctionConfig

### Phase 2: Storage

1. Add `UserPreferences` and `UserPreferencesProfiles` to `DiyFfbPluginSettings`
2. Add `FunctionOverrides` to `AircraftFfbProfile`
3. Add UI for viewing/editing user preferences

### Phase 3: Profile Integration

1. Extend `AircraftFfbProfile` to include function config deltas
2. Auto-apply profile overrides on vehicle change

### Phase 4: Axis Parameter Overrides (Plugin-Side)

1. Add `AxisOverrides` dictionary to `FunctionConfigExtended` (plugin-side only)
2. Track "base" AxisConfig per axis (before any function override)
3. On function activation: merge overrides into AxisConfig, send to ESP32
4. On function deactivation: restore base AxisConfig, send to ESP32
5. Add `[F]` badge to axis UI when function override is active
6. Add UI to edit per-axis overrides in function config (reuse axis editors)

Overridable axis parameters:

- `KinematicParameters` — linkage geometry, travel limits
- `StaticBalanceConfig` — position-dependent force compensation

#### Plugin Changes

```csharp
// Per-axis parameter overrides (all nullable)
class AxisParameterOverrides {
    public KinematicParameters? Kinematics { get; set; }
    public StaticBalanceConfig? StaticBalance { get; set; }
}

// Extend existing function config wrapper
class FunctionConfigExtended {
    public FunctionConfig ProtoConfig { get; set; }

    // Per-axis parameter overrides (axis_id → overrides)
    public Dictionary<int, AxisParameterOverrides> AxisOverrides { get; set; }
}

// Track base configs for restoration
class AxisConfigManager {
    private Dictionary<int, AxisConfig> _baseConfigs = new();
    private Dictionary<int, int> _activeOverrideFunction = new();  // axis_id → function_id

    public void ApplyFunctionOverride(int axisId, int functionId, AxisParameterOverrides overrides) {
        if (!_baseConfigs.ContainsKey(axisId)) {
            _baseConfigs[axisId] = GetCurrentAxisConfig(axisId).Clone();
        }
        _activeOverrideFunction[axisId] = functionId;

        var merged = GetCurrentAxisConfig(axisId).Clone();
        if (overrides.Kinematics != null)
            merged.KinematicParameters = overrides.Kinematics;
        if (overrides.StaticBalance != null)
            merged.StaticBalanceConfig = overrides.StaticBalance;
        SendAxisConfig(axisId, merged);
    }

    public void ClearFunctionOverride(int axisId) {
        if (_baseConfigs.TryGetValue(axisId, out var baseConfig)) {
            SendAxisConfig(axisId, baseConfig);
            _baseConfigs.Remove(axisId);
            _activeOverrideFunction.Remove(axisId);
        }
    }

    public bool HasFunctionOverride(int axisId) => _activeOverrideFunction.ContainsKey(axisId);
}
```

**No ESP32 changes required** — device receives standard AxisConfig updates.

### Unit Testing

Core logic is testable as pure functions without mocking. Focus testing on merge/diff/conflict logic where subtle bugs are most likely.

#### Testable Components

| Component          | Responsibility                               | Dependencies |
|--------------------|----------------------------------------------|--------------|
| `ConfigMerger`     | Merge overrides into base config             | None (pure)  |
| `ConfigComparer`   | Determine if two configs are equal           | None (pure)  |
| `ConflictDetector` | Find axis conflicts between active functions | None (pure)  |
| `FieldRouter`      | Determine target layer for a field           | None (pure)  |
| `DeltaExtractor`   | Extract changed fields from full config      | None (pure)  |
| `ChangeTracker`    | Track pending changes per layer              | None (pure)  |

#### ConfigMerger Tests

```csharp
[TestFixture]
public class ConfigMergerTests
{
    // === Axis Parameter Override Merging ===

    [Test]
    public void Merge_OnlyKinematics_PreservesStaticBalance()
    {
        var baseConfig = new AxisConfig {
            KinematicParameters = DefaultKinematics(),
            StaticBalanceConfig = CalibratedBalance()
        };
        var overrides = new AxisParameterOverrides {
            Kinematics = CustomKinematics(),
            StaticBalance = null  // don't override
        };

        var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

        Assert.AreEqual(CustomKinematics(), merged.KinematicParameters);
        Assert.AreEqual(CalibratedBalance(), merged.StaticBalanceConfig);
    }

    [Test]
    public void Merge_OnlyStaticBalance_PreservesKinematics()
    {
        var baseConfig = new AxisConfig {
            KinematicParameters = DefaultKinematics(),
            StaticBalanceConfig = CalibratedBalance()
        };
        var overrides = new AxisParameterOverrides {
            Kinematics = null,  // don't override
            StaticBalance = CustomBalance()
        };

        var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

        Assert.AreEqual(DefaultKinematics(), merged.KinematicParameters);
        Assert.AreEqual(CustomBalance(), merged.StaticBalanceConfig);
    }

    [Test]
    public void Merge_BothOverrides_ReplacesAll()
    {
        var baseConfig = new AxisConfig {
            KinematicParameters = DefaultKinematics(),
            StaticBalanceConfig = CalibratedBalance()
        };
        var overrides = new AxisParameterOverrides {
            Kinematics = CustomKinematics(),
            StaticBalance = CustomBalance()
        };

        var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

        Assert.AreEqual(CustomKinematics(), merged.KinematicParameters);
        Assert.AreEqual(CustomBalance(), merged.StaticBalanceConfig);
    }

    [Test]
    public void Merge_EmptyOverrides_ReturnsBaseUnchanged()
    {
        var baseConfig = new AxisConfig {
            KinematicParameters = DefaultKinematics(),
            StaticBalanceConfig = CalibratedBalance()
        };
        var overrides = new AxisParameterOverrides {
            Kinematics = null,
            StaticBalance = null
        };

        var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

        Assert.AreEqual(DefaultKinematics(), merged.KinematicParameters);
        Assert.AreEqual(CalibratedBalance(), merged.StaticBalanceConfig);
    }

    [Test]
    public void Merge_DoesNotMutateOriginal()
    {
        var baseConfig = new AxisConfig {
            KinematicParameters = DefaultKinematics(),
            StaticBalanceConfig = CalibratedBalance()
        };
        var originalKinematics = baseConfig.KinematicParameters;
        var overrides = new AxisParameterOverrides { Kinematics = CustomKinematics() };

        ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

        Assert.AreEqual(originalKinematics, baseConfig.KinematicParameters);
    }

    // === Function Config Delta Merging ===

    [Test]
    public void MergeFunctionConfig_ScalarOverride_ReplacesValue()
    {
        var baseConfig = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };
        var delta = new FunctionConfigOverrides { OutputMax = 0.8f };

        var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

        Assert.AreEqual(0.0f, merged.OutputMin);
        Assert.AreEqual(0.8f, merged.OutputMax);
    }

    [Test]
    public void MergeFunctionConfig_NullDelta_ReturnsBaseUnchanged()
    {
        var baseConfig = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };

        var merged = ConfigMerger.MergeFunctionConfig(baseConfig, null);

        Assert.AreEqual(0.0f, merged.OutputMin);
        Assert.AreEqual(1.0f, merged.OutputMax);
    }

    // === Three-Layer Merge (User > Profile > Default) ===

    [Test]
    public void MergeThreeLayers_UserWins()
    {
        var hardware = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };
        var profile = new FunctionConfigOverrides { OutputMax = 0.9f };
        var user = new FunctionConfigOverrides { OutputMax = 0.8f };

        var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);

        Assert.AreEqual(0.8f, merged.OutputMax);  // User wins
    }

    [Test]
    public void MergeThreeLayers_ProfileWins_WhenNoUserOverride()
    {
        var hardware = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };
        var profile = new FunctionConfigOverrides { OutputMax = 0.9f };
        var user = new FunctionConfigOverrides { OutputMax = null };  // no override

        var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);

        Assert.AreEqual(0.9f, merged.OutputMax);  // Profile wins
    }

    [Test]
    public void MergeThreeLayers_HardwareWins_WhenNoOverrides()
    {
        var hardware = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };
        var profile = new FunctionConfigOverrides { };
        var user = new FunctionConfigOverrides { };

        var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);

        Assert.AreEqual(1.0f, merged.OutputMax);  // Hardware default
    }
}
```

#### ConfigComparer Tests

```csharp
[TestFixture]
public class ConfigComparerTests
{
    // === Equality Checks ===

    [Test]
    public void AreEqual_IdenticalConfigs_ReturnsTrue()
    {
        var config1 = CreateAxisConfig(kinematics: DefaultKinematics());
        var config2 = CreateAxisConfig(kinematics: DefaultKinematics());

        Assert.IsTrue(ConfigComparer.AreEqual(config1, config2));
    }

    [Test]
    public void AreEqual_DifferentKinematics_ReturnsFalse()
    {
        var config1 = CreateAxisConfig(kinematics: DefaultKinematics());
        var config2 = CreateAxisConfig(kinematics: CustomKinematics());

        Assert.IsFalse(ConfigComparer.AreEqual(config1, config2));
    }

    [Test]
    public void AreEqual_FloatTolerance_HandlesRoundingErrors()
    {
        var config1 = CreateAxisConfig(outputMax: 0.1f + 0.2f);  // floating point arithmetic
        var config2 = CreateAxisConfig(outputMax: 0.3f);

        Assert.IsTrue(ConfigComparer.AreEqual(config1, config2));
    }

    [Test]
    public void AreEqual_NestedStructures_ComparesDeep()
    {
        var config1 = CreateAxisConfig(kinematics: new KinematicParameters {
            TravelMin = 0, TravelMax = 100
        });
        var config2 = CreateAxisConfig(kinematics: new KinematicParameters {
            TravelMin = 0, TravelMax = 100
        });

        Assert.IsTrue(ConfigComparer.AreEqual(config1, config2));
    }

    [Test]
    public void AreEqual_NestedStructures_DetectsDifference()
    {
        var config1 = CreateAxisConfig(kinematics: new KinematicParameters {
            TravelMin = 0, TravelMax = 100
        });
        var config2 = CreateAxisConfig(kinematics: new KinematicParameters {
            TravelMin = 0, TravelMax = 110  // different
        });

        Assert.IsFalse(ConfigComparer.AreEqual(config1, config2));
    }

    [Test]
    public void AreEqual_RepeatedFields_ComparesArrayContents()
    {
        var config1 = new StaticBalanceConfig { Coeffs = { 1.0f, 2.0f, 3.0f } };
        var config2 = new StaticBalanceConfig { Coeffs = { 1.0f, 2.0f, 3.0f } };

        Assert.IsTrue(ConfigComparer.AreEqual(config1, config2));
    }

    [Test]
    public void AreEqual_RepeatedFields_DetectsLengthDifference()
    {
        var config1 = new StaticBalanceConfig { Coeffs = { 1.0f, 2.0f, 3.0f } };
        var config2 = new StaticBalanceConfig { Coeffs = { 1.0f, 2.0f } };

        Assert.IsFalse(ConfigComparer.AreEqual(config1, config2));
    }

    [Test]
    public void AreEqual_NullHandling_BothNull_ReturnsTrue()
    {
        Assert.IsTrue(ConfigComparer.AreEqual<AxisConfig>(null, null));
    }

    [Test]
    public void AreEqual_NullHandling_OneNull_ReturnsFalse()
    {
        var config = CreateAxisConfig();
        Assert.IsFalse(ConfigComparer.AreEqual(config, null));
        Assert.IsFalse(ConfigComparer.AreEqual(null, config));
    }
}
```

#### ConflictDetector Tests

```csharp
[TestFixture]
public class ConflictDetectorTests
{
    // === No Conflicts ===

    [Test]
    public void DetectConflicts_SingleFunction_NoConflict()
    {
        var activeFunctions = new Dictionary<int, AxisParameterOverrides> {
            [1] = new() { Kinematics = CustomKinematics() }  // function 1 → axis 2
        };
        var functionAxisMapping = new Dictionary<int, HashSet<int>> {
            [1] = new() { 2 }
        };

        var conflicts = ConflictDetector.FindConflicts(activeFunctions, functionAxisMapping);

        Assert.IsEmpty(conflicts);
    }

    [Test]
    public void DetectConflicts_MultipleFunctions_DifferentAxes_NoConflict()
    {
        var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>> {
            [1] = new() { [2] = new() { Kinematics = CustomKinematics() } },  // func 1 → axis 2
            [2] = new() { [3] = new() { Kinematics = CustomKinematics() } }   // func 2 → axis 3
        };

        var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

        Assert.IsEmpty(conflicts);
    }

    // === Conflicts ===

    [Test]
    public void DetectConflicts_TwoFunctions_SameAxis_ReturnsConflict()
    {
        var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>> {
            [1] = new() { [2] = new() { Kinematics = CustomKinematics() } },  // func 1 → axis 2
            [2] = new() { [2] = new() { Kinematics = OtherKinematics() } }    // func 2 → axis 2
        };

        var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

        Assert.AreEqual(1, conflicts.Count);
        Assert.AreEqual(2, conflicts[0].AxisId);
        CollectionAssert.AreEquivalent(new[] { 1, 2 }, conflicts[0].ConflictingFunctionIds);
    }

    [Test]
    public void DetectConflicts_ThreeFunctions_SameAxis_ReturnsAllInConflict()
    {
        var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>> {
            [1] = new() { [2] = new() { Kinematics = CustomKinematics() } },
            [2] = new() { [2] = new() { Kinematics = OtherKinematics() } },
            [3] = new() { [2] = new() { StaticBalance = CustomBalance() } }
        };

        var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

        Assert.AreEqual(1, conflicts.Count);
        CollectionAssert.AreEquivalent(new[] { 1, 2, 3 }, conflicts[0].ConflictingFunctionIds);
    }

    [Test]
    public void DetectConflicts_MultipleAxes_ReturnsMultipleConflicts()
    {
        var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>> {
            [1] = new() { [2] = new() { Kinematics = CustomKinematics() } },
            [2] = new() { [2] = new() { Kinematics = OtherKinematics() },
                         [3] = new() { Kinematics = CustomKinematics() } },
            [3] = new() { [3] = new() { Kinematics = OtherKinematics() } }
        };

        var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

        Assert.AreEqual(2, conflicts.Count);  // axis 2 and axis 3
    }

    // === Edge Cases ===

    [Test]
    public void DetectConflicts_EmptyOverrides_NoConflict()
    {
        var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>> {
            [1] = new() { [2] = new() { Kinematics = null, StaticBalance = null } },  // empty
            [2] = new() { [2] = new() { Kinematics = CustomKinematics() } }
        };

        var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

        // Empty overrides don't conflict — they don't actually override anything
        Assert.IsEmpty(conflicts);
    }

    [Test]
    public void DetectConflicts_PartialOverlap_OnlyKinematicsVsOnlyStaticBalance_NoConflict()
    {
        var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>> {
            [1] = new() { [2] = new() { Kinematics = CustomKinematics(), StaticBalance = null } },
            [2] = new() { [2] = new() { Kinematics = null, StaticBalance = CustomBalance() } }
        };

        var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

        // No conflict: they override different fields
        Assert.IsEmpty(conflicts);
    }

    [Test]
    public void DetectConflicts_PartialOverlap_BothOverrideKinematics_Conflict()
    {
        var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>> {
            [1] = new() { [2] = new() { Kinematics = CustomKinematics(), StaticBalance = null } },
            [2] = new() { [2] = new() { Kinematics = OtherKinematics(), StaticBalance = CustomBalance() } }
        };

        var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

        // Conflict: both override kinematics
        Assert.AreEqual(1, conflicts.Count);
    }
}
```

#### FieldRouter Tests

```csharp
[TestFixture]
public class FieldRouterTests
{
    [Test]
    public void GetTargetLayer_OutputMin_ReturnsUser()
    {
        Assert.AreEqual(ConfigLayer.User, FieldRouter.GetTargetLayer("output_min"));
    }

    [Test]
    public void GetTargetLayer_OutputMax_ReturnsUser()
    {
        Assert.AreEqual(ConfigLayer.User, FieldRouter.GetTargetLayer("output_max"));
    }

    [Test]
    public void GetTargetLayer_ShifterGateWidth_ReturnsProfile()
    {
        Assert.AreEqual(ConfigLayer.Profile, FieldRouter.GetTargetLayer("shifter_config.gate_width"));
    }

    [Test]
    public void GetTargetLayer_KinematicParameters_ReturnsHardware()
    {
        Assert.AreEqual(ConfigLayer.Hardware, FieldRouter.GetTargetLayer("kinematic_parameters"));
    }

    [Test]
    public void GetTargetLayer_StaticBalanceConfig_ReturnsHardware()
    {
        Assert.AreEqual(ConfigLayer.Hardware, FieldRouter.GetTargetLayer("static_balance_config"));
    }

    [Test]
    public void GetTargetLayer_LinkedAxes_ReturnsHardware()
    {
        Assert.AreEqual(ConfigLayer.Hardware, FieldRouter.GetTargetLayer("linked_axes"));
    }

    [Test]
    public void GetTargetLayer_UnknownField_ReturnsProfile()
    {
        // Default to Profile layer for unrecognized fields (safest middle ground)
        Assert.AreEqual(ConfigLayer.Profile, FieldRouter.GetTargetLayer("unknown_field"));
    }
}
```

#### DeltaExtractor Tests

```csharp
[TestFixture]
public class DeltaExtractorTests
{
    [Test]
    public void ExtractDelta_NoChanges_ReturnsEmptyDelta()
    {
        var baseConfig = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };
        var currentConfig = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };

        var delta = DeltaExtractor.ExtractFunctionConfigDelta(baseConfig, currentConfig);

        Assert.IsNull(delta.OutputMin);
        Assert.IsNull(delta.OutputMax);
        Assert.IsTrue(delta.IsEmpty);
    }

    [Test]
    public void ExtractDelta_SingleChange_ReturnsOnlyChangedField()
    {
        var baseConfig = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };
        var currentConfig = new FunctionConfig { OutputMin = 0.0f, OutputMax = 0.8f };

        var delta = DeltaExtractor.ExtractFunctionConfigDelta(baseConfig, currentConfig);

        Assert.IsNull(delta.OutputMin);  // unchanged
        Assert.AreEqual(0.8f, delta.OutputMax);  // changed
    }

    [Test]
    public void ExtractDelta_MultipleChanges_ReturnsAllChangedFields()
    {
        var baseConfig = new FunctionConfig { OutputMin = 0.0f, OutputMax = 1.0f };
        var currentConfig = new FunctionConfig { OutputMin = 0.1f, OutputMax = 0.9f };

        var delta = DeltaExtractor.ExtractFunctionConfigDelta(baseConfig, currentConfig);

        Assert.AreEqual(0.1f, delta.OutputMin);
        Assert.AreEqual(0.9f, delta.OutputMax);
    }

    [Test]
    public void ExtractDelta_NestedChange_ReturnsNestedDelta()
    {
        var baseConfig = new FunctionConfig {
            ShifterConfig = new ShifterConfig { GateWidth = 10 }
        };
        var currentConfig = new FunctionConfig {
            ShifterConfig = new ShifterConfig { GateWidth = 12 }
        };

        var delta = DeltaExtractor.ExtractFunctionConfigDelta(baseConfig, currentConfig);

        Assert.AreEqual(12, delta.ShifterConfig.GateWidth);
    }
}
```

#### ChangeTracker Tests

```csharp
[TestFixture]
public class ChangeTrackerTests
{
    // === Pending Changes ===

    [Test]
    public void TrackChange_AddsToUserPending()
    {
        var tracker = new ChangeTracker();

        tracker.TrackChange(functionId: 1, "output_max", 0.8f, ConfigLayer.User);

        Assert.IsTrue(tracker.HasUnsavedChanges);
        Assert.IsTrue(tracker.HasUnsavedChanges(ConfigLayer.User));
        Assert.IsFalse(tracker.HasUnsavedChanges(ConfigLayer.Profile));
    }

    [Test]
    public void TrackChange_AddsToProfilePending()
    {
        var tracker = new ChangeTracker();

        tracker.TrackChange(functionId: 1, "shifter_config.gate_width", 12, ConfigLayer.Profile);

        Assert.IsTrue(tracker.HasUnsavedChanges);
        Assert.IsFalse(tracker.HasUnsavedChanges(ConfigLayer.User));
        Assert.IsTrue(tracker.HasUnsavedChanges(ConfigLayer.Profile));
    }

    [Test]
    public void TrackChange_SameFieldTwice_UpdatesValue()
    {
        var tracker = new ChangeTracker();

        tracker.TrackChange(functionId: 1, "output_max", 0.8f, ConfigLayer.User);
        tracker.TrackChange(functionId: 1, "output_max", 0.7f, ConfigLayer.User);

        var pending = tracker.GetPendingChanges(ConfigLayer.User);
        Assert.AreEqual(1, pending.Count);
        Assert.AreEqual(0.7f, pending[1]["output_max"]);
    }

    // === Commit ===

    [Test]
    public void CommitUserChanges_ClearsUserPending()
    {
        var tracker = new ChangeTracker();
        tracker.TrackChange(functionId: 1, "output_max", 0.8f, ConfigLayer.User);
        tracker.TrackChange(functionId: 1, "gate_width", 12, ConfigLayer.Profile);

        tracker.CommitChanges(ConfigLayer.User);

        Assert.IsFalse(tracker.HasUnsavedChanges(ConfigLayer.User));
        Assert.IsTrue(tracker.HasUnsavedChanges(ConfigLayer.Profile));  // untouched
    }

    [Test]
    public void CommitProfileChanges_ClearsProfilePending()
    {
        var tracker = new ChangeTracker();
        tracker.TrackChange(functionId: 1, "output_max", 0.8f, ConfigLayer.User);
        tracker.TrackChange(functionId: 1, "gate_width", 12, ConfigLayer.Profile);

        tracker.CommitChanges(ConfigLayer.Profile);

        Assert.IsTrue(tracker.HasUnsavedChanges(ConfigLayer.User));  // untouched
        Assert.IsFalse(tracker.HasUnsavedChanges(ConfigLayer.Profile));
    }

    // === Discard ===

    [Test]
    public void DiscardAll_ClearsBothLayers()
    {
        var tracker = new ChangeTracker();
        tracker.TrackChange(functionId: 1, "output_max", 0.8f, ConfigLayer.User);
        tracker.TrackChange(functionId: 1, "gate_width", 12, ConfigLayer.Profile);

        tracker.DiscardAll();

        Assert.IsFalse(tracker.HasUnsavedChanges);
    }

    [Test]
    public void DiscardChanges_SingleLayer_OnlyClearsThatLayer()
    {
        var tracker = new ChangeTracker();
        tracker.TrackChange(functionId: 1, "output_max", 0.8f, ConfigLayer.User);
        tracker.TrackChange(functionId: 1, "gate_width", 12, ConfigLayer.Profile);

        tracker.DiscardChanges(ConfigLayer.User);

        Assert.IsFalse(tracker.HasUnsavedChanges(ConfigLayer.User));
        Assert.IsTrue(tracker.HasUnsavedChanges(ConfigLayer.Profile));
    }

    // === Edge Cases ===

    [Test]
    public void HasUnsavedChanges_EmptyTracker_ReturnsFalse()
    {
        var tracker = new ChangeTracker();

        Assert.IsFalse(tracker.HasUnsavedChanges);
    }

    [Test]
    public void GetPendingChanges_EmptyTracker_ReturnsEmptyDict()
    {
        var tracker = new ChangeTracker();

        var pending = tracker.GetPendingChanges(ConfigLayer.User);

        Assert.IsEmpty(pending);
    }
}
```

#### Test Coverage Summary

| Component          | Test Categories                                                     | Priority   |
|--------------------|---------------------------------------------------------------------|------------|
| `ConfigMerger`     | Axis overrides, function deltas, three-layer merge, mutation safety | **High**   |
| `ConfigComparer`   | Equality, floating point, nested structures, arrays, null handling  | **High**   |
| `ConflictDetector` | No conflicts, conflicts, partial overlap, edge cases                | **High**   |
| `FieldRouter`      | User fields, profile fields, hardware fields, unknown fields        | Medium     |
| `DeltaExtractor`   | No changes, single change, multiple changes, nested                 | Medium     |
| `ChangeTracker`    | Track, commit, discard, edge cases                                  | Medium     |

**Recommendation:** Implement tests alongside Phase 1 (merge logic) and Phase 4 (conflict detection). Run tests in CI to catch regressions.

## UI Design

### Layer Indicators

Each config field shows its source via a small badge:

| Badge | Meaning |
|-------|---------|
| (none) | Value = Hardware default |
| `[P]` | Value = Profile override |
| `[U]` | Value = User override |
| `[A]` | Value = Axis hardware params (default for kinematics/static balance) |
| `[F]` | Value = Function override (axis params, replaces `[A]` when active) |

### Axis Parameter Editing in Axis Tab

Since axis parameter controls (kinematics, static balance) live in the axis tab, a **function selector** determines what you're editing:

```
Axis params for: [▼ Axis 2 (base)        ]
                    ─────────────────────
                    Axis 2 (base)
                    FlightStickRoll [F]
                    Collective
```

- **Axis N (base)** — Edit the axis hardware params (`[A]`)
- **FunctionName [F]** — Edit existing parameter override for this axis
- **FunctionName** (no badge) — Function links to this axis but has no override yet

Selecting a function without an override creates one, initialized from the axis base params. The override can later be removed via context menu ("Clear Override").

### Active Functions per Profile

Each vehicle profile includes a list of **active functions** (checkbox/toggle per function):

```csharp
class AircraftFfbProfile {
    // Existing
    string GraphPath;
    Dictionary<string, object> GraphParamValues;
    Dictionary<int, FunctionConfigOverrides> FunctionOverrides;

    // Add: Which functions are active for this profile
    HashSet<int> ActiveFunctionIds;
}
```

On profile load, plugin:

1. Sends axis configs (with parameter overrides merged) for active functions
2. Sends function configs for active functions

### Function Conflict Detection

If multiple **active** functions override the same axis's parameters:

- Show **warning dialog** listing the conflict
- **Lock config updates** on affected axes until resolved
- User must deactivate one of the conflicting functions

```
┌─────────────────────────────────────────────────────────────┐
│  ⚠ Axis Parameter Conflict                                  │
├─────────────────────────────────────────────────────────────┤
│  Multiple active functions override Axis 2 parameters:      │
│    • FlightStickRoll                                        │
│    • FlightStickPitch                                       │
│                                                             │
│  Config updates to Axis 2 are locked until resolved.        │
│  Deactivate one function to continue.                       │
│                                                             │
│                                        [Open Profile...]    │
└─────────────────────────────────────────────────────────────┘
```

### Override Menu

Accessible via **click on badge** or **right-click on field**:

```
Pedal Travel: [  85  ] mm  [P]
                            ↓
                     ┌────────────────────────┐
                     │ ✓ Profile: 85mm        │
                     │   Hardware: 100mm      │
                     │ ──────────────────     │
                     │   Save to User         │
                     │   Reset to Hardware    │
                     └────────────────────────┘
```

Menu options:

- **Current layer** shown with checkmark
- **Lower layers** shown with their values (click to reset)
- **Save to [layer]** to redirect where this value is stored

### Field-Based Default Routing

Each field has a "home layer" — changes auto-save there unless overridden:

| Field Category | Default Target |
|----------------|----------------|
| `output_min`, `output_max`, force scaling | User |
| `ShifterConfig` gates, detent positions | Profile |
| `kinematic_parameters`, `static_balance_config`, `linked_axes` | Hardware |

### Explicit Save Actions

Users can save pending changes at any time via:

- **Save button** in toolbar
- **Per-layer save** via context menu: "Save to User" / "Save to Profile"
- **Keyboard shortcut** (Ctrl+S)

Save button shows indicator when there are unsaved changes (e.g., dot or asterisk).

**Save behavior:**

- If all pending changes have unambiguous target layers (per field-based routing), save immediately
- If any changes have ambiguous targets, show review dialog to confirm destination for each

### Save/Discard Dialog on Context Change

Trigger: User profile change, vehicle change, or plugin exit with unsaved changes.

```
┌─────────────────────────────────────────────────────────────┐
│  Unsaved Changes                                            │
├─────────────────────────────────────────────────────────────┤
│  The following changes have not been saved:                 │
│                                                             │
│  User Preferences:                                          │
│    • Pedal Travel: 85mm → 90mm                              │
│    • Force Max: 50N → 45N                                   │
│                                                             │
│  Profile "IRacing::MX5_Cup":                                │
│    • Shifter Gate Width: 10mm → 12mm                        │
│                                                             │
│              [Discard]  [Save All]  [Review...]             │
└─────────────────────────────────────────────────────────────┘
```

**[Review...]** opens a detailed triage view for pending changes:

```
┌───────────────────────────────────────────────────────────────────────────────┐
│  Review Pending Changes                                                        │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                                │
│  Function Config Changes:                                                      │
│  ────────────────────────                                                      │
│  ☑ Pedal Travel: 85mm → 90mm              [User ▼]          [Discard]         │
│  ☑ Force Max: 50N → 45N                   [User ▼]          [Discard]         │
│  ☑ Shifter Gate Width: 10mm → 12mm        [Profile ▼]       [Discard]         │
│  ☑ Detent Strength: 0.5 → 0.7             [Profile ▼]       [Discard]         │
│                                                                                │
│  Axis Parameter Overrides:                                                     │
│  ─────────────────────────                                                     │
│  ☑ Axis 2 Kinematics                      [FlightStickRoll ▼]  [Discard]      │
│  ☑ Axis 3 StaticBalance                   [Collective ▼]       [Discard]      │
│                                                                                │
│  ─────────────────────────────────────────────────────────────────────────    │
│  Function config dropdown: User | Profile                                      │
│  Axis override dropdown: (list of functions that link to this axis)            │
│                                                                                │
│                                        [Cancel]  [Save Selected]               │
└───────────────────────────────────────────────────────────────────────────────┘
```

Features:

- **Checkbox** — Include/exclude from save
- **Layer dropdown** (function configs) — Re-route change to User or Profile layer
- **Function dropdown** (axis overrides) — Assign override to a specific function
- **Discard button** — Remove individual change from pending list
- **Batch operations** — "Select All User" / "Select All Profile" for quick triage

Useful after legacy config imports where field routing needs manual adjustment.

### Dirty State Tracking

Plugin tracks pending changes per layer:

```csharp
class ConfigChangeTracker {
    Dictionary<FunctionID, FunctionConfigOverrides> PendingUserChanges;
    Dictionary<FunctionID, FunctionConfigOverrides> PendingProfileChanges;

    bool HasUnsavedChanges => PendingUserChanges.Any() || PendingProfileChanges.Any();

    void CommitUserChanges();      // Save to UserPreferences
    void CommitProfileChanges();   // Save to AircraftFfbProfile
    void DiscardAll();
}
```

## Open Questions

1. ~~**Granularity of profile key**~~ - `Game::Vehicle` (resolved)
2. ~~**Conflict resolution**~~ - User wins (resolved)
3. ~~**UI indication**~~ - Badge + context menu (resolved)
4. ~~**Export/import**~~ - Vehicle profiles exclude user layer; user preferences have separate export (resolved)

**Resolved:** ESP32 just receives merged configs from plugin. No layer awareness needed in firmware.

## Future Considerations

Lower-priority items to address if needed:

- **Schema versioning** - Migration path if `UserPreferences` or `FunctionConfigOverrides` structure changes

## Risks

- **Complexity** - Three-layer merge adds cognitive overhead
- **Debugging** - "Why is this value X?" becomes harder to answer
- **Migration** - Existing users have flat configs that need categorization

## Alternatives Considered

### "Profiles All The Way Down"
Make everything a profile, including "default" and "user".
- Rejected: Conflates different concepts, makes "reset to default" unclear.

### "Config Inheritance"
Profile inherits from default, user inherits from profile.
- Similar to Option A but with class-like semantics.
- Rejected: Overcomplicates for minimal benefit.

### "Keep It Flat"
Just have named config files, user manages manually.
- Rejected: Doesn't solve the core UX problems.

## Summary

| Aspect | Decision |
|--------|----------|
| Hierarchy | User > Profile > Default |
| Merge strategy | Delta overlay with type-specific rules |
| User identity | Named profiles (Windows username default) |
| Axis vs Function | Axis params in AxisConfig; functions override per-axis via plugin-side merge |
| Storage | `DiyFfbPluginSettings` (user prefs) + `AircraftFfbProfile` (vehicle) + ESP32 EEPROM (hardware) |

---

**Status:** ✅ Implementation Complete
**Author:** Claude
**Date:** 2026-02-03

## Implementation Status

All core phases implemented on branch `ck_tiered_config`:

| Phase | Description                            | Status  |
| ----- | -------------------------------------- | ------- |
| 1     | Foundation (delta config types, merge) | ✅ Done |
| 2     | Storage (settings integration)         | ✅ Done |
| 3     | Profile Integration (auto-apply)       | ✅ Done |
| 4     | Axis Parameter Overrides (plugin-side) | ✅ Done |
| Tests | Unit tests (80 tests)                  | ✅ Done |

### Deferred (Optional)

| Item                         | Priority | Notes                                    |
| ---------------------------- | -------- | ---------------------------------------- |
| `DeltaExtractor` class       | Medium   | Not needed for current flow              |
| Per-field `[P]`/`[U]` badges | Low      | Implemented in Plan 25 (Phases 1-12)     |
| `ChangeTracker` tests        | ✅ Done  | 26 tests in ChangeTrackerTests.cs        |
| `FieldRouter` tests          | ✅ Done  | 26 tests in FieldRouterTests.cs          |
| Save/Discard Review Dialog   | Planned  | → [Plan 32](32_Badge_Menu_And_Review_Dialog.md) |

See `HANDOFF.md` for full implementation details.
