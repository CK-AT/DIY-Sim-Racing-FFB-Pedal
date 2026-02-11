# Unified Override System: Registry + Layer Badge Control

## Implementation Status

**Last updated:** 2026-02-05

| Phase     | Status          | Description                                              |
|-----------|-----------------|----------------------------------------------------------|
| Phase 1   | ✅ **Complete** | OverrideFieldRegistry + 40 unit tests (172/172 passing)  |
| Phase 2   | ✅ **Complete** | LayerBadgeWrapper WPF control                            |
| Phase 3   | ✅ **Complete** | Event system (ContextChanged + OverrideFieldChanged)     |
| Phase 4   | ✅ **Complete** | Function baseline storage (Hardware layer)               |
| Phase 5   | ✅ **Complete** | Field expansion (17 fields, corrected in Phase 10)       |
| Phase 6   | ✅ **Complete** | Event wiring (context changes + field edits)             |
| Phase 7   | ✅ **Complete** | Baseline integration + "Save as Baseline" button         |
| Phase 8   | ✅ **Complete** | UI integration (badges on all function editors)          |
| Phase 9   | ✅ **Complete** | Badge initialization fix (OnLoaded + SwitchFunction)     |
| Phase 10  | ✅ **Complete** | Registry structure correction (per-field vs whole-config)|
| Phase 11  | ✅ **Complete** | ConfigMerger complete (all function types)               |
| Phase 12  | ✅ **Complete** | Proof-of-concept (simulated_mass with full persistence)  |
| Phase 13  | ✅ **Complete** | Enhanced tooltips (show all layer values)                |
| Phase 14  | ✅ **Complete** | Context menu re-route + review dialog → [Plan 32](32_Badge_Menu_And_Review_Dialog.md) |
| Phase 15  | ✅ **Complete** | All event handlers migrated to override pattern          |

**Files created:**

- `TieredConfig/OverrideFieldRegistry.cs`
- `TieredConfigTests/OverrideFieldRegistryTests.cs`
- `Controls/LayerBadgeWrapper.xaml.cs`
- `Controls/LayerBadgeWrapper.xaml`

**Files modified:**

- `TieredConfig/TieredConfigTypes.cs` (FunctionConfigOverrides expanded, DamperConfigOverrides added)
- `TieredConfig/FieldRouter.cs` (new field routing logic)
- `DiyFfbPlugin.cs` (events + baseline methods)
- `DiyFfbPluginSettings.cs` (FunctionBaselines storage)
- `DiyFfbPlugin.csproj` (project file updates)
- `TieredConfigTests.csproj` (test file updates)
- `TieredConfigTests/OverrideFieldRegistryTests.cs` (removed MinValue/MaxValue tests)

**Registry contents:** 17 fields registered (corrected in Phase 10)

**Build status:** ✅ All code compiles, 172/172 tests passing

## Actual Implementation vs Original Plan

**Major architectural changes discovered during implementation:**

1. **Hybrid two-track system** (not in original plan):
   - `function.Config` = Primary working copy (preserves all edits)
   - `FunctionConfigManager` = Secondary tracker (badge state only)
   - Required to support incremental migration alongside non-wrapped fields

2. **JSON serialization for baselines** (discovered in Phase 7):
   - Protobuf types incompatible with JSON.NET (SimHub's serializer)
   - Solution: Store as JSON strings using Google.Protobuf.JsonFormatter
   - Settings: `Dictionary<int, string>` instead of `Dictionary<int, FunctionConfig>`

3. **Baseline requirement for overrides** (discovered in Phase 12):
   - Creating overrides without baseline caused config corruption
   - Solution: Check `HasFunctionBaseline()` before allowing override creation
   - Workflow: Save baseline first, then create overrides

4. **Manager not used for UI display** (critical discovery):
   - Original plan: UI reads from manager's merged config
   - Actual: UI reads from function.Config (preserves non-wrapped field edits)
   - Manager's merged config used for ESP32 send only

5. **FunctionConfigChanged event handler fix** (Phase 12):
   - Event was overwriting function.Config with merged config
   - Lost all direct edits to non-wrapped fields
   - Solution: Commented out `functions[funcId].Config = e.NewConfig;`

See HANDOFF.md for complete implementation details and all fixes discovered during POC testing.

## LayerBadgeWrapper - Current Implementation vs Plan

**Implemented:**
- ✅ Badge display ([U]/[P] text)
- ✅ Badge color (blue for User, green for Profile)
- ✅ Badge visibility (shown only when override exists)
- ✅ Basic tooltip (field name + layer source)
- ✅ Badge initialization (Plugin/FunctionId setup)
- ✅ Badge refresh (UpdateBadge() on override changes)

**Missing (deferred to Phase 14 → [Plan 32](32_Badge_Menu_And_Review_Dialog.md)):**
- ✅ Enhanced tooltip showing all layer values (User/Profile/Baseline) with active indicator
- ✅ Context menu (right-click on badge)
- ✅ "Clear User override" menu item
- ✅ "Clear Profile override" menu item
- ❌ "Move to User" / "Move to Profile" menu items (re-route between layers)
- ❌ "Save to Baseline" menu item (bake into baseline)
- ❌ Override Review Dialog (see all overrides, batch re-route/discard)

---

## Fields to Wrap - Actual Implementation

**EDIT THIS SECTION** - Complete the list of fields to wrap with badges.

### All Functions

| Field | Path | Type | Layer | Status | Notes |
|-------|------|------|-------|--------|-------|
| Static Balance Enabled | static_balance_tuning.enabled | bool | User | ✅ In registry | |
| Static Balance Gain | static_balance_tuning.gain | float | User | ✅ In registry | |
| Simulated Mass | simulated_mass | float | User | ✅ In registry | |
| Friction | friction | float | User | ✅ In registry | |

### AutomotivePedals (Acc/Brake/Clutch)

| Field | Path | Type | Layer | Status | Notes |
|-------|------|------|-------|--------|-------|
| Force Curve | force_curve | complex | Profile | ⏳ TODO | Wrap entire editor |
| Damper Positive Factor | damper_config.positive_factor | float | User | ⏳ TODO | Add to registry |
| Damper Negative Factor | damper_config.negative_factor | float | User | ⏳ TODO | Add to registry |

### FlightPedals

| Field                     | Path                                  | Type    | Layer   | Status     | Notes                               |
|---------------------------|---------------------------------------|---------|---------|------------|-------------------------------------|
| Motion Range              | flight_pedals.motion_range            | Complex | User    | ⏳ TODO    | Range slider (near/far), FormatValue |
| Damping                   | flight_pedals.damping                 | Float   | User    | ⏳ TODO    | Add to registry                     |
| Centering Spring Constant | flight_pedals.centering_spring_const  | Float   | User    | ⏳ TODO    | Add to registry                     |
| Rudder Brake Force Range  | aux_function.rudder_brake.force_range | Complex | User    | ⏳ TODO    | Range slider, FormatValue           |

### FlightStick (Pitch/Roll/Collective)

| Field                     | Path                                  | Type    | Layer   | Status     | Notes                               |
|---------------------------|---------------------------------------|---------|---------|------------|-------------------------------------|
| Motion Range              | flight_stick_*.motion_range           | Complex | User    | ⏳ TODO    | Range slider (min/max), FormatValue |
| Damping                   | flight_stick_*.damping                | Float   | User    | ⏳ TODO    | Add to registry                     |
| Centering Spring Constant | flight_stick_*.centering_spring_const | Float   | User    | ⏳ TODO    | Add to registry                     |

### Shifter

| Field         | Path           | Type    | Layer   | Status     | Notes             |
|---------------|----------------|---------|---------|------------|-------------------|
| ShifterConfig | shifter_config | Complex | Profile | ⏳ TODO    | Wrap entire panel |


**Notes:**

- OutputMin/OutputMax are NOT exposed in Functions tab (assigned internally)
- All fields use original sliders/controls (not creating new controls)
- Scalar fields (Float/Bool) use simple nullable accessors, straightforward wrapping
- Complex fields need special handling:
  - Range sliders (motion_range, force_range) - wrap single control editing two values, use FormatValue for tooltip
  - Full configs (force_curve, shifter_config) - wrap entire editor panel, use FormatValue or custom tooltip

---

## Problem Statement

### 1. Scattered Field Definitions
Override field definitions are scattered across 5+ files with duplicate switch statements. Adding a new field requires edits in 8+ locations.

### 2. No Layer Visibility in Functions Tab
The Functions tab (primary editor) shows no indication of where values come from (Hardware/Profile/User) or where edits will be saved.

### 3. Two Separate Override Systems

- `FunctionConfigOverrides` — scalar fields (OutputMin, Friction, etc.)
- `AxisParameterOverrides` — complex fields per axis (Kinematics, StaticBalance)

These use different APIs and have no unified layer coordination.

### 4. No Persistent Hardware Baseline for Functions

Function configs are loaded from ESP32 on connect, but the same physical axes can be used by different function types (e.g., axes 2-3 could be an H-pattern shifter OR flight rudder pedals). Importing a new compound config overwrites the ESP32 state, losing the previous baseline.

Without persistent Hardware Presets, users would need overrides for essentially every field to switch between use cases — impractical.

## Solution Overview

### Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ User Overrides       │ Few fields (Friction, OutputMin...)  │
│                      │ Personal preferences, follow user    │
├──────────────────────┼──────────────────────────────────────┤
│ Profile Overrides    │ Few fields (ForceCurve, ShifterCfg)  │
│                      │ Vehicle-specific tuning              │
├──────────────────────┼──────────────────────────────────────┤
│ Hardware Preset      │ Complete FunctionConfig              │
│                      │ Selected per-function, stored in     │
│                      │ SimHub (not ESP32)                   │
└─────────────────────────────────────────────────────────────┘
```

**Function Baselines** are complete `FunctionConfig` snapshots stored in SimHub plugin settings, one per function slot. This is the "Hardware layer" for merge operations. Baselines are updated only by:

- Importing a compound config (replaces baseline)
- Explicit "Save to Hardware" action (bakes override into baseline)

Normal editing in Functions tab routes to User/Profile layers as overrides, NOT to baseline.

**Override Fields** are a small subset of tunable parameters (OutputMin/Max, Friction, SimulatedMass, ForceCurve, ShifterConfig geometry). These are sparse deltas stored per-profile (Profile layer) or per-user (User layer). This is what `OverrideFieldRegistry` manages.

**Active Functions** (existing) determines which functions are enabled for each vehicle profile. The Vehicle Profile tab's activation checkboxes and the Functions tab's "Active for [vehicle]" toggle are two synchronized views of the same underlying state.

### Component Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Unified Override System                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────┐    ┌─────────────────────────────────┐   │
│  │ OverrideFieldRegistry│    │      LayerBadgeWrapper          │   │
│  │                     │    │                                 │   │
│  │ • Field definitions │◄───│ • Displays [U]/[P] badge        │   │
│  │ • Metadata (min/max)│    │ • Routes edits to correct layer │   │
│  │ • Layer routing     │    │ • Context menu (clear/re-route) │   │
│  │ • Accessors         │    │ • Wraps function-level controls │   │
│  └─────────────────────┘    └─────────────────────────────────┘   │
│           │                              │                         │
│           ▼                              ▼                         │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                    Functions Tab (Primary Editor)            │  │
│  │                                                              │  │
│  │  • Function-level fields wrapped with LayerBadgeWrapper      │  │
│  │  • Activation toggle: "Active for [current vehicle]"         │  │
│  │  • "Linked Axes" summary with navigation to Axis tab         │  │
│  │  • Badges show layer source at a glance                      │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                              │                                      │
│           ┌──────────────────┴──────────────────┐                   │
│           │ click ⚙                             │ edits baseline    │
│           ▼                                     ▼                   │
│  ┌────────────────────────┐     ┌────────────────────────────────┐ │
│  │    Axis Tab (Existing) │     │   Function Baseline Storage    │ │
│  │                        │     │                                │ │
│  │  • Kinematics editors  │     │  • FunctionBaselines.json      │ │
│  │  • StaticBalance       │     │  • Complete FunctionConfig     │ │
│  │  • Single-layer        │     │  • One per function slot       │ │
│  └────────────────────────┘     └────────────────────────────────┘ │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
              │
              │  Profile/User overrides = sparse deltas
              ▼
      ┌─────────────────────────────────┐
      │     User/Profile Overrides      │
      │  (FunctionConfigOverrides.json) │
      │  Sparse delta on top of baseline│
      └─────────────────────────────────┘
```

**Note:** Function baselines are persisted in SimHub. Edits in Functions tab update the baseline; importing compound configs replaces it. User/Profile overrides are sparse deltas on top.

## Part 1: Override Field Registry

### Field Scope

The registry covers **function-level overridable fields** with User/Profile/Hardware layering:

**Function-Level Fields (FunctionConfigOverrides)**

| Name | Path | Type | Layer | Group |
|------|------|------|-------|-------|
| OutputMin | output_min | float | User | Output Scaling |
| OutputMax | output_max | float | User | Output Scaling |
| SimulatedMass | simulated_mass | float | User | Physics |
| Friction | friction | float | User | Physics |
| StaticBalanceEnabled | static_balance_tuning.enabled | bool | User | Static Balance Tuning |
| StaticBalanceGain | static_balance_tuning.gain | float | User | Static Balance Tuning |
| ForceCurve | force_curve | complex | Profile | Force Feedback |
| ShifterConfig | shifter_config | complex | Profile | Shifter |

**Axis-Level Fields (AxisParameterOverrides) — Single Layer Only**

Axis overrides are **Hardware-level only** (no User/Profile layering). They represent physical hardware configuration that doesn't vary by user preference or vehicle profile.

| Name | Path | Type | Storage | Group |
|------|------|------|---------|-------|
| Kinematics | kinematic_parameters | complex | Single | Axis Geometry |
| StaticBalance | static_balance_config | complex | Single | Axis Calibration |

These fields are edited in the **Axis tab** (not embedded in Functions tab) and show an `[Override]` badge when a function has customized them.

### Field Types

```csharp
public enum OverrideFieldType
{
    Float,      // Slider + textbox
    Bool,       // Checkbox
    Complex     // Custom editor (force curve, shifter config, kinematics)
}
```

### Registry API

```csharp
public static class OverrideFieldRegistry
{
    // === Field Definitions ===

    public static OverrideFieldDefinition GetField(string fieldPath);
    public static IEnumerable<OverrideFieldDefinition> GetAllFields();
    public static IEnumerable<OverrideFieldDefinition> GetFieldsByGroup(OverrideFieldGroup group);

    // === Layer Routing ===

    public static ConfigLayer GetTargetLayer(string fieldPath);
    public static bool IsUserTunable(string fieldPath);

    // === Value Operations ===

    public static object GetValue(FunctionConfigOverrides overrides, string fieldPath);
    public static void SetValue(FunctionConfigOverrides overrides, string fieldPath, object value);
    public static void ClearValue(FunctionConfigOverrides overrides, string fieldPath);
    public static bool HasValue(FunctionConfigOverrides overrides, string fieldPath);

    // === Path Normalization ===

    public static string NormalizeFieldPath(string fieldName);  // "OutputMin" → "output_min"
}
```

**Note:** The registry covers **function-level fields only** (those with User/Profile/Hardware layering). Axis overrides (`AxisParameterOverrides`) are single-layer and accessed via existing `DiyFfbPlugin` methods (`GetAxisParameterOverride`, `SetAxisParameterOverride`, etc.).

### Field Definition Structure

```csharp
public sealed class OverrideFieldDefinition
{
    // Identity
    public string Name { get; }              // "OutputMin"
    public string FieldPath { get; }         // "output_min"

    // Metadata
    public string DisplayName { get; }       // "Output Min"
    public string Tooltip { get; }
    public OverrideFieldType FieldType { get; }
    public OverrideFieldGroup Group { get; }

    // Routing
    public ConfigLayer DefaultLayer { get; }

    // Validation (for scalar types)
    public float? MinValue { get; }
    public float? MaxValue { get; }

    // Accessors (compiled delegates for FunctionConfigOverrides)
    internal Func<FunctionConfigOverrides, bool> HasValue { get; }
    internal Func<FunctionConfigOverrides, object> GetValue { get; }
    internal Action<FunctionConfigOverrides, object> SetValue { get; }
    internal Action<FunctionConfigOverrides> ClearValue { get; }
}
```

## Part 2: Layer Badge Wrapper Control

A composable WPF control that wraps any editor and provides layer coordination.

### Visual Design

```
┌─────────────────────────────────────────────────────────────┐
│                                                        [U]  │  ← Badge (upper-right)
│  ┌───────────────────────────────────────────────────────┐  │
│  │                                                       │  │
│  │            (any editable control)                     │  │
│  │                                                       │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘

Badge colors:
  [U] = Blue (#64B5F6)  — User override active
  [P] = Green (#4CAF50) — Profile override active
  (no badge)           — Hardware baseline (no override)

Non-override fields (function type, axis assignments) don't use LayerBadgeWrapper
and show no badge — they're Hardware-only with no User/Profile layering.
```

### Control Definition

```csharp
public class LayerBadgeWrapper : ContentControl
{
    // === Configuration ===

    public static readonly DependencyProperty FieldPathProperty;
    public static readonly DependencyProperty FunctionIdProperty;

    public string FieldPath { get; set; }
    public int FunctionId { get; set; }

    // === Computed State ===

    public ConfigLayer? SourceLayer { get; }      // Where current value comes from
    public ConfigLayer TargetLayer { get; }       // Where edits will be saved
    public OverrideFieldDefinition Field { get; } // From registry

    // === Layer Values (for tooltip) ===

    public object UserValue { get; }
    public object ProfileValue { get; }
    public object HardwareValue { get; }
    public object EffectiveValue { get; }  // Merged result

    // === Actions ===

    public void ClearOverride(ConfigLayer layer);
    public void SaveTo(ConfigLayer layer);  // Re-route current value

    // === Events ===

    public event Action<object> ValueChanged;  // Intercepts child edits
}
```

**Note:** `LayerBadgeWrapper` is for **function-level fields only** (User/Profile/Hardware layering). Axis overrides are displayed in the "Linked Axes" summary panel and edited in the Axis tab.

### Usage Examples

```xml
<!-- Function-level scalar field -->
<controls:LayerBadgeWrapper FieldPath="output_min" FunctionId="{Binding FunctionId}">
    <Slider Minimum="0" Maximum="1" Value="{Binding OutputMin}" />
</controls:LayerBadgeWrapper>

<!-- Function-level complex field (force curve) -->
<controls:LayerBadgeWrapper FieldPath="force_curve" FunctionId="{Binding FunctionId}">
    <local:FfbGraphEditor Config="{Binding ForceCurve}" />
</controls:LayerBadgeWrapper>

<!-- Section-level badge (wraps entire group) -->
<controls:LayerBadgeWrapper FieldPath="shifter_config" FunctionId="{Binding FunctionId}">
    <local:ShifterConfigPanel Config="{Binding ShifterConfig}" />
</controls:LayerBadgeWrapper>
```

**Note:** Axis-level fields (kinematics, static balance) are edited in the Axis tab, not wrapped with LayerBadgeWrapper in the Functions tab. See "Linked Axes" section below.

### Complex Editor Integration

For Complex types, `LayerBadgeWrapper` only handles badge display and context menu — it does NOT intercept edits. The **parent code-behind** subscribes to change events and routes to the correct layer.

Pattern: "provide effective config, receive change events":

**Required changes to complex editors:**

```csharp
// Add to SplineForceCurve.xaml.cs
public event EventHandler<SplineForceCurveConfig> ConfigChanged;

// Fire on drag end (Rectangle_MouseLeftButtonUp)
private void Rectangle_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
{
    // ... existing code ...
    ConfigChanged?.Invoke(this, config);
}

// Fire on range slider changes
private void Rangeslider_travel_range_LowerValueChanged(...)
{
    // ... existing code ...
    ConfigChanged?.Invoke(this, config);
}
```

**Wrapper integration (code-behind):**

```csharp
// In function editor
private void SetupForceCurveWrapper()
{
    // Provide effective config to editor
    var effectiveConfig = GetEffectiveForceCurveConfig(FunctionId);
    ForceCurveEditor.UpdateConfig(effectiveConfig);

    // Subscribe to changes
    ForceCurveEditor.ConfigChanged += (sender, newConfig) =>
    {
        // Route full config to target layer (Profile for force curves)
        Plugin.SetForceCurveOverride(FunctionId, newConfig, ConfigLayer.Profile);
        ForceCurveWrapper.UpdateBadge();
    };
}
```

**Complex editors requiring this pattern:**

| Editor                 | Field Path     | Target Layer |
|------------------------|----------------|--------------|
| `SplineForceCurve`     | force_curve    | Profile      |
| `ShifterConfigControl` | shifter_config | Profile      |

Scalar fields (Float, Bool) use direct two-way binding and don't need this pattern.

### ESP32 Send Policy

**Key design decision:** ESP32 config sends are **manual**, not automatic during edits.

| Action | Local storage | Badge update | ESP32 send |
|--------|---------------|--------------|------------|
| Edit override field | ✓ Immediate | ✓ Immediate | ✗ No |
| Clear override | ✓ Immediate | ✓ Immediate | ✗ No |
| Click "Upload Changes" | — | — | ✓ Merged config |
| Profile/vehicle switch | — | ✓ Full refresh | ✓ Merged config |

**"Upload Changes" button** in Vehicle Profile tab:
- Visible when there are unapplied override changes (dirty state tracking)
- Merges all layers for active functions, sends to ESP32
- Existing per-function "Upload" button logic unchanged

This separates UI editing (immediate feedback) from hardware communication (explicit user action).

### Context Change Handling

When profile/user changes, all override fields must refresh to show new effective values.

**Pattern:** Single refresh method in function editor, visual tree search for wrappers.

```csharp
// In function editor
private void OnLoaded(object sender, RoutedEventArgs e)
{
    Plugin.ActiveContextChanged += OnActiveContextChanged;
}

private void OnActiveContextChanged(object sender, EventArgs e)
{
    RefreshAllOverrideFields();
}

private void RefreshAllOverrideFields()
{
    // Complex editors — push new effective config
    ForceCurveEditor.UpdateConfig(GetEffectiveForceCurve(FunctionId));
    ShifterEditor.UpdateConfig(GetEffectiveShifterConfig(FunctionId));

    // All badges — visual tree search, no manual registration
    foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
        wrapper.UpdateBadge();
}

private static IEnumerable<T> FindVisualChildren<T>(DependencyObject parent) where T : DependencyObject
{
    for (int i = 0; i < VisualTreeHelper.GetChildrenCount(parent); i++)
    {
        var child = VisualTreeHelper.GetChild(parent, i);
        if (child is T t) yield return t;
        foreach (var descendant in FindVisualChildren<T>(child))
            yield return descendant;
    }
}
```

**Plugin events:**

```csharp
// In DiyFfbPlugin.cs

// Full context change — profile/user/vehicle switch
public event EventHandler ContextChanged;

// Targeted field change — single override edit (NO ESP32 send)
public event EventHandler<OverrideFieldChangedEventArgs> OverrideFieldChanged;

public class OverrideFieldChangedEventArgs : EventArgs
{
    public int FunctionId { get; set; }
    public string FieldPath { get; set; }
}

// Fire ContextChanged on:
// - Profile switch
// - User switch
// - Vehicle change
// - "Upload Changes" button click (after ESP32 send)

// Fire OverrideFieldChanged on:
// - Override added/modified/cleared (NO ESP32 send — local only)
```

**Two-tier event design:**

| Event | Fires when | ESP32 send | Subscriber action |
|-------|------------|------------|-------------------|
| `ContextChanged` | Profile/user/vehicle switch, Upload | Yes | Full badge refresh |
| `OverrideFieldChanged` | Field edit | **No** | Targeted badge refresh |

**Broader usage:** These events replace scattered refresh logic across all context-dependent UI components:

| Subscriber | Refresh action |
|------------|----------------|
| Function editor | Override field values, badges, complex editors |
| Vehicle Profile tab | Override summaries, activation state display |
| Axis tab | Function override dropdown state |
| Any standalone graph display | Effective curve |

Subscribers don't need to know *why* effective config changed — just that it did.

### ShifterConfigControl Strategy

**Decision:** Keep ShifterConfigControl unified (no split), but extract misplaced fields.

**Analysis:** The control currently mixes three concerns:

1. **Axis Assignment** (Hardware) — X/Y axis selectors, Sequential mode checkbox
2. **Shift Geometry** (Profile) — Motion range, gates, detents, slots, damping, force params
3. **Function Physics** (User) — Friction, SimulatedMass (these are `FunctionConfig`-level, not shifter-specific)

Splitting axis assignment from geometry was considered but rejected due to tight coupling:

- Sequential mode stored in `shifter_config.Sequential` but affects axis mapping
- Motion range sliders constrained by axis travel limits
- Preview canvas needs both axis state and geometry data

**Approach:**

1. **Extract Friction/SimulatedMass** — Move to separate controls in the function editor with `[U]` badges. They don't belong in shifter config.

2. **Keep ShifterConfigControl unified** — Wrap entire control with `LayerBadgeWrapper` showing `[P]` for geometry.

3. **Add visual Hardware indicator** — Mark axis assignment section with a subtle "Hardware" label (not a layer badge, just informational text).

4. **Add `GeometryChanged` event** — Fire when any geometry field changes (not axis assignment):

   ```csharp
   public event EventHandler<ShifterConfig> GeometryChanged;
   ```

5. **Parent routes changes to layers:**
   - Axis assignment changes → Hardware (existing `function?.OnAxisUpdate()` path)
   - Geometry changes → Profile layer via `GeometryChanged` event

**Visual result:**

```
┌─────────────────────────────────────────────────────────────┐
│ Shifter Config                                         [P]  │
├─────────────────────────────────────────────────────────────┤
│ Axis Assignment                              (Hardware)     │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ X Axis: [Dropdown]   Y Axis: [Dropdown]                 │ │
│ │ [x] Sequential mode                                     │ │
│ └─────────────────────────────────────────────────────────┘ │
│                                                             │
│ Motion Range                                                │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ X: [====●====●====]  Y: [====●====●====]                │ │
│ └─────────────────────────────────────────────────────────┘ │
│ ... (gates, detents, slots, preview) ...                    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Physics                                                     │
│ Friction      [====●===========]  2.5               [U]     │
│ Simulated Mass [===●============]  0.15 kg          [U]     │
└─────────────────────────────────────────────────────────────┘
```

**Files affected:**

- `ShifterConfigControl.xaml` — Add "Hardware" label to axis section, remove Friction/SimulatedMass
- `ShifterConfigControl.xaml.cs` — Add `GeometryChanged` event, remove Friction/SimulatedMass handlers
- Function editor — Add separate Physics section with Friction/SimulatedMass + badges

### Context Menu

Right-click on badge or control shows:

```
┌────────────────────────────────────┐
│ ✓ User override: 0.2               │  ← Current source (checked)
│   Profile value: 0.1               │
│   Hardware baseline: 0.0           │
├────────────────────────────────────┤
│   Clear User override              │
│   Clear Profile override           │
├────────────────────────────────────┤
│   Save to User                     │
│   Save to Profile                  │
│   Save to Hardware                 │  ← Updates baseline, clears source override
└────────────────────────────────────┘
```

**"Save to Hardware" behavior:**

1. Write current effective value to the function baseline
2. Clear the override from whichever layer provided the value

This "bakes" an override into the baseline, making it the new default.

### Tooltip

Hover shows all layer values.

**Scalar fields (Float, Bool):**

```
Output Min
─────────────────────
User:     0.2 ◄ active
Profile:  0.1
Hardware: 0.0
```

**Complex fields (ForceCurve, ShifterConfig):**

```
Force Curve
─────────────────────
User:     (not set)
Profile:  (configured) ◄ active
Hardware: (configured)
```

Complex fields show presence/absence at each layer, not the actual values (which can't be displayed inline).

## Part 3: Functions Tab Refactor

### Current State

- Functions tab: Full editor, no layer visibility
- Vehicle Profile tab: Activation toggles + limited override editor with badges

### Target State

- Functions tab: Full editor with badges everywhere + activation toggle
- Vehicle Profile tab: Vehicle selection, quick overview (optional)

### Functions Tab Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│ Function: Shifter (H-Pattern)                    [Active for MX-5 ✓]│
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│ Output Scaling                                                      │
│ ┌─────────────────────────────────────────────────────────────┐    │
│ │ Output Min    [====●===========]  0.20               [U]    │    │
│ │ Output Max    [===============●]  1.00                      │    │
│ └─────────────────────────────────────────────────────────────┘    │
│                                                                     │
│ Physics                                                             │
│ ┌─────────────────────────────────────────────────────────────┐    │
│ │ Simulated Mass [===●============]  15.0 kg           [P]    │    │
│ │ Friction       [=●==============]  2.5               [U]    │    │
│ └─────────────────────────────────────────────────────────────┘    │
│                                                                     │
│ Force Curve                                                    [P]  │
│ ┌─────────────────────────────────────────────────────────────┐    │
│ │                    (Force Curve Editor)                     │    │
│ └─────────────────────────────────────────────────────────────┘    │
│                                                                     │
│ Shifter Config                                                 [P]  │
│ ┌─────────────────────────────────────────────────────────────┐    │
│ │ Gate Width    [======●=========]  12.5 mm                   │    │
│ │ Gate Depth    [========●=======]  18.0 mm                   │    │
│ │ ...                                                         │    │
│ └─────────────────────────────────────────────────────────────┘    │
│                                                                     │
│ Linked Axes                                                         │
│ ┌─────────────────────────────────────────────────────────────┐    │
│ │ Axis 0 (X)  Kinematics [Override]  StaticBal [Default]   ⚙ │    │
│ │ Axis 1 (Y)  Kinematics [Default]   StaticBal [Default]   ⚙ │    │
│ └─────────────────────────────────────────────────────────────┘    │
│                              (⚙ = click to edit in Axis tab)        │
└─────────────────────────────────────────────────────────────────────┘
```

**Linked Axes section:**
- Shows summary of axis overrides for this function (not embedded editors)
- `[Override]` badge when function has customized kinematics/static balance
- `[Default]` or no badge when using base axis config
- Click ⚙ navigates to Axis tab with this function pre-selected in dropdown

### Activation Toggle

The "Active for [vehicle]" checkbox in the function header:
- Shows current vehicle profile name
- Toggles function activation for that profile
- Synchronized with Vehicle Profile tab's activation checkboxes (same underlying state, two views)

## Implementation Plan

### Phase 1: Core Infrastructure

**1.1 Create OverrideFieldRegistry.cs**

```csharp
// SimHubPlugin/TieredConfig/OverrideFieldRegistry.cs

public static class OverrideFieldRegistry
{
    static OverrideFieldRegistry()
    {
        // Output Scaling
        RegisterField(new OverrideFieldDefinition
        {
            Name = "OutputMin",
            FieldPath = "output_min",
            DisplayName = "Output Min",
            FieldType = OverrideFieldType.Float,
            Group = OverrideFieldGroup.OutputScaling,
            DefaultLayer = ConfigLayer.User,
            MinValue = 0f,
            MaxValue = 1f,
            Tooltip = "Minimum output value (0-1)",
            HasValue = o => o.OutputMin.HasValue,
            GetValue = o => o.OutputMin,
            SetValue = (o, v) => o.OutputMin = (float?)v,
            ClearValue = o => o.OutputMin = null,
        });

        // ... OutputMax, SimulatedMass, Friction ...

        // Static Balance Tuning (nested)
        RegisterField(new OverrideFieldDefinition
        {
            Name = "StaticBalanceEnabled",
            FieldPath = "static_balance_tuning.enabled",
            DisplayName = "Enabled",
            FieldType = OverrideFieldType.Bool,
            Group = OverrideFieldGroup.StaticBalanceTuning,
            DefaultLayer = ConfigLayer.User,
            Tooltip = "Enable static balance compensation",
            HasValue = o => o.StaticBalanceTuning?.Enabled.HasValue == true,
            GetValue = o => o.StaticBalanceTuning?.Enabled,
            SetValue = (o, v) => {
                o.StaticBalanceTuning ??= new StaticBalanceTuningOverrides();
                o.StaticBalanceTuning.Enabled = (bool?)v;
            },
            ClearValue = o => {
                if (o.StaticBalanceTuning != null) {
                    o.StaticBalanceTuning.Enabled = null;
                    if (o.StaticBalanceTuning.IsEmpty) o.StaticBalanceTuning = null;
                }
            },
        });

        // ... StaticBalanceGain, ForceCurve, ShifterConfig ...
    }
}
```

**Note:** Axis-level fields (Kinematics, StaticBalance) are **not** registered — they use existing `AxisParameterOverrides` API without layering.

**1.2 Refactor existing code to use registry**

- `FieldRouter.cs` → delegates to `OverrideFieldRegistry.GetTargetLayer()`
- `ConfigLayerProvider.cs` → delegates to `OverrideFieldRegistry.HasValue()`
- `DiyFfbPlugin.cs` → removes switch statements, uses registry

### Phase 2: Layer Badge Wrapper Control

**2.1 Create LayerBadgeWrapper.cs**

```csharp
// SimHubPlugin/Controls/LayerBadgeWrapper.cs

public class LayerBadgeWrapper : ContentControl
{
    private Border _badge;
    private TextBlock _badgeText;
    private ContextMenu _contextMenu;

    public LayerBadgeWrapper()
    {
        // Build visual tree: Grid with Content + Badge overlay
    }

    private void UpdateBadge()
    {
        var layer = GetSourceLayer();
        _badgeText.Text = ConfigLayerProvider.GetLayerBadgeText(layer);
        _badge.Background = GetLayerColor(layer);
        _badge.Visibility = layer.HasValue ? Visibility.Visible : Visibility.Collapsed;
    }

    private ConfigLayer? GetSourceLayer()
    {
        var field = OverrideFieldRegistry.GetField(FieldPath);
        if (field == null) return null;

        if (AxisId.HasValue)
            return _layerProvider.GetAxisFieldSourceLayer(FunctionId, AxisId.Value, FieldPath);
        else
            return _layerProvider.GetFieldSourceLayer(FunctionId, FieldPath);
    }

    private void OnClearOverride(ConfigLayer layer) { /* ... */ }
    private void OnSaveTo(ConfigLayer layer) { /* ... */ }
}
```

**2.2 Create LayerBadgeWrapper.xaml (ControlTemplate)**

```xml
<ControlTemplate TargetType="controls:LayerBadgeWrapper">
    <Grid>
        <ContentPresenter />
        <Border x:Name="PART_Badge"
                HorizontalAlignment="Right"
                VerticalAlignment="Top"
                CornerRadius="2"
                Padding="4,1"
                Margin="2">
            <TextBlock x:Name="PART_BadgeText"
                       FontSize="10"
                       FontWeight="Bold"
                       Foreground="White"/>
        </Border>
    </Grid>
</ControlTemplate>
```

### Phase 3: Functions Tab Integration

**3.1 Add activation toggle to function header**

**3.2 Wrap existing controls with LayerBadgeWrapper**

Incremental approach — wrap controls one section at a time:

1. Output Scaling fields (simplest, existing)
2. Physics fields
3. Static Balance Tuning
4. Force Curve (graph editor)
5. Shifter Config

**3.3 Add "Linked Axes" summary panel**

```csharp
// New API in DiyFfbPlugin.cs
public List<AxisOverrideSummary> GetAxisOverrideSummaries(int functionId)
{
    var links = GetFunctionsLinkingToAxis(...);
    return links.Select(link => new AxisOverrideSummary
    {
        AxisId = link.AxisId,
        AxisName = GetAxisName(link.AxisId),
        HasKinematicsOverride = HasAxisParameterOverride(functionId, link.AxisId)
            && GetAxisParameterOverride(functionId, link.AxisId)?.Kinematics != null,
        HasStaticBalanceOverride = HasAxisParameterOverride(functionId, link.AxisId)
            && GetAxisParameterOverride(functionId, link.AxisId)?.StaticBalance != null,
    }).ToList();
}

public class AxisOverrideSummary
{
    public int AxisId { get; set; }
    public string AxisName { get; set; }
    public bool HasKinematicsOverride { get; set; }
    public bool HasStaticBalanceOverride { get; set; }
}
```

**3.4 Add navigation to Axis tab**

```csharp
// In function editor — when user clicks ⚙ button
private void OnEditAxisOverride(int axisId, int functionId)
{
    // Switch to Axis tab
    MainTabControl.SelectedIndex = AXIS_TAB_INDEX;

    // Pre-select this function in the axis control's dropdown
    AxisConfigControl.SelectFunctionOverride(axisId, functionId);
}

// In AxisConfigControl.xaml.cs — new public method
public void SelectFunctionOverride(int axisId, int functionId)
{
    // Switch to correct axis if multi-axis UI
    SelectAxis(axisId);

    // Find function in dropdown, select it
    var item = _functionSelectorItems.FirstOrDefault(i => i.FunctionId == functionId);
    if (item != null)
        FunctionSelector.SelectedItem = item;
}
```

### Phase 4: ConfigLayerProvider Extensions

```csharp
public class ConfigLayerProvider
{
    // Existing
    public ConfigLayer? GetFieldSourceLayer(int functionId, string fieldPath);

    // New: Get all layer values for tooltip display
    public LayerValues GetAllLayerValues(int functionId, string fieldPath);
}

public class LayerValues
{
    public object UserValue { get; set; }
    public object ProfileValue { get; set; }
    public object HardwareValue { get; set; }
    public object EffectiveValue { get; set; }
    public ConfigLayer? SourceLayer { get; set; }
}
```

**Note:** Axis overrides don't use ConfigLayerProvider — they're single-layer (no User/Profile distinction). The "Linked Axes" panel queries `HasAxisParameterOverride()` directly.

### Phase 5: Unit Tests

**New test file: `TieredConfigTests/OverrideFieldRegistryTests.cs`**

- All function-level fields registered
- Lookup by name and path (case-insensitive)
- Layer routing matches expected values (User/Profile)
- Value get/set/clear operations work for float/bool/complex types
- Group filtering works
- Nested field accessors work correctly (StaticBalanceTuning)

## Files to Create/Modify

| File | Action |
|------|--------|
| `TieredConfig/OverrideFieldRegistry.cs` | **Create** — unified field registry |
| `Controls/LayerBadgeWrapper.cs` | **Create** — composable badge control |
| `Controls/LayerBadgeWrapper.xaml` | **Create** — control template |
| `TieredConfig/FieldRouter.cs` | Refactor — delegate to registry |
| `TieredConfig/ConfigLayerProvider.cs` | Extend — all-values query for tooltips |
| `DiyFfbPlugin.cs` | Refactor — remove switches, add `GetAxisOverrideSummaries()`, add `ContextChanged`/`OverrideFieldChanged` events |
| `DiyFfbPluginUI.xaml.cs` | Refactor — wrap controls with badges, add "Upload Changes" button |
| `FunctionEditorControl.xaml.cs` | Add activation toggle + Linked Axes panel |
| `AxisConfigControl.xaml.cs` | Add `SelectFunctionOverride()` for navigation |
| `SplineForceCurve.xaml.cs` | Add `ConfigChanged` event for wrapper integration |
| `TieredConfigTests/OverrideFieldRegistryTests.cs` | **Create** |

## Adding a New Override Field (After)

1. Add property to `FunctionConfigOverrides` or `AxisParameterOverrides`
2. Add field definition to `OverrideFieldRegistry`
3. Wrap the UI control with `LayerBadgeWrapper`

**Done.** (vs 8+ scattered locations before)

## Migration Strategy

### Incremental Rollout

1. **Phase 1**: Registry + refactor existing code (no UI changes)
2. **Phase 2**: LayerBadgeWrapper control (tested in isolation)
3. **Phase 3**: Wrap scalar fields in Vehicle Profile tab (existing badges → new control)
4. **Phase 4**: Wrap function-level fields in Functions tab (new capability)
5. **Phase 5**: Add "Linked Axes" summary panel + navigation to Axis tab
6. **Phase 6**: Add activation toggle to Functions tab
7. **Phase 7**: Simplify Vehicle Profile tab (optional)

Each phase is independently testable and deployable.

### Axis Override Strategy (Option B)

Axis overrides (Kinematics, StaticBalance) remain **single-layer** and are edited in the **Axis tab**:

- Functions tab shows summary badges (`[Override]` / `[Default]`)
- Click navigates to Axis tab with function pre-selected
- No embedded kinematics editors in function controls
- Existing `AxisConfigControl` function selector dropdown continues to work

This avoids complexity of layered axis storage while providing visibility in the Functions tab.

## Verification

1. **Unit tests**: Registry tests pass
2. **Existing tests**: 132 tiered config tests still pass
3. **Build**: `MSBuild DiyFfbPlugin.csproj`
4. **Manual test**:
   - Badges appear on all wrapped function-level controls
   - Correct layer shown ([U]/[P] for function fields)
   - Tooltip shows all layer values
   - Context menu clear/re-route works
   - Linked Axes panel shows override status correctly
   - Click on axis ⚙ navigates to Axis tab with function selected
   - Activation toggle works in Functions tab
   - Values persist after save/reload

## Risk Assessment

**Medium risk** — UI changes but behavior preserved:

- Registry refactor is low risk (pure consolidation)
- New wrapper control is additive (doesn't break existing)
- Functions tab changes are incremental (wrap controls one at a time)
- Existing tests provide safety net

## Future Extensions

1. **Named Presets**: Save/load multiple named baseline configs per function for quick switching between use cases (racing shifter ↔ flight rudder).
2. **Batch operations**: "Clear all User overrides" / "Reset to Profile"
3. **Diff view**: Show what changed vs baseline
4. **Import/Export**: Serialize overrides using registry metadata
5. **Undo/Redo**: Track changes with layer-aware history
6. **Profile comparison**: Side-by-side layer values across profiles
