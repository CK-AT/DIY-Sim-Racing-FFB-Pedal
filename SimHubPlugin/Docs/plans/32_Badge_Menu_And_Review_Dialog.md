# Badge Context Menu & Override Review Dialog

**Date:** 2026-02-10
**Status:** Planned
**Branch:** `ck_tiered_config`
**Depends on:** [Plan 24](24_Tiered_Config_Overrides.md) (tiered config foundation),
[Plan 25](25_Override_Field_Registry_Plan.md) (override field registry + badge wrapper)

## Problem Statement

The override badge system (Plan 25) displays `[U]`/`[P]` badges showing which layer owns a value, with a context menu that can **clear** overrides. Two capabilities are missing:

1. **Re-routing**: The badge context menu has no "Save to User" / "Save to Profile" / "Save to Baseline" actions. Users can't move an override from one layer to another — they must clear it and re-create it manually. (Plan 25, Phase 14 TODO)

2. **Override Review**: There is no way to see all active overrides across all functions at a glance. Users can't audit which values are overridden, clean up stale overrides, or re-route multiple fields in batch. (Plan 24, deferred "Save/Discard Review Dialog")

## Scope

### Part 1: Badge Context Menu — "Save to Layer" Operations

Extend the existing `LayerBadgeWrapper` context menu (currently has clear operations only) with re-routing and baseline-bake actions.

### Part 2: Override Review Dialog

New WPF dialog showing all active overrides across all functions, with per-item re-route/discard and batch operations.

## Part 1: Badge Context Menu Enhancement

### Current State

[LayerBadgeWrapper.xaml.cs:270-336](SimHubPlugin/Controls/LayerBadgeWrapper.xaml.cs#L270-L336) — context menu has:
- Section 1: Layer values (User/Profile/Baseline with checkmark on active)
- Section 2: Clear operations ("Clear User override", "Clear Profile override")
- Section 3: **TODO** — "Save to User", "Save to Profile", "Save to Baseline"

### Target Menu

```
┌──────────────────────────────────┐
│ ✓ User: 0.20                     │  ← non-clickable header
│   Profile: (not set)             │
│   Baseline: 0.00                 │
├──────────────────────────────────┤
│   Clear User override            │  ← existing (shown when layer has value)
├──────────────────────────────────┤
│   Move to Profile                │  ← NEW: re-route from User → Profile
│   Save to Baseline               │  ← NEW: bake into baseline, clear override
└──────────────────────────────────┘
```

When active layer is `[P]` (Profile):

```
┌──────────────────────────────────┐
│   User: (not set)                │
│ ✓ Profile: 12.5mm               │
│   Baseline: 10.0mm              │
├──────────────────────────────────┤
│   Clear Profile override         │
├──────────────────────────────────┤
│   Move to User                   │  ← re-route from Profile → User
│   Save to Baseline               │  ← bake into baseline, clear override
└──────────────────────────────────┘
```

### Menu Item Rules

| Item | Shown when | Action |
|------|-----------|--------|
| "Clear User override" | User layer has value | Remove from User overrides |
| "Clear Profile override" | Profile layer has value | Remove from Profile overrides |
| "Move to User" | Profile layer has value AND User layer is empty | Copy value to User, clear from Profile |
| "Move to Profile" | User layer has value AND Profile layer is empty | Copy value to User, clear from User |
| "Save to Baseline" | Any override exists (User or Profile) | Bake effective value into baseline, clear source override |

**"Move to" is hidden when the target layer already has a value** — to prevent silent overwrite. The review dialog handles conflicts.

**"Save to Baseline" behavior:**

1. Read the current effective value for this field (from merged config)
2. Write that value into the function baseline (`SetFunctionBaseline`)
3. Clear the override from whichever layer provided it
4. Update badge (should disappear since value is now in baseline)

### Orchestrator API

New method on `TieredConfigOrchestrator`:

```csharp
/// <summary>
/// Move an override value from one layer to another.
/// Reads the value from sourceLayer, writes to targetLayer, clears from sourceLayer.
/// Re-applies merged overrides afterward.
/// </summary>
public void RerouteFunctionOverrideField(
    int functionId, string fieldPath,
    ConfigLayer sourceLayer, ConfigLayer targetLayer)
{
    // 1. Read value from source
    var sourceOverrides = GetOverridesForLayer(functionId, sourceLayer);
    if (sourceOverrides == null) return;
    var value = OverrideFieldRegistry.GetValue(sourceOverrides, fieldPath);
    if (value == null) return;

    // 2. Write to target
    var targetOverrides = GetOrCreateOverridesForLayer(functionId, targetLayer);
    OverrideFieldRegistry.SetValue(targetOverrides, fieldPath, value);

    // 3. Clear from source
    OverrideFieldRegistry.ClearValue(sourceOverrides, fieldPath);
    CleanupEmptyOverrides(functionId, sourceLayer, sourceOverrides);

    // 4. Re-apply merged config
    if (_functionConfigManager.HasBaseConfig(functionId))
        ReapplyMergedOverrides(functionId, diffCheck: false);

    // 5. Notify UI
    OnOverrideFieldChanged(functionId, fieldPath);
}

/// <summary>
/// Bake a field's current effective value into the baseline, clearing the override.
/// </summary>
public void BakeFieldToBaseline(int functionId, string fieldPath)
{
    // 1. Get current effective value from merged FunctionConfig
    var currentConfig = _functionConfigManager.GetCurrentConfig(functionId);
    if (currentConfig == null) return;

    // 2. Get baseline, write the field value into it
    var baseline = GetFunctionBaseline(functionId);
    if (baseline == null) return;

    var updatedBaseline = baseline.Clone();
    WriteFieldToFunctionConfig(updatedBaseline, fieldPath, currentConfig);
    SetFunctionBaseline(functionId, updatedBaseline);

    // 3. Clear override from whichever layer had it
    ClearFunctionOverrideField(functionId, fieldPath, ConfigLayer.User);
    ClearFunctionOverrideField(functionId, fieldPath, ConfigLayer.Profile);

    // 4. Notify UI
    OnOverrideFieldChanged(functionId, fieldPath);
}
```

Helper methods needed:

```csharp
private FunctionConfigOverrides GetOverridesForLayer(int functionId, ConfigLayer layer)
{
    return layer == ConfigLayer.User
        ? GetUserFunctionOverrides(functionId)
        : GetFunctionOverrides(functionId);
}

private FunctionConfigOverrides GetOrCreateOverridesForLayer(int functionId, ConfigLayer layer)
{
    return layer == ConfigLayer.User
        ? GetOrCreateUserFunctionOverrides(functionId)
        : GetOrCreateFunctionOverrides(functionId);
}

private void CleanupEmptyOverrides(int functionId, ConfigLayer layer, FunctionConfigOverrides overrides)
{
    if (!overrides.IsEmpty) return;
    if (layer == ConfigLayer.User)
    {
        var prefs = GetCurrentUserOverrides();
        prefs?.FunctionOverrides?.Remove(functionId);
    }
    else
    {
        var profile = _getActiveProfile();
        profile?.FunctionOverrides?.Remove(functionId);
    }
}
```

### LayerBadgeWrapper Changes

In `OnBadgeRightClick`, replace the TODO section (line 330-331) with:

```csharp
// Section 3: Re-route operations
var sourceLayer = layerProvider.GetFieldSourceLayer(FunctionId, FieldPath);
if (sourceLayer == ConfigLayer.User)
{
    // Offer to move to Profile (only if Profile doesn't already have a value)
    if (!layerProvider.HasFieldValue(FunctionId, FieldPath, ConfigLayer.Profile))
    {
        var moveToProfile = new MenuItem { Header = "Move to Profile" };
        moveToProfile.Click += (s, args) => OnRerouteOverride(ConfigLayer.User, ConfigLayer.Profile);
        contextMenu.Items.Add(moveToProfile);
    }
}
else if (sourceLayer == ConfigLayer.Profile)
{
    // Offer to move to User (only if User doesn't already have a value)
    if (!layerProvider.HasFieldValue(FunctionId, FieldPath, ConfigLayer.User))
    {
        var moveToUser = new MenuItem { Header = "Move to User" };
        moveToUser.Click += (s, args) => OnRerouteOverride(ConfigLayer.Profile, ConfigLayer.User);
        contextMenu.Items.Add(moveToUser);
    }
}

// Save to Baseline (always available when any override exists)
if (sourceLayer == ConfigLayer.User || sourceLayer == ConfigLayer.Profile)
{
    var saveToBaseline = new MenuItem { Header = "Save to Baseline" };
    saveToBaseline.Click += (s, args) => OnBakeToBaseline();
    contextMenu.Items.Add(saveToBaseline);
}
```

New event and handler methods:

```csharp
/// <summary>
/// Fired when an override is re-routed to a different layer.
/// Parent control should update the UI to reflect the new effective value.
/// </summary>
public event EventHandler<OverrideReroutedEventArgs> OverrideRerouted;

public class OverrideReroutedEventArgs : EventArgs
{
    public string FieldPath { get; set; }
    public ConfigLayer FromLayer { get; set; }
    public ConfigLayer ToLayer { get; set; }
}

private void OnRerouteOverride(ConfigLayer fromLayer, ConfigLayer toLayer)
{
    if (Plugin == null || FunctionId < 0 || string.IsNullOrEmpty(FieldPath))
        return;

    Plugin.ConfigOrchestrator.RerouteFunctionOverrideField(FunctionId, FieldPath, fromLayer, toLayer);
    UpdateBadge();
    UpdateTooltip();

    OverrideRerouted?.Invoke(this, new OverrideReroutedEventArgs
    {
        FieldPath = FieldPath,
        FromLayer = fromLayer,
        ToLayer = toLayer
    });
}

private void OnBakeToBaseline()
{
    if (Plugin == null || FunctionId < 0 || string.IsNullOrEmpty(FieldPath))
        return;

    Plugin.ConfigOrchestrator.BakeFieldToBaseline(FunctionId, FieldPath);
    UpdateBadge();
    UpdateTooltip();

    OverrideCleared?.Invoke(this, new OverrideClearedEventArgs
    {
        FieldPath = FieldPath,
        ClearedLayer = ConfigLayer.Baseline // signals "baked to baseline"
    });
}
```

## Part 2: Override Review Dialog

### Purpose

A standalone dialog that shows **all active overrides** across all functions, grouped by function, with the ability to:

- See at a glance which values are overridden and at which layer
- Re-route individual overrides to a different layer (User ↔ Profile)
- Discard individual overrides
- Batch discard by layer ("Clear all User overrides", "Clear all Profile overrides")
- Bake overrides into baseline

### Dialog Layout

```
┌───────────────────────────────────────────────────────────────────────┐
│  Override Review                                                   X  │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Function    Field              Value       Layer     Actions         │
│  ─────────────────────────────────────────────────────────────────    │
│  Brake       Simulated Mass     15.0 kg     [U ▼]    [Bake] [Clear]  │
│  Brake       Friction           2.50        [U ▼]    [Bake] [Clear]  │
│  Brake       Force Curve        (configured) [P ▼]   [Bake] [Clear]  │
│  Shifter     Shifter Config     (configured) [P ▼]   [Bake] [Clear]  │
│                                                                       │
│  ─────────────────────────────────────────────────────────────────    │
│  4 overrides across 2 functions                                       │
│                                                                       │
│  [Clear All User]  [Clear All Profile]            [Close]             │
└───────────────────────────────────────────────────────────────────────┘
```

### Data Model

```csharp
/// <summary>
/// ViewModel row for the override review dialog.
/// </summary>
public class OverrideReviewItem
{
    public int FunctionId { get; set; }
    public string FunctionName { get; set; }
    public string FieldPath { get; set; }
    public string FieldDisplayName { get; set; }
    public string ValueDisplay { get; set; }
    public ConfigLayer CurrentLayer { get; set; }
    public bool CanMoveToUser { get; set; }       // true if currently Profile, User is empty
    public bool CanMoveToProfile { get; set; }    // true if currently User, Profile is empty
}
```

### Orchestrator API for Dialog

New method to gather all overrides:

```csharp
/// <summary>
/// Get all active function overrides across all functions and layers.
/// Used by the Override Review dialog.
/// </summary>
public List<OverrideReviewItem> GetAllActiveOverrides()
{
    var result = new List<OverrideReviewItem>();
    var layerProvider = CreateConfigLayerProvider();

    foreach (var functionId in _functionConfigManager.GetKnownFunctionIds())
    {
        var funcEnum = (FunctionID)functionId;
        var funcName = funcEnum.ToString().CamelCaseToTitleCase();

        foreach (var field in OverrideFieldRegistry.GetAllFields())
        {
            var sourceLayer = layerProvider.GetFieldSourceLayer(functionId, field.FieldPath);
            if (sourceLayer == null || sourceLayer == ConfigLayer.Baseline)
                continue; // No override

            var value = layerProvider.GetFieldValue(functionId, field.FieldPath, sourceLayer.Value);
            string valueDisplay;
            if (field.FieldType == OverrideFieldType.Complex)
                valueDisplay = "(configured)";
            else if (field.FormatValue != null)
                valueDisplay = field.FormatValue(value);
            else if (value is float f)
                valueDisplay = f.ToString("F2");
            else if (value is bool b)
                valueDisplay = b ? "Enabled" : "Disabled";
            else
                valueDisplay = value?.ToString() ?? "(null)";

            result.Add(new OverrideReviewItem
            {
                FunctionId = functionId,
                FunctionName = funcName,
                FieldPath = field.FieldPath,
                FieldDisplayName = field.DisplayName,
                ValueDisplay = valueDisplay,
                CurrentLayer = sourceLayer.Value,
                CanMoveToUser = sourceLayer == ConfigLayer.Profile
                    && !layerProvider.HasFieldValue(functionId, field.FieldPath, ConfigLayer.User),
                CanMoveToProfile = sourceLayer == ConfigLayer.User
                    && !layerProvider.HasFieldValue(functionId, field.FieldPath, ConfigLayer.Profile)
            });
        }
    }

    return result;
}
```

### Dialog Implementation

**New files:**
- `Controls/OverrideReviewDialog.xaml` — WPF dialog layout (dark theme, matches existing dialogs)
- `Controls/OverrideReviewDialog.xaml.cs` — code-behind with DataGrid operations

**Styling:** Match existing dark theme (`Background="#1B1B1B"`, `Foreground="White"`, `BorderBrush="#4A4A4A"`) from [ThemedMessageBox.xaml](SimHubPlugin/Controls/ThemedMessageBox.xaml) and [ParamReviewWindow.xaml](SimHubPlugin/GraphEditor/ParamReviewWindow.xaml).

**DataGrid columns:**

| Column | Width | Binding | Notes |
|--------|-------|---------|-------|
| Function | 100 | `FunctionName` | Text, grouped visually |
| Field | * | `FieldDisplayName` | Text |
| Value | 100 | `ValueDisplay` | Text, right-aligned |
| Layer | 70 | `CurrentLayer` | ComboBox dropdown (User/Profile) |
| Actions | 120 | — | "Bake" + "Clear" buttons |

**Layer ComboBox behavior:**

- Shows current layer as selected item
- Dropdown shows available layers: always shows current + available targets
- Changing selection calls `RerouteFunctionOverrideField` on the orchestrator
- Items disabled if target layer already has a value (prevents silent overwrite)

**Button handlers:**

```csharp
private void OnClearClick(object sender, RoutedEventArgs e)
{
    var item = (OverrideReviewItem)((Button)sender).DataContext;
    _orchestrator.ClearFunctionOverrideField(item.FunctionId, item.FieldPath, item.CurrentLayer);
    RefreshItems();
}

private void OnBakeClick(object sender, RoutedEventArgs e)
{
    var item = (OverrideReviewItem)((Button)sender).DataContext;
    _orchestrator.BakeFieldToBaseline(item.FunctionId, item.FieldPath);
    RefreshItems();
}

private void OnLayerChanged(object sender, SelectionChangedEventArgs e)
{
    var combo = (ComboBox)sender;
    var item = (OverrideReviewItem)combo.DataContext;
    var newLayer = (ConfigLayer)combo.SelectedItem;
    if (newLayer == item.CurrentLayer) return;

    _orchestrator.RerouteFunctionOverrideField(
        item.FunctionId, item.FieldPath,
        item.CurrentLayer, newLayer);
    RefreshItems();
}

private void OnClearAllUserClick(object sender, RoutedEventArgs e)
{
    var userItems = _items.Where(i => i.CurrentLayer == ConfigLayer.User).ToList();
    foreach (var item in userItems)
        _orchestrator.ClearFunctionOverrideField(item.FunctionId, item.FieldPath, ConfigLayer.User);
    RefreshItems();
}
```

### Access Point

Add a button to the Vehicle Profile tab header area:

```xml
<Button Content="Review Overrides"
        Click="OnReviewOverridesClick"
        Background="#3A3A3A"
        Foreground="White"
        BorderBrush="#5A5A5A"
        Padding="8,4"
        Margin="4"/>
```

The button could show a count badge when overrides exist.

## Implementation Phases

### Phase 1: Orchestrator Methods

**Files:** [TieredConfigOrchestrator.cs](SimHubPlugin/TieredConfig/TieredConfigOrchestrator.cs)

1. Add `RerouteFunctionOverrideField()` method
2. Add `BakeFieldToBaseline()` method
3. Add `GetAllActiveOverrides()` method
4. Add helper methods: `GetOverridesForLayer()`, `GetOrCreateOverridesForLayer()`,
   `CleanupEmptyOverrides()`, `WriteFieldToFunctionConfig()`

### Phase 2: Badge Context Menu Enhancement

**Files:** [LayerBadgeWrapper.xaml.cs](SimHubPlugin/Controls/LayerBadgeWrapper.xaml.cs)

1. Add "Move to User" / "Move to Profile" menu items (replace TODO at line 330)
2. Add "Save to Baseline" menu item
3. Add `OverrideRerouted` event + `OverrideReroutedEventArgs`
4. Add `OnRerouteOverride()` and `OnBakeToBaseline()` handlers

### Phase 3: Override Review Dialog

**Files:** `Controls/OverrideReviewDialog.xaml` (new), `Controls/OverrideReviewDialog.xaml.cs` (new)

1. Create dialog XAML (dark-themed DataGrid with layer dropdowns)
2. Create code-behind (populate from orchestrator, handle re-route/clear/bake)
3. Add `OverrideReviewItem` data model class

### Phase 4: UI Integration

**Files:** [DiyFfbPluginUI.xaml.cs](SimHubPlugin/DiyFfbPluginUI.xaml.cs) (or Vehicle Profile tab)

1. Add "Review Overrides" button to UI
2. Wire button to open `OverrideReviewDialog`
3. Update project file if needed

### Phase 5: Unit Tests

**Files:** `TieredConfigTests/`

1. Test `RerouteFunctionOverrideField` — value moves, source cleared, target populated
2. Test `BakeFieldToBaseline` — baseline updated, overrides cleared
3. Test `GetAllActiveOverrides` — returns correct items across functions/layers
4. Test edge cases: re-route when target already has value (should be no-op or error),
   bake with no baseline (should be no-op)

## WriteFieldToFunctionConfig — Field Mapping

The `BakeFieldToBaseline` method needs to copy a field value from the merged
`FunctionConfig` into the baseline `FunctionConfig`. This requires a field-path-to-property
mapping, complementing `ConfigLayerProvider.GetFieldValueFromConfig` (which reads values)
with a write counterpart.

```csharp
/// <summary>
/// Write a field value from the source config into the target config.
/// Used by BakeFieldToBaseline to copy effective values into baselines.
/// </summary>
private static void WriteFieldToFunctionConfig(
    FunctionConfig target, string fieldPath, FunctionConfig source)
{
    switch (fieldPath)
    {
        case "output_min":
            if (target.Base != null && source.Base != null)
                target.Base.OutputMin = source.Base.OutputMin;
            break;
        case "output_max":
            if (target.Base != null && source.Base != null)
                target.Base.OutputMax = source.Base.OutputMax;
            break;
        case "simulated_mass":
            target.SimulatedMass = source.SimulatedMass;
            break;
        case "friction":
            target.Friction = source.Friction;
            break;
        case "static_balance_tuning.enabled":
            if (source.StaticBalanceTuning != null)
            {
                if (target.StaticBalanceTuning == null)
                    target.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning();
                target.StaticBalanceTuning.Enabled = source.StaticBalanceTuning.Enabled;
            }
            break;
        case "static_balance_tuning.gain":
            if (source.StaticBalanceTuning != null)
            {
                if (target.StaticBalanceTuning == null)
                    target.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning();
                target.StaticBalanceTuning.Gain = source.StaticBalanceTuning.Gain;
            }
            break;
        case "force_curve":
            if (source.AutomotivePedal != null)
            {
                if (target.AutomotivePedal == null)
                    target.AutomotivePedal = new FunctionConfig.Types.AutomotivePedalConfig();
                target.AutomotivePedal.ForceCurveConfig = source.AutomotivePedal.ForceCurveConfig?.Clone();
            }
            break;
        case "damper_config.positive_factor":
        case "damper_config.negative_factor":
            if (source.AutomotivePedal?.DamperConfig != null)
            {
                if (target.AutomotivePedal == null)
                    target.AutomotivePedal = new FunctionConfig.Types.AutomotivePedalConfig();
                if (target.AutomotivePedal.DamperConfig == null)
                    target.AutomotivePedal.DamperConfig = new DamperConfig();
                if (fieldPath == "damper_config.positive_factor")
                    target.AutomotivePedal.DamperConfig.PositiveFactor = source.AutomotivePedal.DamperConfig.PositiveFactor;
                else
                    target.AutomotivePedal.DamperConfig.NegativeFactor = source.AutomotivePedal.DamperConfig.NegativeFactor;
            }
            break;
        case "shifter_config":
            if (source.Shifter != null)
                target.Shifter = source.Shifter.Clone();
            break;
        // FlightPedals/FlightStick fields follow same pattern...
    }
}
```

## Risk Assessment

**Low risk** — all changes are additive:

- Badge menu: adds items to existing context menu, no behavior changes to existing items
- Review dialog: new standalone window, no coupling to existing UI beyond orchestrator calls
- Orchestrator: new methods only, no modifications to existing methods
- Existing tests unaffected

**Edge cases to handle:**

1. Re-route when target already has a value → menu item hidden (Part 1) or dropdown entry disabled (Part 2)
2. Bake with no baseline stored → no-op, log warning
3. Bake Complex fields (force curve, shifter config) → uses `.Clone()` for protobuf types
4. Review dialog opened with no overrides → show "No overrides" message, disable batch buttons

## Files Summary

| File | Action | Phase |
|------|--------|-------|
| `TieredConfig/TieredConfigOrchestrator.cs` | Extend — 3 new public methods + helpers | 1 |
| `Controls/LayerBadgeWrapper.xaml.cs` | Extend — menu items, events, handlers | 2 |
| `Controls/OverrideReviewDialog.xaml` | **Create** — dialog layout | 3 |
| `Controls/OverrideReviewDialog.xaml.cs` | **Create** — dialog code-behind | 3 |
| `TieredConfig/TieredConfigTypes.cs` | Extend — `OverrideReviewItem` class | 3 |
| `DiyFfbPluginUI.xaml.cs` or similar | Extend — "Review Overrides" button | 4 |
| `DiyFfbPlugin.csproj` | Extend — new file references | 3-4 |
| `TieredConfigTests/*.cs` | Extend — new test cases | 5 |

## Future Extensions

- **Context change prompt**: Show quick summary dialog before vehicle/user switch if overrides exist
- **Conflict resolution**: If re-routing would overwrite an existing value, show confirmation
- **Export/Import overrides**: Serialize override sets using review dialog as selection UI
- **Undo support**: Track re-route/bake operations for undo
