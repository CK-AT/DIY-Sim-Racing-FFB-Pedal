# Motion Range Override Migration HOWTO

Reference implementation: **FlightStickConfigControl.xaml.cs** (fully migrated)

This guide covers migrating a function control's `Rangeslider_travel_range` to the override system. The FlightStick control is the proof-of-concept; FlightPedals still needs migration.

**Out of scope:** SplineForceCurve is a separate complex override (`force_curve`) that stores the entire `SplineForceCurveConfig`. It has its own registry entry and will be migrated independently.

## What's Already Done (all controls)

These defensive fixes are in place for FlightStick, FlightPedals, and SplineForceCurve:

1. **Degenerate bounds check** in `OnKinematicParametersChanged` — skips when `newMin >= newMax`
2. **`is_updating` protection** — wraps bounds changes to suppress WPF clamping events
3. **ContextIdle deferred clear** — waits for all WPF deferred events before clearing `is_updating`
4. **Slider value restoration** — re-applies config values after bounds change to counteract clamping

## What Needs Migration (per control)

### Step 1: Add `allowOverrideCreation` flag

Prevents override creation during init event storms (slider setup, deferred KP broadcasts).

```csharp
// Field declaration (next to is_updating)
private bool allowOverrideCreation = false;
```

### Step 2: Reset flag in SwitchFunction / UpdateConfig

At the START of the function switch, before any slider manipulation:

```csharp
allowOverrideCreation = false;  // Reset until function switch stabilizes
```

At the END, AFTER `is_updating = false`, defer enabling:

```csharp
// Allow override creation only after all deferred events have been processed
Dispatcher.BeginInvoke(new Action(() => allowOverrideCreation = true),
    System.Windows.Threading.DispatcherPriority.ContextIdle);
```

### Step 3: Add stale event filtering to range handlers

WPF RangeSlider fires deferred events with stale values. Add at the top of each value-changed handler:

```csharp
private void Rangeslider_travel_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
{
    if (!is_updating)
    {
        var newValue = Convert.ToInt16(e.NewValue);

        // Skip stale deferred events
        if (Rangeslider_travel_range != null &&
            Convert.ToInt16(Rangeslider_travel_range.LowerValue) != newValue)
            return;

        // ... existing config update ...
    }
    // ... label update, markers ...
}
```

Same pattern for `UpperValueChanged`.

### Step 4: Add override creation to range handlers

Inside the `if (!is_updating)` block, AFTER the existing config write, add the override call. The field path and override property depend on the control:

| Control | Field path | Override property | Override fields |
|---------|-----------|-------------------|-----------------|
| FlightStick | `"flight_stick.motion_range"` | `FlightStickMotionRange` | `.Min` / `.Max` |
| FlightPedals | `"flight_pedals.motion_range"` | `FlightPedalsMotionRange` | `.NearLim` / `.FarLim` |
| SplineForceCurve | *(no registry entry yet — see Step 6)* | — | — |

**FlightPedals example** (LowerValueChanged):

```csharp
var oldValue = config.PosNearLim;  // capture BEFORE write
config.PosNearLim = newValue;
function_config.Base.OutputMin = newValue;

if (allowOverrideCreation && newValue != oldValue &&
    plugin != null && function != null &&
    plugin.HasFunctionBaseline((int)function.ID))
{
    plugin.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.motion_range",
        overrides =>
        {
            if (overrides.FlightPedalsMotionRange == null)
                overrides.FlightPedalsMotionRange = new TieredConfig.MotionRangeOverrides();
            overrides.FlightPedalsMotionRange.NearLim = newValue;
        });
}
```

**FlightPedals example** (UpperValueChanged):

```csharp
var oldValue = config.PosFarLim;
config.PosFarLim = newValue;
function_config.Base.OutputMax = newValue;

if (allowOverrideCreation && newValue != oldValue &&
    plugin != null && function != null &&
    plugin.HasFunctionBaseline((int)function.ID))
{
    plugin.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.motion_range",
        overrides =>
        {
            if (overrides.FlightPedalsMotionRange == null)
                overrides.FlightPedalsMotionRange = new TieredConfig.MotionRangeOverrides();
            overrides.FlightPedalsMotionRange.FarLim = newValue;
        });
}
```

### Step 5: Add `OnBadgeOverrideCleared` handler

Subscribe in `OnLoaded`, unsubscribe in `OnUnloaded`:

```csharp
// In OnLoaded:
foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
{
    wrapper.OverrideCleared += OnBadgeOverrideCleared;
}

// In OnUnloaded:
foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
{
    wrapper.OverrideCleared -= OnBadgeOverrideCleared;
}
```

The handler reads the merged config (with override now cleared) and pushes values back to the UI:

**FlightPedals example:**

```csharp
private void OnBadgeOverrideCleared(object sender, LayerBadgeWrapper.OverrideClearedEventArgs e)
{
    if (plugin == null || function == null) return;

    var mergedConfig = plugin.FunctionConfigManager.GetCurrentConfig((int)function.ID);
    if (mergedConfig == null) return;

    is_updating = true;
    switch (e.FieldPath)
    {
        case "flight_pedals.motion_range":
            config.PosNearLim = mergedConfig.FlightPedals.PosNearLim;
            config.PosFarLim = mergedConfig.FlightPedals.PosFarLim;
            function_config.Base.OutputMin = config.PosNearLim;
            function_config.Base.OutputMax = config.PosFarLim;
            Rangeslider_travel_range.LowerValue = config.PosNearLim;
            Rangeslider_travel_range.UpperValue = config.PosFarLim;
            if (Label_near_pos != null)
                Label_near_pos.Content = String.Format("Near\n{0}mm", config.PosNearLim);
            if (Label_far_pos != null)
                Label_far_pos.Content = String.Format("Far\n{0}mm", config.PosFarLim);
            UpdateTravelMarkers();
            break;

        // Add other fields as they're migrated
    }
    is_updating = false;
}
```

## Field Mapping Quick Reference

| Control | Config field (Lower) | Config field (Upper) | Override class | Override Lower | Override Upper |
|---------|---------------------|---------------------|----------------|---------------|----------------|
| FlightStick | `GetPosMin()` / `SetPosMin()` | `GetPosMax()` / `SetPosMax()` | `MotionRangeOverrides` | `.Min` | `.Max` |
| FlightPedals | `config.PosNearLim` | `config.PosFarLim` | `MotionRangeOverrides` | `.NearLim` | `.FarLim` |

## Step 6: XAML — Add LayerBadgeWrapper

The badge template uses `RenderTransform Y=-14` to float above the wrapped control. It needs vertical space above it (typically a section header label) or it gets clipped by SimHub's outer ScrollViewer.

**Pattern A — Regular Slider** (used in "Additional Settings" for damping, friction, etc.):

```xml
<StackPanel Width="400" Height="40">
    <Label Content="Field Name:" Foreground="White" FontSize="10" FontFamily="Arial"
           HorizontalAlignment="Left" VerticalAlignment="Top" Padding="0,0,0,8"/>
    <badge:LayerBadgeWrapper FieldPath="field.path">
        <Slider ... Width="400" Height="10"/>
    </badge:LayerBadgeWrapper>
</StackPanel>
```

The label above the slider provides the badge's landing zone within the fixed-height container.

**Pattern B — RangeSlider** (used for motion_range and rudder brake force_range):

```xml
<StackPanel Width="400" Orientation="Horizontal">
    <Label x:Name="Label_min" Width="42" Height="30" Content="Min" .../>
    <badge:LayerBadgeWrapper FieldPath="field.path">
        <metro:RangeSlider ... Width="316" Height="20"/>
    </badge:LayerBadgeWrapper>
    <Label x:Name="Label_max" Width="42" Height="30" Content="Max" .../>
</StackPanel>
```

Min/Max labels go **outside** the badge wrapper. The section header above provides the badge's landing zone. Widths must sum to the container width (42 + 316 + 42 = 400).

## Checklist

Per control, check off:

- [ ] `allowOverrideCreation` field declared
- [ ] Flag reset in SwitchFunction/UpdateConfig
- [ ] Flag deferred-enable via ContextIdle at end of SwitchFunction
- [ ] Stale event filtering in `LowerValueChanged`
- [ ] Stale event filtering in `UpperValueChanged`
- [ ] Override creation in `LowerValueChanged`
- [ ] Override creation in `UpperValueChanged`
- [ ] `OnBadgeOverrideCleared` handler implemented
- [ ] `OnBadgeOverrideCleared` subscribed in OnLoaded
- [ ] `OnBadgeOverrideCleared` unsubscribed in OnUnloaded
- [ ] Build + test: edit range → [U] badge appears
- [ ] Build + test: right-click badge → clear → values revert to baseline
