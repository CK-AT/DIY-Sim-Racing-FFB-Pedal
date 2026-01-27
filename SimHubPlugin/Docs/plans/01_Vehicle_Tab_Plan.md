# Vehicle Tab Implementation Plan

## Overview

Add a new top-level "VEHICLE" tab alongside Functions/Axes/System that provides a central place for tuning non-function, non-system graph parameters. Parameters are organized into collapsible panels by their `Group` property.

## Problem Statement

Include sub-graphs can define Param nodes with arbitrary `Group` values, but currently:
- Function groups (`FlightStickPitch`, `FlightStickRoll`, etc.) show in function-specific tabs
- `System` group shows in System tab
- **All other groups have no UI** - making them impossible to tune at runtime

## Design Decisions

| Decision | Choice |
|----------|--------|
| Unknown/empty Group handling | Create `<unknown>` panel, only if such params exist |
| Group naming conventions | No enforcement for now |
| Panel ordering | Alphabetical by Group name |
| UI pattern | Reuse FFB Parameters panel style from function tabs |
| Expander state | Expanded by default |
| Sync | Bi-directional via `GraphParamChanged` event |

## Filtering Logic

A parameter appears in the Vehicle tab if:
```csharp
bool IsVehicleParam(GraphParam p)
{
    var group = p.Ui?.Group;

    // Null/empty -> "<unknown>" panel
    if (string.IsNullOrEmpty(group))
        return true;

    // Exclude System
    if ("System".Equals(group, StringComparison.OrdinalIgnoreCase))
        return false;

    // Exclude function groups
    if (group.StartsWith("FlightStick", StringComparison.OrdinalIgnoreCase))
        return false;
    if (group.StartsWith("FlightPedals", StringComparison.OrdinalIgnoreCase))
        return false;
    if (group.StartsWith("Automotive", StringComparison.OrdinalIgnoreCase))
        return false;
    if (group.StartsWith("Shifter", StringComparison.OrdinalIgnoreCase))
        return false;

    return true;
}
```

## UI Structure

```
VEHICLE Tab
├── ScrollViewer
│   └── StackPanel (vehicleParamsContainer)
│       ├── Expander (Group = "Aero")
│       │   └── StackPanel
│       │       ├── ParamPanel (Label + Control)
│       │       └── ParamPanel (Label + Control)
│       ├── Expander (Group = "Engine")
│       │   └── StackPanel
│       │       └── ParamPanel (Label + Control)
│       └── Expander (Group = "<unknown>")  ← only if needed
│           └── StackPanel
│               └── ParamPanel (Label + Control)
```

Each Expander styled similarly to FFB Parameters Border:
- Background: `#7F4E4E4E`
- CornerRadius: 5
- Header: Group name (Arial Black)
- Content: StackPanel with parameter controls

## Implementation Steps

### Phase 1: Add Vehicle Tab to XAML

**File**: `DiyFfbPluginUI.xaml`

1. Add new `TabItem` after `Tab_System` (or between existing tabs):
   ```xml
   <TabItem x:Name="Tab_Vehicle">
       <TabItem.Header>
           <!-- Icon + "VEHICLE" label, same style as other tabs -->
       </TabItem.Header>
       <ScrollViewer VerticalScrollBarVisibility="Auto">
           <StackPanel x:Name="VehicleParamsContainer"
                       Orientation="Vertical"
                       Margin="10"/>
       </ScrollViewer>
   </TabItem>
   ```

### Phase 2: Add Vehicle Parameter Logic

**File**: `DiyFfbPluginUI.xaml.cs`

1. Add tracking dictionaries:
   ```csharp
   private Dictionary<string, FrameworkElement> vehicleParamControls = new();
   private Dictionary<string, Expander> vehicleGroupExpanders = new();
   ```

2. Add `RefreshVehicleParams()` method:
   ```csharp
   private void RefreshVehicleParams()
   {
       VehicleParamsContainer.Children.Clear();
       vehicleParamControls.Clear();
       vehicleGroupExpanders.Clear();

       var allParams = Plugin.GetActiveGraphParams();

       // Filter to vehicle params only
       var vehicleParams = allParams.Values
           .Where(IsVehicleParam)
           .ToList();

       if (vehicleParams.Count == 0)
       {
           // Show "No vehicle parameters" message
           return;
       }

       // Group by Group property
       var grouped = vehicleParams
           .GroupBy(p => string.IsNullOrEmpty(p.Ui?.Group) ? "<unknown>" : p.Ui.Group)
           .OrderBy(g => g.Key == "<unknown>" ? "zzz" : g.Key); // <unknown> last

       foreach (var group in grouped)
       {
           var expander = CreateGroupExpander(group.Key, group.ToList());
           VehicleParamsContainer.Children.Add(expander);
           vehicleGroupExpanders[group.Key] = expander;
       }
   }
   ```

3. Add `CreateGroupExpander()` helper:
   ```csharp
   private Expander CreateGroupExpander(string groupName, List<GraphParam> params)
   {
       var expander = new Expander
       {
           Header = groupName,
           IsExpanded = true,
           // Style similar to FFB Parameters
       };

       var panel = new StackPanel { Orientation = Orientation.Vertical };

       foreach (var param in params.OrderBy(p => p.Ui?.Label ?? p.Name))
       {
           var paramPanel = CreateParamPanel(param);
           panel.Children.Add(paramPanel);
       }

       expander.Content = panel;
       return expander;
   }
   ```

4. Add `CreateParamPanel()` helper (reuse pattern from `RefreshSystemGraphParams`):
   ```csharp
   private StackPanel CreateParamPanel(GraphParam param)
   {
       var panel = new StackPanel
       {
           Width = 400,
           Height = 40,
           Orientation = Orientation.Vertical
       };

       var label = new Label
       {
           Foreground = Brushes.White,
           FontSize = 10,
           FontFamily = new FontFamily("Arial"),
           Content = $"{param.Ui?.Label ?? param.Name}",
           Padding = new Thickness(0, 0, 0, 8)
       };

       double currentValue = Plugin.GetGraphParamValue(param.Name);
       var control = GraphParamControlBuilder.BuildControl(
           param,
           value => Plugin.SetGraphParamValue(param.Name, value),
           width: 400,
           initialValue: currentValue
       );

       panel.Children.Add(label);
       panel.Children.Add(control);

       vehicleParamControls[param.Name] = control;
       return panel;
   }
   ```

### Phase 3: Wire Up Refresh Calls

**File**: `DiyFfbPluginUI.xaml.cs`

1. Call `RefreshVehicleParams()` when graph changes:
   - In `OnActiveGraphChanged()` or equivalent
   - After `RefreshSystemGraphParams()` call

2. Add update method for live value changes (if needed):
   ```csharp
   private void UpdateVehicleParamValue(string paramName, double value)
   {
       if (vehicleParamControls.TryGetValue(paramName, out var control))
       {
           // Update control value without triggering callback
       }
   }
   ```

### Phase 4: Handle Empty State

When no vehicle parameters exist, show a friendly message:
```csharp
if (vehicleParams.Count == 0)
{
    var message = new TextBlock
    {
        Text = "No vehicle parameters defined.\n\nVehicle parameters can be added via graph Param nodes with custom Group values.",
        Foreground = Brushes.Gray,
        FontStyle = FontStyles.Italic,
        Margin = new Thickness(10)
    };
    VehicleParamsContainer.Children.Add(message);
    return;
}
```

## Bi-Directional Sync

The plugin fires `GraphParamChanged` event when any parameter value changes (from graph editor, other controls, etc.). The Vehicle tab must:

1. **Subscribe** to `Plugin.GraphParamChanged` event
2. **Handle** incoming changes by updating the corresponding control
3. **Guard** against infinite loops with `isUpdatingVehicleParams` flag

**Event Handler Pattern** (from FlightStickConfigControl):
```csharp
private bool isUpdatingVehicleParams = false;

private void OnGraphParamChanged(object sender, GraphParamChangedEventArgs e)
{
    if (isUpdatingVehicleParams)
        return;

    Dispatcher.Invoke(() =>
    {
        isUpdatingVehicleParams = true;
        try
        {
            if (vehicleParamControls.TryGetValue(e.ParamName, out var control))
            {
                if (control is Slider slider)
                {
                    slider.Value = e.Value;
                }
                else if (control is TextBox textBox)
                {
                    var allParams = Plugin?.GetActiveGraphParams();
                    int precision = 3;
                    if (allParams != null && allParams.TryGetValue(e.ParamName, out var param))
                    {
                        precision = param.Ui?.Precision ?? 3;
                    }
                    textBox.Text = e.Value.ToString($"F{precision}");
                }
                else if (control is CheckBox checkBox)
                {
                    checkBox.IsChecked = e.Value > 0.5;
                }
                else if (control is ComboBox comboBox)
                {
                    // Find matching option by value
                    // ...
                }
            }

            // Optionally update label if it shows current value
            if (vehicleParamLabels.TryGetValue(e.ParamName, out var label))
            {
                // Update label content
            }
        }
        finally
        {
            isUpdatingVehicleParams = false;
        }
    });
}
```

**Subscription** (in constructor or initialization):

```csharp
Plugin.ActiveGraphChanged += OnActiveGraphChanged_Vehicle;
Plugin.GraphParamChanged += OnGraphParamChanged_Vehicle;
```

**Unsubscription** (important for cleanup):

```csharp
// When control is unloaded or disposed
Plugin.ActiveGraphChanged -= OnActiveGraphChanged_Vehicle;
Plugin.GraphParamChanged -= OnGraphParamChanged_Vehicle;
```

## Files to Modify

| File                     | Changes                                                                                                                                                      |
|--------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `DiyFfbPluginUI.xaml`    | Add Tab_Vehicle TabItem with ScrollViewer and StackPanel                                                                                                     |
| `DiyFfbPluginUI.xaml.cs` | Add RefreshVehicleParams(), IsVehicleParam(), CreateGroupExpander(), CreateParamPanel(), tracking dictionaries, GraphParamChanged handler, sync guard flag   |

## Testing

1. **No vehicle params**: Verify empty state message appears
2. **Single group**: Verify expander appears with correct params
3. **Multiple groups**: Verify alphabetical ordering, `<unknown>` last
4. **Mixed params**: Verify function/system params don't appear in Vehicle tab
5. **Value changes**: Verify slider/control changes persist
6. **Graph switch**: Verify params refresh when switching graphs

## Future Enhancements (Out of Scope)

- Drag-and-drop panel reordering
- Collapsible state persistence
- Search/filter within Vehicle tab
- Custom group icons
- Group description/tooltip from graph metadata
