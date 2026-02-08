# Clear Baseline

**Status: DONE** (commit `15aefff8`)

## Problem

Once a baseline is saved (via "Save as Baseline"), there is no way to remove it
short of editing the settings JSON. Users need to be able to clear a baseline so
that the next ESP32 upload, config import, or manual edit establishes a fresh
starting point.

Overrides should be **preserved** across a baseline clear — they are stored
independently in Settings and will re-apply automatically when a new baseline
arrives.

## Implementation

### `DiyFfbPlugin.cs` — clear methods

Used existing `FunctionConfigManager.ResetFunction()` (equivalent to the planned
`RemoveBaseConfig`) and `AxisConfigManager.ResetAxis()`.

**`ClearFunctionBaseline(int functionId)`:**

- Remove from `Settings.FunctionBaselines`
- `_functionConfigManager.ResetFunction(functionId)`
- `SaveCommonSettings("GeneralSettings", Settings)`

**`ClearAxisBaseline(int axisId)`:**

- Remove from `Settings.AxisBaselines`
- `_axisConfigManager.ResetAxis(axisId)`
- `SaveCommonSettings("GeneralSettings", Settings)`

### `DiyFfbPluginUI.xaml` — buttons

**Function baseline:** `btn_clear_function_baseline` in the same StackPanel
after `btn_save_function_baseline`. Same style/size (160x28).

**Axis baseline:** `btn_clear_axis_baseline` after `btn_save_axis_baseline`.
Same style/size (130x22, FontSize 10).

### `DiyFfbPluginUI.xaml.cs` — click handlers

**`OnClearFunctionBaselineClicked`:**

- Guard: return if no function selected or no baseline exists
- `ThemedMessageBox.Show(...)` with YesNo + Question icon
- Call `Plugin.ClearFunctionBaseline(funcId)`
- Refresh UI badges

**`OnClearAxisBaselineClicked`:**

- Guard: return if no axis selected or no baseline exists
- `ThemedMessageBox.Show(...)` with YesNo + Question icon
- Call `Plugin.ClearAxisBaseline(axisId)`
- Refresh UI

## Files touched

| File | Change |
| ---- | ------ |
| `DiyFfbPlugin.cs` | Add `ClearFunctionBaseline`, `ClearAxisBaseline` |
| `DiyFfbPluginUI.xaml` | Add two buttons |
| `DiyFfbPluginUI.xaml.cs` | Add two click handlers |

## Design notes

- Overrides (profile + user) are **not** cleared when the baseline is removed.
  They persist in `Settings.FunctionOverrides` / `UserPreferencesProfiles` and
  re-apply automatically via the normal `OnFunctionConfigUpdate` ->
  `ApplyProfileOverridesToFunction` flow when a new baseline arrives.
- `FunctionConfigManager.ResetFunction()` already existed and does exactly what
  the originally planned `RemoveBaseConfig` specified (clears all 5 dictionaries).
- `AxisConfigManager.ResetAxis()` already exists and does exactly what's needed
  for the axis side.
