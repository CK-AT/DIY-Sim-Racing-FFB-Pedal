# Themed MessageBox Plan

Date: 2026-01-31
Status: Implemented

## Goal

Replace Windows native `MessageBox.Show()` calls with a dark-themed WPF dialog that matches the plugin's visual style.

## Problem Statement

- Native Windows MessageBox uses system theme (light mode)
- Appears jarring against the plugin's dark UI (#1B1B1B background)
- No way to customize appearance of system dialogs

## Scope

- Create `ThemedMessageBox` static class with `Show()` method
- Support common button combinations (OK, OKCancel, YesNo, YesNoCancel)
- Support message icons (Info, Warning, Error, Question)
- Replace existing `MessageBox.Show()` calls throughout the plugin
- **Out of scope**: File dialogs (OpenFileDialog/SaveFileDialog) - these remain system-themed

## Design

### ThemedMessageBox API

```csharp
public static class ThemedMessageBox
{
    public static MessageBoxResult Show(
        string message,
        string title,
        MessageBoxButton buttons = MessageBoxButton.OK,
        MessageBoxImage icon = MessageBoxImage.None);

    public static MessageBoxResult Show(
        Window owner,
        string message,
        string title,
        MessageBoxButton buttons = MessageBoxButton.OK,
        MessageBoxImage icon = MessageBoxImage.None);
}
```

### Dialog Layout

```
┌─────────────────────────────────────────────┐
│  [Icon]  Title                              │
├─────────────────────────────────────────────┤
│                                             │
│  [Icon]   Message text here that can       │
│           wrap to multiple lines            │
│                                             │
├─────────────────────────────────────────────┤
│                    [Button1] [Button2] ...  │
└─────────────────────────────────────────────┘
```

### Styling (match existing dialogs)

- Background: `#1B1B1B`
- Foreground: `White`
- Border: `#4A4A4A`
- Button backgrounds:
  - Default: `#3A3A3A`
  - Primary (Yes/OK): `#4A6A4A` (green tint)
  - Destructive (No when warning): `#6A4A4A` (red tint)
- Icons:
  - Info: Blue circle with "i"
  - Warning: Yellow triangle with "!"
  - Error: Red circle with "X"
  - Question: Blue circle with "?"

## File Structure

```
SimHubPlugin/
  Controls/
    ThemedMessageBox.xaml        # Dialog UI
    ThemedMessageBox.xaml.cs     # Code-behind with static Show() methods
```

## Implementation Steps

1. Create `ThemedMessageBox.xaml` with dark-themed layout
2. Create `ThemedMessageBox.xaml.cs` with static `Show()` methods
3. Add icon resources (simple shapes or Unicode symbols)
4. Add to `DiyFfbPlugin.csproj`
5. Replace `MessageBox.Show()` calls:
   - DiyFfbPluginUI.xaml.cs (~15 calls)
   - GraphEditorWindow.xaml.cs (~8 calls)
   - ProfileBrowserDialog.xaml.cs (~5 calls)
   - GraphTemplateSelectorDialog.xaml.cs (1 call)
   - ParamReviewWindow.xaml.cs (2 calls)
   - AutomotivePedalConfigControl.xaml.cs (2 calls)
   - GraphEditorControl.xaml.cs (1 call)

## Verification

1. Build succeeds
2. All message dialogs appear with dark theme
3. Button combinations work correctly (OK, OKCancel, YesNo, YesNoCancel)
4. Icons display appropriately for each message type
5. Dialog centers on owner window
6. Escape key and X button work as expected

## Migration Pattern

```csharp
// Before
MessageBox.Show("Message", "Title", MessageBoxButton.YesNo, MessageBoxImage.Question);

// After
ThemedMessageBox.Show("Message", "Title", MessageBoxButton.YesNo, MessageBoxImage.Question);
```

Same API, just change the class name.
