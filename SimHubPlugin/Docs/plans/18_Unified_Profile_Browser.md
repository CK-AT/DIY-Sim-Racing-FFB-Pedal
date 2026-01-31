# Unified Profile Browser Dialog

Date: 2026-01-30
Status: Implemented
Issue: #9 from 17_Profile_System_Improvements.md

## Overview

A unified dialog that consolidates template selection, profile management, and profile import into a single reusable component.

## Problem Statement

Current UX issues:
1. Template selector only offers graph files, not existing tuned profiles
2. "Save/Load Aircraft FFB" buttons are ambiguous
3. No visibility into stored profiles (can't browse, delete, or copy from other vehicles)
4. Profile management buttons are buried below System Parameters

## Proposed Solution

### Dialog Modes

| Mode | Trigger | Primary Action |
|------|---------|----------------|
| NewVehicle | New vehicle detected | Select starting template or copy from existing |
| ManageProfiles | "Manage Profiles" button | Browse, delete, export profiles |
| CopyFromVehicle | "Copy from Vehicle" button | Copy tuning from another vehicle |

### Dialog Layout

```
┌─────────────────────────────────────────────────────────┐
│  Profile Browser                                        │
├─────────────────────────────────────────────────────────┤
│  (○) Templates  (○) My Vehicles  [Import File...]       │
├─────────────────────────────────────────────────────────┤
│  ┌───────────────────────────────────────────────────┐  │
│  │ Name              │ Graph           │ Tuned │ Del │  │
│  ├───────────────────┼─────────────────┼───────┼─────┤  │
│  │ Cessna 172        │ xplane_GA.json  │  ✓    │ [X] │  │
│  │ Baron 58          │ xplane_GA.json  │  ✓    │ [X] │  │
│  │ R22 Beta II       │ xplane_heli.json│  ✓    │ [X] │  │
│  └───────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────┤
│  Selected: Cessna 172                                   │
│  Graph: graphs/xplane_GA.json                           │
│  Tuning: 3 parameters customized                        │
├─────────────────────────────────────────────────────────┤
│  [Cancel]  [Use Graph Only]  [Use Graph + Tuning]       │
└─────────────────────────────────────────────────────────┘
```

### Tab Contents

- **Templates**: Graph templates from `GraphTemplateRegistry` (no tuning)
- **My Vehicles**: Stored profiles from `AircraftFfbProfiles` (with tuning)
- **Import File...**: Button opens file dialog to load exported profile

### Action Buttons (context-dependent)

| Mode | Use Graph Only | Use Graph + Tuning | Export |
|------|----------------|-------------------|--------|
| NewVehicle | Apply template defaults | Apply template + copy params | Hidden |
| ManageProfiles | Change to selected graph | Change graph + copy params | Visible |
| CopyFromVehicle | Change to selected graph | Change graph + copy params | Hidden |

## File Structure

```
SimHubPlugin/
  ProfileBrowser/
    ProfileBrowserDialog.xaml       # Dialog UI (WPF)
    ProfileBrowserDialog.xaml.cs    # Code-behind
    ProfileBrowserEntry.cs          # List item view model
    ProfileBrowserMode.cs           # Usage context enum
```

## Data Classes

### ProfileBrowserMode.cs

```csharp
public enum ProfileBrowserMode
{
    NewVehicle,      // Template picker for new vehicle
    ManageProfiles,  // Browse/delete/export
    CopyFromVehicle  // Copy tuning from another vehicle
}
```

### ProfileBrowserEntry.cs

```csharp
public enum ProfileEntrySource { Template, StoredProfile, ImportedFile }

public class ProfileBrowserEntry
{
    // Identity
    public string ProfileKey { get; set; }      // "gameId::carId" for stored
    public ProfileEntrySource Source { get; set; }

    // Display
    public string Name { get; set; }
    public string GraphPath { get; set; }
    public string GraphName { get; set; }       // Filename only
    public string Description { get; set; }     // For templates

    // Tuning info (stored profiles only)
    public bool HasTuning { get; set; }
    public int TunedParamCount { get; set; }

    // Data references
    public AircraftFfbProfile Profile { get; set; }
    public GraphTemplateEntry TemplateEntry { get; set; }

    // Factory methods
    public static ProfileBrowserEntry FromTemplate(GraphTemplateEntry t);
    public static ProfileBrowserEntry FromProfile(string key, AircraftFfbProfile p, string graphPath);
    public static ProfileBrowserEntry FromImportedFile(ExportedProfile e);
}
```

## Dialog Implementation

### Constructor

```csharp
public ProfileBrowserDialog(
    DiyFfbPlugin plugin,
    ProfileBrowserMode mode,
    string gameId = null,
    string carId = null)
```

### Public Result Properties

```csharp
public ProfileBrowserEntry SelectedEntry { get; private set; }
public bool UseGraphOnly { get; private set; }
public bool UseTuning { get; private set; }
```

### Key Methods

| Method | Purpose |
|--------|---------|
| `ConfigureForMode()` | Set title, default tab, button visibility |
| `LoadTemplates()` | Populate from `GraphTemplateRegistry.GetTemplates()` |
| `LoadStoredProfiles()` | Populate from `Settings.AircraftFfbProfiles` |
| `OnImportClick()` | File dialog → deserialize `ExportedProfile` |
| `OnDeleteClick()` | Remove profile with confirmation |
| `OnExportClick()` | Save profile to file |

## Integration Points

### DiyFfbPlugin.cs

Add helper methods:

```csharp
public string GetGraphPathForProfileKey(string profileKey)
// Parse "gameId::carId" and look up in AircraftFfbProfiles[key].GraphPath

public void ApplyProfileFromBrowser(string graphPath, AircraftFfbProfile profile, bool useTuning)
// 1. Set graph path for current vehicle (creates profile if needed)
// 2. If useTuning, copy GraphParamValues
// 3. Reload graph and notify UI
```

**Note:** Graph paths are now stored in `AircraftFfbProfile.GraphPath` instead of a separate `VehicleGraphPaths` dictionary. This consolidation ensures "My Vehicles" shows all vehicles with assigned graphs.

### DiyFfbPluginUI.xaml.cs

Add:

```csharp
private void ShowProfileBrowser(ProfileBrowserMode mode)
{
    var dialog = new ProfileBrowserDialog(Plugin, mode, ...);
    dialog.Owner = Window.GetWindow(this);
    if (dialog.ShowDialog() == true && dialog.SelectedEntry != null)
    {
        Plugin.ApplyProfileFromBrowser(dialog.SelectedEntry, dialog.UseTuning);
        RefreshGraphSelection();
    }
}

private void btn_manage_profiles_Click(object sender, RoutedEventArgs e)
    => ShowProfileBrowser(ProfileBrowserMode.ManageProfiles);
```

### DiyFfbPluginUI.xaml

Add "Manage Profiles" button near existing Save/Load/Reset buttons.

## Implementation Order

1. **ProfileBrowserMode.cs** - Simple enum
2. **ProfileBrowserEntry.cs** - View model with factory methods
3. **ProfileBrowserDialog.xaml** - UI layout following `GraphTemplateSelectorDialog` patterns
4. **ProfileBrowserDialog.xaml.cs** - Tab switching, list population, actions
5. **DiyFfbPlugin.cs** - Add `ApplyProfileFromBrowser()` helper
6. **DiyFfbPluginUI.xaml/.cs** - Add button and handler
7. **DiyFfbPlugin.csproj** - Add new files

## Styling Reference

Follow patterns from `GraphTemplateSelectorDialog.xaml`:
- `Background="#1B1B1B"`, `Foreground="White"`
- `WindowStartupLocation="CenterOwner"`, `ResizeMode="NoResize"`
- ListBox with custom `ItemContainerStyle` for hover/selected states
- Border colors: `#4A4A4A` (border), `#2A2A2A` (hover), `#3A3A3A` (selected)

## Verification

1. Build succeeds
2. "Manage Profiles" button opens dialog
3. Templates tab shows templates from registry
4. My Vehicles tab shows stored profiles with tuning indicator
5. Import File loads exported profile
6. "Use Graph Only" applies graph without tuning
7. "Use Graph + Tuning" copies parameters
8. Delete removes profile with confirmation
9. Export saves profile to file

## Future Enhancements

- Replace `PromptForGraphTemplate()` in DiyFfbPlugin.cs to use ProfileBrowserDialog in NewVehicle mode
- Add search/filter for large profile lists
- Add LastUsed timestamp to AircraftFfbProfile for sorting
