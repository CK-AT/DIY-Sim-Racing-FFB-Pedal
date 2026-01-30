# Shared Graph Save Protection

Date: 2026-01-30
Status: Implemented

## Problem

When a user edits a graph file in the Graph Editor, they may not realize that the file is shared by multiple vehicles. Saving changes could inadvertently affect all those vehicles without warning.

**Scenarios where graphs are shared:**

1. **Direct sharing**: Multiple vehicles use the same top-level graph file (e.g., `xplane_GA.json` used by Cessna 172, Baron 58, etc.)
2. **Include sharing**: A graph file is included by multiple other graphs (e.g., `common_damping.json` included by several vehicle-specific graphs)

## Current Behavior

- User opens graph in editor
- User makes changes
- User saves → file is overwritten
- All vehicles using that graph (directly or via include) are affected
- **No warning is shown**

## Proposed Solution

### On Save: Detect Shared Usage

Before saving, scan all stored profiles and graph files to determine:

1. Which vehicles reference this graph as their `GraphPath`
2. Which other graph files include this graph (recursively)
3. Which vehicles use those including graphs

### Show Impact Dialog

If the graph is used by more than one vehicle (or by any vehicle other than the current one), show a dialog:

```
┌─────────────────────────────────────────────────────────────┐
│  Save Shared Graph                                    [X]   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ⚠ This graph is used by multiple vehicles:                │
│                                                             │
│  Direct usage:                                              │
│    • MSFS::Cessna_172 (current)                            │
│    • MSFS::Baron_58                                        │
│    • MSFS::Bonanza_G36                                     │
│                                                             │
│  Included by:                                               │
│    • graphs/xplane_heli.json                               │
│      → XPlane::R22_Beta_II                                 │
│      → XPlane::Bell_407                                    │
│                                                             │
│  Saving will affect ALL listed vehicles.                    │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│  [Save Anyway]  [Save as Copy...]  [Cancel]                │
└─────────────────────────────────────────────────────────────┘
```

### Actions

| Button | Behavior |
| ------ | -------- |
| **Save Anyway** | Overwrite the file; all vehicles affected |
| **Save as Copy...** | Prompt for new filename, save there, update current vehicle's GraphPath to point to the copy |
| **Cancel** | Return to editor, keep changes unsaved |

## Implementation

### Phase 1: Usage Scanner

Create a utility class to scan graph usage:

```csharp
public class GraphUsageScanner
{
    // Find all vehicles that directly use this graph path
    public List<string> GetDirectUsers(string graphPath);

    // Find all graph files that include this graph (recursively)
    public List<string> GetIncludingGraphs(string graphPath);

    // Find all vehicles using graphs that include this graph
    public List<(string graphPath, List<string> vehicles)> GetIndirectUsers(string graphPath);

    // Combined: is this graph shared?
    public GraphUsageReport GetUsageReport(string graphPath, string currentVehicleKey);
}

public class GraphUsageReport
{
    public string GraphPath;
    public string CurrentVehicleKey;
    public List<string> DirectUsers;           // Vehicle keys
    public List<IncludeUsage> IncludedBy;      // Graph paths + their users
    public bool IsShared => DirectUsers.Count > 1 || IncludedBy.Count > 0;
}

public class IncludeUsage
{
    public string IncludingGraphPath;
    public List<string> VehicleKeys;
}
```

### Phase 2: Save Interception

Modify `GraphEditorWindow` or `GraphEditorTabManager` save logic:

```csharp
private void SaveGraph(string path)
{
    var scanner = new GraphUsageScanner(plugin);
    var report = scanner.GetUsageReport(path, plugin.GetActiveProfileKey());

    if (report.IsShared)
    {
        var dialog = new SharedGraphSaveDialog(report);
        var result = dialog.ShowDialog();

        switch (result)
        {
            case SharedGraphSaveResult.SaveAnyway:
                DoSave(path);
                break;

            case SharedGraphSaveResult.SaveAsCopy:
                var newPath = PromptForNewPath(path);
                if (newPath != null)
                {
                    DoSave(newPath);
                    plugin.SetVehicleGraphPath(newPath);  // Update current vehicle
                }
                break;

            case SharedGraphSaveResult.Cancel:
                return;
        }
    }
    else
    {
        DoSave(path);
    }
}
```

### Phase 3: Save as Copy Logic

When saving as a copy:

1. Prompt user for new filename (default: `{original}_copy.json` or `{original}_{vehicleId}.json`)
2. Save graph to new file
3. Update current vehicle's profile to use the new path
4. Keep editor open with new file
5. Original file remains unchanged

### Data Sources for Scanning

**Direct users:**
- `Settings.AircraftFfbProfiles` → each profile's `GraphPath`

**Include scanning:**
- Load each graph file in `graphs/` directory
- Parse `GraphDefinition.Includes` array
- Build reverse lookup: `includedPath → [parentPaths]`
- Recursively expand to find all ancestors

### Performance Considerations

- Cache the include graph on first scan
- Invalidate cache when:
  - A graph file is saved
  - A new graph file is created
  - A vehicle's GraphPath changes
- For large graph collections, consider lazy loading

### Edge Cases

1. **Circular includes**: Detect and handle gracefully (already a problem elsewhere)
2. **Missing files**: Skip files that can't be loaded
3. **External graphs**: Graphs outside `graphs/` folder may not be scanned
4. **Template graphs**: Templates in registry should be flagged as "widely shared"

## UI/UX Notes

- Dialog should clearly show impact scope
- "Save Anyway" should not be the default button (prevent accidental clicks)
- Consider adding a "Don't show again for this graph" checkbox for advanced users
- Show vehicle names (not just IDs) where available

## Future Enhancements

1. **Diff preview**: Show what changed before saving
2. **Selective propagation**: Choose which vehicles should get the update
3. **Graph versioning**: Track versions and allow rollback
4. **Notifications**: After "Save Anyway", notify user which other profiles may need review

## Files to Modify

| File | Changes |
| ---- | ------- |
| `GraphEditor/GraphEditorWindow.xaml.cs` | Add save interception |
| `GraphEditor/GraphEditorTabManager.cs` | Add save interception for tab saves |
| NEW: `GraphUsageScanner.cs` | Usage scanning logic |
| NEW: `SharedGraphSaveDialog.xaml` | Impact dialog UI |
| NEW: `SharedGraphSaveDialog.xaml.cs` | Dialog code-behind |
| `DiyFfbPlugin.csproj` | Add new files |

## Related Documents

- [16_Vehicle_Profile_Lifecycle.md](16_Vehicle_Profile_Lifecycle.md) — Profile structure and storage
- [06_FFB_Graph_Editor_Tabs.md](06_FFB_Graph_Editor_Tabs.md) — Tab management and save logic
- [15_Graph_Param_Override_Migration_Plan.md](15_Graph_Param_Override_Migration_Plan.md) — Related: detecting graph changes

## Priority

**High** — This is a data safety issue. Users could unknowingly break configurations for multiple vehicles.

## Effort Estimate

- Phase 1 (Scanner): Low-Medium
- Phase 2 (Save interception): Low
- Phase 3 (Save as Copy): Medium
- Dialog UI: Low

**Total: Medium effort**
