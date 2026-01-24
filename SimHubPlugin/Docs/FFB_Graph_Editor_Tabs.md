# Graph Editor Tab System Design

Status: Design document for multi-tab support in the graph editor.

## Overview

Add a tab control to the graph editor allowing multiple graphs to be open simultaneously:
- **Active graph tab**: Always present, auto-loaded based on vehicle selection, non-closeable
- **Manual tabs**: User-opened graphs, closeable, opened via File > Open or Include navigation

## Current Architecture

```
GraphEditorWindow
├── Toolbar (Load/Save/Reset)
├── Left Panel (Hierarchy tree, Library)
└── GraphEditorControl (single instance)
    ├── Canvas (nodes, edges)
    └── Inspector (node properties)
```

**State tracking:**
- `rootGraph` / `rootGraphPath` - main loaded graph
- `currentGraphPath` - currently viewed include (or null for root)
- `includeCache` - cached include graphs

**Include navigation:** Double-click fires `IncludeOpenRequested` event, switches single control to display include.

## Proposed Architecture

```
GraphEditorWindow
├── Toolbar (Load/Save/Reset/Close Tab)
├── Left Panel (Hierarchy tree, Library)
└── TabControl
    ├── Tab: "Active: plane.json" [pinned]
    │   └── GraphEditorControl instance
    ├── Tab: "trim_curve.json" [closeable]
    │   └── GraphEditorControl instance
    └── Tab: "damper_common.json *" [closeable, dirty]
        └── GraphEditorControl instance
```

## Data Model

### GraphEditorTab

```csharp
public sealed class GraphEditorTab : INotifyPropertyChanged
{
    public string Id { get; } = Guid.NewGuid().ToString("N");

    // Tab metadata
    public string FilePath { get; set; }           // Absolute path, null for unsaved
    public string DisplayName { get; }             // Computed: filename or "Untitled"
    public bool IsPinned { get; set; }             // True for active graph tab
    public bool IsDirty { get; set; }              // Unsaved changes
    public bool IsActiveGraph { get; set; }        // True for vehicle-bound graph

    // Graph state
    public GraphDefinition Graph { get; set; }
    public GraphEditorControl EditorControl { get; } // Dedicated control instance

    // Undo/redo (future)
    public UndoStack UndoStack { get; }
}
```

### GraphEditorTabManager

```csharp
public sealed class GraphEditorTabManager
{
    public ObservableCollection<GraphEditorTab> Tabs { get; }
    public GraphEditorTab ActiveGraphTab { get; private set; }  // Pinned tab
    public GraphEditorTab SelectedTab { get; set; }             // Currently visible

    // Tab operations
    public GraphEditorTab OpenGraph(string path);       // Opens or switches to existing
    public GraphEditorTab CreateNewTab();               // New untitled graph
    public void CloseTab(GraphEditorTab tab);           // Close if not pinned
    public GraphEditorTab FindTabByPath(string path);   // Find existing tab

    // Active graph management
    public void SetActiveGraph(string path);            // Called on vehicle change
    public void RefreshActiveGraph();                   // Reload from disk
}
```

## Tab Visual Design

### Tab Header Layout

```
┌─────────────────────────────────────────────────────┐
│ [📌 Active: plane.json] [trim_curve.json ×] [damper_common.json * ×] │
└─────────────────────────────────────────────────────┘
```

**Elements:**
- **Pin icon** (📌 or pushpin glyph): Indicates active graph tab, non-closeable
- **Filename**: Display name from path, or "Untitled" for new graphs
- **Dirty indicator** (*): Shown before close button when unsaved changes
- **Close button** (×): Only on non-pinned tabs, right side of tab header

### Tab Header Styling

| Tab Type | Background | Icon | Close Button |
|----------|------------|------|--------------|
| Active graph (selected) | Accent color | Pin | Hidden |
| Active graph (unselected) | Subtle accent | Pin | Hidden |
| Manual tab (selected) | Standard | None | Visible |
| Manual tab (unselected) | Muted | None | Visible |
| Dirty tab | Same as above | None | `* ×` |

### Suggested Colors

```csharp
// Active graph tab - distinctive blue tint
ActiveTabSelected = #1E3A5F      // Dark blue
ActiveTabUnselected = #152A42    // Darker blue
ActiveTabBorder = #3B7DD8        // Accent border

// Manual tabs - neutral
ManualTabSelected = #2D2D30      // VS-style dark
ManualTabUnselected = #1E1E1E    // Darker
ManualTabBorder = #3F3F46        // Subtle border

// Dirty indicator
DirtyColor = #FFCC00             // Yellow asterisk
```

## Behavior Specifications

### Include Double-Click

When user double-clicks an Include node:

1. Extract `IncludePath` from node
2. Resolve to absolute path
3. Check if tab already exists for this path:
   - **Yes**: Switch to existing tab (`SelectedTab = existingTab`)
   - **No**: Create new tab, load graph, add to `Tabs`, select it

```csharp
private void OnIncludeOpenRequested(string includePath)
{
    string resolved = ResolvePath(includePath);

    var existing = TabManager.FindTabByPath(resolved);
    if (existing != null)
    {
        TabManager.SelectedTab = existing;
        return;
    }

    var tab = TabManager.OpenGraph(resolved);
    if (tab == null)
    {
        MessageBox.Show($"Include not found:\n{includePath}");
    }
}
```

### Active Graph Tab Behavior

1. **Auto-creation**: Created when `SetActiveGraph(path)` called (vehicle change)
2. **Auto-refresh**: Reloads from disk when vehicle selection changes
3. **Non-closeable**: Close button hidden, `CloseTab()` ignores pinned tabs
4. **Visual distinction**: Pin icon + accent coloring
5. **Label format**: `"Active: {filename}"` or `"Active: (none)"` if no vehicle

### Tab Close Behavior

1. Click close button (×) on tab header
2. If tab is dirty, prompt: "Save changes to {filename}?"
   - Save: Save file, then close
   - Don't Save: Discard changes, close
   - Cancel: Abort close
3. Remove tab from `Tabs` collection
4. Select adjacent tab (prefer left, fallback right, fallback active)

### Save Behavior

- **Ctrl+S**: Save current tab's graph to its `FilePath`
- **Save button**: Same as Ctrl+S
- If `FilePath` is null, prompt Save As dialog
- Clear dirty flag on successful save
- Update tab display name if path changed

### Load/Open Behavior

- **Load button / Ctrl+O**: Open file dialog
- Check if file already open in a tab:
  - **Yes**: Switch to existing tab
  - **No**: Create new tab with loaded graph

## XAML Structure

### TabControl Template

```xml
<TabControl x:Name="EditorTabs"
            ItemsSource="{Binding Tabs}"
            SelectedItem="{Binding SelectedTab}">
    <TabControl.ItemTemplate>
        <DataTemplate>
            <StackPanel Orientation="Horizontal">
                <!-- Pin icon for active graph -->
                <TextBlock Text="&#xE718;" FontFamily="Segoe MDL2 Assets"
                           Visibility="{Binding IsPinned, Converter={StaticResource BoolToVis}}"
                           Margin="0,0,4,0" FontSize="10"/>

                <!-- Tab label -->
                <TextBlock Text="{Binding DisplayName}"/>

                <!-- Dirty indicator -->
                <TextBlock Text=" *"
                           Visibility="{Binding IsDirty, Converter={StaticResource BoolToVis}}"/>

                <!-- Close button -->
                <Button Content="×"
                        Visibility="{Binding IsPinned, Converter={StaticResource InverseBoolToVis}}"
                        Click="CloseTab_Click"
                        Style="{StaticResource TabCloseButton}"
                        Margin="6,0,0,0"/>
            </StackPanel>
        </DataTemplate>
    </TabControl.ItemTemplate>

    <TabControl.ContentTemplate>
        <DataTemplate>
            <ContentPresenter Content="{Binding EditorControl}"/>
        </DataTemplate>
    </TabControl.ContentTemplate>
</TabControl>
```

## Implementation Phases

### Phase 1: Core Tab Infrastructure
- [x] Create `GraphEditorTab` class with properties
- [x] Create `GraphEditorTabManager` with basic operations
- [x] Add `TabControl` to `GraphEditorWindow.xaml`
- [x] Wire up tab selection and content switching
- [x] Migrate single-control state to tab-based state

### Phase 2: Active Graph Tab
- [x] Implement pinned tab behavior
- [x] Wire `SetActiveGraph()` to vehicle selection events
- [x] Add pin icon and visual distinction
- [x] Ensure non-closeable behavior

### Phase 3: Include Navigation
- [x] Modify `OnIncludeOpenRequested` to open/switch tabs
- [x] Update hierarchy tree to work with tabs (show current tab's includes)
- [x] Handle path resolution relative to each tab's file path

### Phase 4: Tab Close and Save
- [x] Implement close button with dirty prompt
- [x] Track dirty state on graph changes
- [x] Update Save/Load to work with current tab
- [ ] Add keyboard shortcuts (Ctrl+W close, Ctrl+S save)

### Phase 5: Polish
- [x] Tab header styling and colors
- [ ] Tab overflow (scroll or dropdown for many tabs)
- [ ] Drag to reorder tabs (optional)
- [ ] Context menu on tabs (Close, Close Others, Close All)

## Edge Cases

1. **Same file opened twice**: Detect by absolute path, switch to existing tab
2. **Active graph file deleted**: Show error state in tab, allow save-as
3. **External file modification**: Optional: detect and prompt reload
4. **Unsaved new graph**: `FilePath = null`, prompt save-as on close/save
5. **Include path changes**: Tabs track absolute paths, not relative

## Migration Notes

### State to Migrate from GraphEditorWindow

| Current Field | New Location |
|---------------|--------------|
| `rootGraph` | `ActiveGraphTab.Graph` |
| `rootGraphPath` | `ActiveGraphTab.FilePath` |
| `currentGraphPath` | Removed (each tab tracks own path) |
| `includeCache` | Shared across tabs or per-tab (TBD) |
| `GraphEditor` (control) | `SelectedTab.EditorControl` |

### Event Rewiring

- `GraphEditor.IncludeOpenRequested` → Opens new tab or switches
- `GraphEditor.GraphChanged` → Sets `SelectedTab.IsDirty = true`
- Vehicle selection changed → `TabManager.SetActiveGraph(path)`

## Design Decisions

1. **Include cache scope**: Shared across all tabs, with cache invalidation on external save.
   - Implemented in `GraphEditorTabManager.IncludeCache`

2. **Hierarchy tree scope**: Shows includes for selected tab only.
   - Implemented in `RefreshHierarchy()` which reads from `CurrentTab.Graph`

3. **New tab from template**: Shows template picker when creating new tabs.
   - Implemented in `ButtonNew_Click()` using `GraphTemplateSelectorDialog`
