# Include Context Auto-Select Plan

**Status**: DONE
**Created**: 2026-01-26
**Completed**: 2026-01-26

## Overview

Two related improvements to the Include Context Preview feature:

1. **Auto-select context on Include double-click**: When live mode is active and user double-clicks an Include node, automatically select that Include's context in the newly opened tab.

2. **Resilient context selection**: Don't automatically fall back to "(standalone)" on spurious live data dropouts. The current behavior resets context selection when the context cache is temporarily empty, causing UX disruption.

## Problem Statement

### Issue 1: Manual Context Selection After Navigation

**Current behavior**:
- User double-clicks Include node → included graph opens in new tab
- Context dropdown shows available contexts but defaults to "(standalone)"
- User must manually select the context to see live values from the parent

**Desired behavior**:
- When live mode is active, double-clicking an Include node should:
  1. Open the included graph (existing behavior)
  2. Auto-select that specific Include node's context in the new tab

### Issue 2: Spurious Context Resets

**Current behavior** (in `RefreshPreview()` at line 2104-2107):
```csharp
// Context no longer available, fall back to standalone
_selectedContextId = null;
RefreshContextDropdown();
return;
```

And in `RefreshContextDropdown()` at line 2283-2293:
```csharp
if (contexts == null || contexts.Count == 0)
{
    if (PanelEvalContext.Visibility != Visibility.Collapsed)
    {
        ComboEvalContext.Items.Clear();
        ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });
        PanelEvalContext.Visibility = Visibility.Collapsed;
        _selectedContextId = null;  // <-- Problematic reset
        _lastContextIds.Clear();
    }
    return;
}
```

**Problem**: The context cache is cleared at the start of each top-level evaluation cycle (by design). If a timer-based refresh fires during the brief window when the cache is empty, the context selection is lost.

**Desired behavior**: Keep the context selection sticky. Only clear it when:
- User explicitly selects "(standalone)"
- The graph file changes
- The Include node is confirmed removed from the parent graph (not just temporarily absent from cache)

## Implementation Plan

### Part 1: Auto-Select Context on Include Double-Click

#### Step 1.1: Extend IncludeOpenRequested Event

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Change the event signature to pass the Include node ID:

```csharp
// Line 97 - Change from:
public event Action<string> IncludeOpenRequested;

// To:
public event Action<string, string> IncludeOpenRequested;  // (path, includeNodeId)
```

#### Step 1.2: Update Double-Click Handler

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Update `Node_MouseLeftButtonDown` (around line 971-977):

```csharp
// Change from:
if (e.ClickCount == 2 && node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
{
    IncludeOpenRequested?.Invoke(node.IncludePath);
    e.Handled = true;
    return;
}

// To:
if (e.ClickCount == 2 && node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
{
    // Pass node.Id so the new tab can auto-select this context
    string contextId = _liveInputsEnabled ? node.Id : null;
    IncludeOpenRequested?.Invoke(node.IncludePath, contextId);
    e.Handled = true;
    return;
}
```

Also update other call sites (lines 3659, 3943) to pass `null` for contextId if they don't have a specific context.

#### Step 1.3: Update Event Handler in Window

**File**: `GraphEditor/GraphEditorWindow.xaml.cs`

Update `OnIncludeOpenRequested` (line 516):

```csharp
// Change from:
private void OnIncludeOpenRequested(string path)

// To:
private void OnIncludeOpenRequested(string path, string contextId)
```

Update the handler logic:

```csharp
private void OnIncludeOpenRequested(string path, string contextId)
{
    if (string.IsNullOrWhiteSpace(path))
    {
        return;
    }

    // Resolve relative to current tab's base directory
    string baseDir = CurrentTab?.BaseDirectory;
    string resolved = GraphEditorTabManager.ResolvePath(path, baseDir);

    // Try to open or switch to existing tab
    var tab = tabManager.OpenGraph(resolved);
    if (tab == null)
    {
        MessageBox.Show(this, $"Include not found:\n{path}", "Include not found",
            MessageBoxButton.OK, MessageBoxImage.Warning);
        return;
    }

    EditorTabs.SelectedItem = tab;
    RefreshHierarchy();

    // Auto-select context if provided and live mode is active
    if (!string.IsNullOrEmpty(contextId))
    {
        tab.EditorControl.SetSelectedContext(contextId);
    }
}
```

#### Step 1.4: Add SetSelectedContext Method

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Add new public method:

```csharp
/// <summary>
/// Programmatically selects a context by Include node ID.
/// Called when navigating to an include graph via double-click while live mode is active.
/// </summary>
public void SetSelectedContext(string contextId)
{
    _selectedContextId = contextId;
    RefreshContextDropdown(force: true);
    RefreshPreview();
    ContextChanged?.Invoke(this, _selectedContextId);
}
```

#### Step 1.5: Update Event Subscription

**File**: `GraphEditor/GraphEditorWindow.xaml.cs`

Update the subscription in `WireTabParamChanges` (line 163):

```csharp
// Change from:
tab.EditorControl.IncludeOpenRequested += OnIncludeOpenRequested;

// To (if signature changed):
tab.EditorControl.IncludeOpenRequested += (path, contextId) => OnIncludeOpenRequested(path, contextId);
```

And the unsubscription in `OnTabRemoved` (line 228) - may need to use a named handler for proper unsubscription.

---

### Part 2: Resilient Context Selection

#### Step 2.1: Add Sticky Context Flag

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Add field to track user-initiated vs auto context:

```csharp
private bool _contextIsUserSelected;  // True if user explicitly selected a context
```

#### Step 2.2: Update RefreshContextDropdown

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Modify `RefreshContextDropdown()` to preserve sticky context:

```csharp
private void RefreshContextDropdown(bool force = false)
{
    if (_contextProvider == null || string.IsNullOrEmpty(_filePath))
    {
        // Only clear selection if not user-selected, or if truly disconnected
        if (!_contextIsUserSelected)
        {
            if (PanelEvalContext.Visibility != Visibility.Collapsed)
            {
                ComboEvalContext.Items.Clear();
                ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });
                PanelEvalContext.Visibility = Visibility.Collapsed;
                _selectedContextId = null;
                _lastContextIds.Clear();
            }
        }
        return;
    }

    // Normalize path for cache lookup
    string normalizedPath = _filePath;
    try
    {
        normalizedPath = System.IO.Path.GetFullPath(_filePath);
    }
    catch { }

    var contexts = _contextProvider(normalizedPath);

    if (contexts == null || contexts.Count == 0)
    {
        // Cache is temporarily empty - DON'T reset if user had selected a context
        // The cache will be repopulated on next evaluation cycle
        if (!_contextIsUserSelected || _selectedContextId == null)
        {
            if (PanelEvalContext.Visibility != Visibility.Collapsed)
            {
                ComboEvalContext.Items.Clear();
                ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });
                PanelEvalContext.Visibility = Visibility.Collapsed;
                _selectedContextId = null;
                _lastContextIds.Clear();
            }
        }
        // If user had a context selected, keep the dropdown visible but show stale state
        // (or hide it but preserve _selectedContextId for when cache repopulates)
        return;
    }

    // ... rest of existing logic ...
}
```

#### Step 2.3: Update RefreshPreview for Resilience

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Modify the context fallback in `RefreshPreview()` (around line 2102-2108):

```csharp
if (ctx != null)
{
    // ... existing input/parameter extraction ...
}
else
{
    // Context not found in cache - could be temporary dropout
    if (_contextIsUserSelected)
    {
        // Don't fall back to standalone immediately; wait for cache to repopulate
        // Use last known values or skip this refresh cycle
        return;
    }

    // Not user-selected, safe to fall back
    _selectedContextId = null;
    RefreshContextDropdown();
    return;
}
```

#### Step 2.4: Track User Selection

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Update `ComboEvalContext_SelectionChanged` (line 2334):

```csharp
private void ComboEvalContext_SelectionChanged(object sender, SelectionChangedEventArgs e)
{
    if (ComboEvalContext.SelectedItem is ComboBoxItem item)
    {
        _selectedContextId = item.Tag as string;
        _contextIsUserSelected = (_selectedContextId != null);  // Track that user made a choice
        UpdateParamControlsEnabled();
        RefreshPreview();
        ContextChanged?.Invoke(this, _selectedContextId);
    }
}
```

Also update `SetSelectedContext` to set the flag:

```csharp
public void SetSelectedContext(string contextId)
{
    _selectedContextId = contextId;
    _contextIsUserSelected = (contextId != null);
    RefreshContextDropdown(force: true);
    RefreshPreview();
    ContextChanged?.Invoke(this, _selectedContextId);
}
```

#### Step 2.5: Clear Flag on Graph/File Change

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

Reset the sticky flag when loading a new graph:

```csharp
// In LoadGraph() or SetGraph() method:
_contextIsUserSelected = false;
_selectedContextId = null;
```

---

## Files to Modify

| File | Changes |
|------|---------|
| `GraphEditor/GraphEditorControl.xaml.cs` | Extended event signature, SetSelectedContext method, sticky context logic |
| `GraphEditor/GraphEditorWindow.xaml.cs` | Updated event handler, pass contextId on include navigation |

## Build Commands

### Build GraphTest (for unit tests)

```bash
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\GraphTest.csproj" \
  /p:Configuration=Debug /v:minimal /nologo
```

### Run GraphTest

```bash
"d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\bin\Debug\net48\GraphTest.exe"
```

### Build Main Plugin (Debug)

```bash
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Debug /v:minimal /nologo
```

### Build Main Plugin (Release)

```bash
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Release /v:minimal /nologo
```

**Note**: The MSBuild path may vary. Use VS2022 path shown above, or find the correct path with:

```bash
"/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
  -latest -requires Microsoft.Component.MSBuild \
  -find "MSBuild\*\*\Bin\MSBuild.exe"
```

## Testing Checklist

### Auto-Select Context Tests

- [ ] Double-click Include node with live mode OFF → opens tab with "(standalone)" selected
- [ ] Double-click Include node with live mode ON → opens tab with Include's context auto-selected
- [ ] Tab label shows "(via IncludeTitle)" suffix when context auto-selected
- [ ] Preview shows live values from parent graph's inputs
- [ ] Switching to "(standalone)" works after auto-select

### Resilient Context Tests

- [ ] Select a context manually → context persists across multiple refresh cycles
- [ ] Simulate cache dropout (rapid refresh) → context selection not lost
- [ ] Change to different graph file → context selection properly reset
- [ ] User selects "(standalone)" → selection not "sticky" (can be changed by system)

## Key Code Locations

| Component | File | Line |
|-----------|------|------|
| IncludeOpenRequested event | `GraphEditorControl.xaml.cs` | 97 |
| Double-click handler | `GraphEditorControl.xaml.cs` | 971-977 |
| OnIncludeOpenRequested | `GraphEditorWindow.xaml.cs` | 516 |
| RefreshContextDropdown | `GraphEditorControl.xaml.cs` | 2256 |
| RefreshPreview context fallback | `GraphEditorControl.xaml.cs` | 2102-2108 |
| ComboEvalContext_SelectionChanged | `GraphEditorControl.xaml.cs` | 2334 |
| _liveInputsEnabled field | `GraphEditorControl.xaml.cs` | 33 |

## Potential Side Effects

1. **Event signature change**: Any code subscribing to `IncludeOpenRequested` with the old signature will break at compile time. This is intentional - ensures all call sites are updated.

2. **Sticky context memory**: If user selects a context for graph A, then loads graph B, then reloads graph A, the context may not be preserved (depends on whether we persist per-file context selection). Current plan is to NOT persist - context clears on file change.

3. **Race condition window**: There's still a brief window where context cache is empty. The resilient design handles this by not resetting during that window, but preview values may be stale for one cycle.

## Success Criteria

- [ ] All existing GraphTest tests pass (50 tests)
- [ ] All KinematicsTests pass (18 tests)
- [ ] Manual testing confirms auto-select works
- [ ] Manual testing confirms context survives spurious dropouts
- [ ] No regression in FFB runtime values
- [ ] Update `FFB_Graph_Progress.md` to move item to Done
