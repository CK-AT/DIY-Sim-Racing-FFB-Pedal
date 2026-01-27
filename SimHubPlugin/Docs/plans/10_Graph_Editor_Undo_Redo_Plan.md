# Graph Editor Undo/Redo Plan

**Status**: PLANNED

## Goals
- Add undo/redo for graph editing actions across nodes, ports, links, and inspector edits.
- Maintain per-tab history (each graph tab has its own undo stack).
- Preserve correct dirty tracking (undo/redo should toggle dirty state correctly).
- Keep runtime/editor preview behavior stable (no divergence between graph state and preview).

## Non-Goals (Initial Scope)
- Cross-tab undo/redo.
- Persisting undo history across sessions.
- Complex multi-selection state restoration (optional).

## Current State (Relevant Areas)
- Graph editing and inspector updates in `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`.
- Per-tab state in `SimHubPlugin/GraphEditor/GraphEditorTab.cs`.
- Graph serialization via `GraphSerializer.Serialize/Deserialize`.
- Dirty tracking via `GraphEditorTab.IsDirty` set in `GraphEditorWindow` GraphChanged handler.

## Proposed Design

### Data Model
Add an undo/redo stack owned by each `GraphEditorTab`.

```csharp
public sealed class GraphUndoStack
{
    public bool CanUndo { get; }
    public bool CanRedo { get; }
    public bool IsDirty { get; } // optional: managed by stack vs tab

    public void PushSnapshot(GraphDefinition graph, GraphEditorViewState view);
    public UndoSnapshot Undo();
    public UndoSnapshot Redo();
    public void Clear();
}

public sealed class UndoSnapshot
{
    public string GraphJson { get; init; }
    public GraphEditorViewState View { get; init; }
    public string SelectionNodeId { get; init; } // optional
}

public sealed class GraphEditorViewState
{
    public double ScaleX { get; init; }
    public double ScaleY { get; init; }
    public double TranslateX { get; init; }
    public double TranslateY { get; init; }
}
```

### Snapshot Strategy
- Use `GraphSerializer.Serialize` to capture graph snapshots (single source of truth).
- Use `GraphSerializer.Deserialize` to restore snapshots.
- Keep the view state (pan/zoom) as part of the snapshot.
- Optional: store selected node id to restore focus after undo/redo.

### Capture Rules
Capture a snapshot after user-facing graph mutations:
- Node add/remove/duplicate.
- Port add/remove/rename.
- Link add/remove/rewire.
- Inspector edits (Title, Op, Func, Const, Include path, SignalGroup, Param UI changes).
- Node movement / multi-select move (capture once at end of drag).

Avoid snapshot spam:
- Debounce text edits (e.g., Include path, Title, Const) to capture at edit commit or focus loss.
- For drag/move operations, capture only on mouse-up.

### Dirty Tracking
Track a "baseline snapshot index" per tab:
- When loading a graph or saving, record the current stack index as clean baseline.
- `IsDirty` = `CurrentIndex != BaselineIndex`.
- Undo/redo should update `IsDirty` accordingly without re-triggering GraphChanged loops.

### UI Integration
Add Undo/Redo buttons and shortcuts:
- Toolbar buttons in `GraphEditorWindow.xaml`.
- Keyboard shortcuts: `Ctrl+Z` undo, `Ctrl+Y` redo (and/or `Ctrl+Shift+Z`).
- Button enable state binds to `CanUndo`/`CanRedo`.

### EditorControl Integration
Provide methods on `GraphEditorControl`:
- `CaptureUndoSnapshot()` after a mutation.
- `RestoreUndoSnapshot(UndoSnapshot snapshot)` to rehydrate graph + view state + selection.
- Suppress GraphChanged and preview refresh loops while restoring.

### Multi-Tab Behavior
- Each `GraphEditorTab` owns a `GraphUndoStack`.
- `GraphEditorWindow` routes undo/redo commands to the active tab only.
- On tab switch, buttons reflect the selected tab's stack state.

## Implementation Steps
1. **Add UndoStack classes**
   - New files: `GraphEditor/GraphUndoStack.cs` (and small model classes).
   - Unit tests for stack behavior in `GraphTest` (snapshot push/undo/redo ordering).
2. **Wire snapshots into editor**
   - Identify graph mutation points in `GraphEditorControl.xaml.cs`.
   - Capture snapshots for add/remove/link/rename/drag/inspector commits.
   - Add `RestoreUndoSnapshot` and guard against re-entrant GraphChanged events.
3. **Per-tab ownership**
   - Add `GraphUndoStack` to `GraphEditorTab`.
   - Set baseline index after load/save.
4. **UI Commands**
   - Add toolbar buttons (Undo/Redo).
   - Wire shortcuts in `GraphEditorWindow`.
   - Update enable state on selection change and after undo/redo.
5. **Validation**
   - Manual tests (below).
   - Automated tests for stack logic and serializer round-trips.

## Manual Test Checklist
- Add node → Undo → node removed → Redo → node restored.
- Move node(s) → Undo restores position.
- Add/remove link → Undo/Redo works.
- Rename port → Undo/Redo restores name.
- Change SignalGroup → port options/labels restored.
- Edit include path → Undo/Redo restores include and ports.
- Undo to clean state → tab dirty indicator clears.
- Undo/redo per tab isolated (switch tabs).

## Build & Test Commands
Build the SimHub plugin (Debug):
```
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" /p:Configuration=Debug /v:minimal /nologo
```

Graph tests (if new unit tests are added):
- Use existing GraphTest harness in `SimHubPlugin/GraphTest` (add tests to `GraphTestRunner`).

## Open Questions
- Should undo/redo restore selection and inspector state beyond selected node id? **Answer:** No.
- Should snapshot include include-port cache or rely on re-derive (`PopulateIncludePorts`)? **Answer:** Rely on re-derive for now.
- Do we want to coalesce text edits into a single snapshot (e.g., 300 ms debounce)? **Answer:** Yes, implement debounce/coalescing.
