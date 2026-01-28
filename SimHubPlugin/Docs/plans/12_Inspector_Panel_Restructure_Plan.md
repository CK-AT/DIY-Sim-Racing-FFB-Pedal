# Inspector Panel Restructure Plan

## Status (2026-01-27)
- Phase 1-3 completed: preview window extracted; per-node templates for all node types; include handling wired.
- Legacy inspector panel removed; inspector now uses templates only.
- Param inspector uses a property-grid style layout with collapsible Range/UI sections.
- Deviation: signal picker uses a flat ComboBox list (hierarchical picker deferred).

## Goal
Reduce inspector clutter and improve relevance by:
- Moving preview inputs/parameters to a modeless floating window opened on demand.
- Using dedicated inspector controls per node type with only must-have fields.

## Scope
- Graph editor inspector UI and related code-behind.
- Preview inputs/parameters UI relocation to a floating window.

## Current Behavior
- Inspector panel mixes node-specific fields with preview inputs/parameters.
- A single inspector view attempts to cover all node types, resulting in irrelevant fields for many selections.

## Proposed Behavior
- **Modeless Preview Window**: Preview Inputs and Preview Params live in a separate, modeless floating window (open on demand).
- **Per-Node Inspector Views**: Each node type gets its own WPF control (or DataTemplate) showing only must-have fields.
- No global fields in the inspector (Title is not universal and is not shown for some node types).

## Must-Have Fields (TBD)
Suggested baseline fields per node type (intended as a starting point):
- Input node:
  - Signal group selector
  - Output port list (name + add/remove)
  - No live value readout, we have the labels in the graph already
- Output node:
  - Signal group selector
  - Input port list (name + add/remove)
  - Inline validation/warnings for missing sources or name conflicts
- Param node:
  - Signal group selector
  - Output port list (name + add/remove)
  - Per-port default/min/max
  - UI metadata for selected port configured inline (no dialog): widget, label, units, step, precision, log scale, options
- Const node:
  - Constant value
- Op node:
  - Operation selector
  - Input ports with op-specific arity rules
  - Auto-assign meaningful input names based on the selected op
  - Single output (fixed)
- Func node:
  - Function selector
  - Input/output ports are fixed per function definition (no add/remove)
- Include node:
  - Include path (browse/open)
  - Refresh ports action
  - Status/warnings (missing file, invalid include)

Optional advanced section (if desired):
- Node Id
- Node position

## Open Design Decisions
Decisions captured:
- Inspector implementation approach: DataTemplateSelector per node type.
- Preview window launch: toolbar button on right side labeled "Preview Inputs".
- Preview window behavior: modeless; remember size/position if practical (optional).
- Include warnings: icon + tooltip; only show when include path is non-empty and resolution fails (avoid noisy transient states).
- Param UI editing: single column with sections; edits apply to selected port only.
- Op node input naming: see Suggested labels/arity below.
- Future improvement: per-input negation toggle for Add/Mul ops (requires data model + runtime changes).

Suggested op input labels/arity:
- Add: variable inputs; labels `a`, `b`, `c`, ...
- Mul: variable inputs; labels `a`, `b`, `c`, ...
- Min: variable inputs; labels `a`, `b`, `c`, ...
- Max: variable inputs; labels `a`, `b`, `c`, ...
- Sub: 2 inputs; labels `a`, `b`
- Div: 2 inputs; labels `a`, `b`
- Abs: 1 input; label `value`
- Clamp: 3 inputs; labels `value`, `min`, `max`
- Lerp: 3 inputs; labels `a`, `b`, `t`

## Implementation Plan (Hand-off)
### Phase 0: Inventory + architecture choice
1. Inventory current inspector UI elements in `GraphEditorControl.xaml` and code-behind fields that drive them.
2. Choose implementation approach:
   - **Preferred:** `DataTemplateSelector` keyed by `GraphNodeKind` with lean templates.
   - Alternative: dedicated `UserControl` per node type.
3. Define a small shared view-model surface for inspector controls (selected node, graph reference, commands).

### Phase 1: Modeless Preview Window
1. Create `PreviewWindow.xaml/.cs` with existing Preview Inputs/Params lists and live mode toggle.
2. Move preview list bindings and update plumbing from inspector to the new window.
3. Add open/close entry point (toolbar button or inspector button).
4. Wire updates:
   - Graph changes → refresh preview entries.
   - Context selection changes → update preview window (if open).
5. Ensure modeless window does not block editor; handle disposal on editor close.

### Phase 2: Per-node inspector controls
1. Create node-type templates/controls:
   - Input, Output, Param, Const, Op, Func, Include.
2. Implement layout rules per node type (see Must-Haves).
3. Remove generic inspector fields that are no longer applicable.
4. Ensure Param UI metadata is edited inline (no dialog).

### Phase 3: Node-specific logic details
1. **Op node**:
   - Fixed single output.
   - Input list respects op arity.
   - Auto-assign input names based on op (e.g., `a`, `b`, `min`, `max`, `t`).
2. **Func node**:
   - Ports fixed by function definition (no add/remove).
3. **Include node**:
   - Include path browsing, refresh ports, status warnings.

### Phase 4: Cleanup + docs/tests
1. Remove unused inspector UI elements, bindings, and event hooks.
2. Update docs if inspector behavior is referenced elsewhere.
3. Add/adjust GraphTest or UI smoke checks if needed.

## Suggested Implementation Order
1. Extract preview UI into modeless window (can be done independently).
2. Add DataTemplateSelector + first two node types (Const, Op) to validate pattern.
3. Migrate remaining node types (Input/Output/Param/Func/Include).
4. Remove old inspector UI and dead code.

## Developer Notes
- Keep a single source of truth for selected node; avoid parallel state.
- Use UI-thread dispatching for preview window updates.
- Add small helper methods for per-op input naming and arity rules.

## Async/Ordering Considerations
- Preview window uses existing preview evaluation flow; ensure UI updates are dispatched to the UI thread.

## Files to Touch (Expected)
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- New: `SimHubPlugin/GraphEditor/PreviewWindow.xaml` and `.xaml.cs` (or similar)
- Docs as needed

## Validation
- Inspector shows only relevant fields for each node type.
- Preview window opens modelessly and stays in sync with graph selection/preview state.
- No regressions to preview evaluation.
