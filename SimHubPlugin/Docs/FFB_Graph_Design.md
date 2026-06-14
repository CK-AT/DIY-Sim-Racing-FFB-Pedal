# FFB Graph Design (Draft)

Status: Working design doc for the node-graph based FFB pipeline and editor UI.  
Scope: Graph runtime model, UI editor behaviors, storage format, and integration boundaries.

## Goals
- Make FFB tuning more flexible and composable without code changes.
- Provide a clear contract between runtime evaluation and UI editing.
- Keep the graph execution deterministic and fast enough for per-frame evaluation.
- Allow reuse via includes and a shared block library.
- Keep everything in a single SimHub plugin DLL.

## Non-goals
- Full visual scripting language (loops, conditionals, or stateful nodes).
- Real-time collaborative editing.
- GPU-accelerated evaluation or live graph profiling.

## Core Concepts
### Graph Definition
- Directed acyclic graph (DAG) of nodes with typed input/output ports.
- Graph schema versioned (currently v4).
- Nodes:
  - Input: pulls a named input value.
  - Param: pulls a tunable parameter.
  - Const: constant numeric value.
  - Op: arithmetic operations (add/sub/mul/div/min/max/abs/neg/clamp/lerp) with output ports labeled by formula; add/mul/min/max accept variable input counts and expand the output label to match (e.g., "a+b+c"). Add/mul inputs can be negated per-port and the output label reflects negation (e.g., "a+b-c", "a*b*-c").
  - Func: known functions (qhat_eff, torque_norm, rpm_norm, assist_loss).
  - Include: references a sub-graph, either **file-backed** (by `IncludePath`) or **embedded** (an inline sub-graph definition stored in the parent node — no path). The two forms are mutually exclusive; a node is embedded iff it carries an inline graph and has no path.
  - Output: exposes a named output.
- Nodes can have multiple input/output ports to reduce total node count.
- Layout metadata (positions, collapsed state, group/section) is stored in JSON.

### Evaluation
- Graphs are compiled into a fast evaluation plan (topo order + node ops).
- Pure evaluation: no side effects, no state in nodes.
- Shared evaluator used by runtime and editor preview.
- Missing inputs default to 0.0.
- Division by near-zero returns 0.0.

### Include Resolution
- Include node can reference a file path or carry embedded (inline) graph content.
- File-backed includes are cached by the resolver and registered in the block library index for reuse across graphs.
- Embedded sub-graphs are evaluated directly from memory (cached per-node by `"inline:" + nodeId`) with **no disk spill** — the resolver no longer materializes inline blocks to `_embedded/{hash}.json`. Embedded blocks are private to their parent and are deliberately *not* registered in the block library (no reuse).
- Include outputs are mapped into the parent graph with explicit port names. For both forms, the Include node's ports are derived from the sub-graph's Input/Output nodes (re-derived on load, not stored on the Include node itself).

### Embedded Sub-Graphs
- An embedded sub-graph is an Include node whose definition lives **inline** in the parent rather than referenced by file path. Use it for one-off, template-specific clusters (e.g. a TR-gate, mu-buzz ramp, or ground-cue) that would otherwise clutter the top level. Reuse across templates stays file-based.
- Stored in the editor JSON as a nested `Inline` block on the Include node DTO; embedding nests recursively (an embedded sub-graph may itself contain embedded includes). Ports are not serialized — they are re-derived from the inline graph's Input/Output nodes on load (`PopulateIncludePorts`), and re-sync live when the sub-graph's interface changes.
- **Library graphs** (`IsLibraryGraph`, schema v4): a reusable sub-graph flagged as a library block whose Input/Output nodes use freeform port names (no signal-catalog binding), so the parent supplies the wiring. Library graphs are file-backed for sharing; embedding is for the non-reused case.

#### Editor operations (right-click context menu)

- **Add Embedded Sub-Graph** — inserts an Include node carrying a blank inline graph.
- **Group N Nodes into Embedded Sub-Graph** — collapses the current selection into a new embedded Include: boundary-crossing links become deduped Input/Output ports, the parent is rewired automatically, and the moved contents are anchored near the sub-graph canvas top-left.
- **Extract Embedded Sub-Graph to File…** — writes an inline block out to a chosen `_embedded/*.json`, sets `IncludePath`, and clears the inline content (embedded → file-backed).
- **Inline This Include (detach from file)** — reads a file include into the node's inline content and clears `IncludePath` (file-backed → embedded). The shared file is left in place; only this node detaches.

#### Embedded sub-graph tabs

- Double-clicking a path-less Include opens its inline graph in its own editor tab, titled `parent/node` (nesting chains, e.g. `file/outer/inner`).
- Edits flush back into the parent Include node's inline content and mark the parent dirty (there is no standalone file). Saving cascades up to the root file tab; an embedded tab never prompts for a filename, and closing it never prompts to save (its content already lives in the parent).
- Embedded tabs key on `(parentTab, nodeId)` for dedupe, not a file path.

#### Include port reordering

- Per-node `InputPortOrder` / `OutputPortOrder` (lists of port names, serialized) let the parent fix the display order of an Include node's derived ports for tidy wiring. This is purely cosmetic — links and maps are name-keyed, so reordering never breaks wiring. Unknown names are ignored and new ports append in derived order. The inspector exposes ▲/▼ buttons to reorder.

## UI Editor Behavior
### Core UX
- Drag nodes, pan/zoom, snap-to-grid.
- Multi-select with Shift, box-select with Shift+drag.
- Delete nodes/edges with Delete.
- Zoom-to-fit with F or context menu.
- Save/Open: Ctrl+S to save, Ctrl+O to open.

### Inspector
- Displays selected node info; preview inputs/params and warnings live in a modeless preview window (toolbar: "Preview Inputs").
- Preview evaluation is throttled (500ms) and skips refresh when live inputs are unchanged.
- Preview runtime/evaluator are cached per graph until the graph or resolver context changes.
- Uses per-node templates (Input/Output/Param/Const/Op/Func/Include) instead of a generic inspector.
- Inspector text inputs commit edits on Enter to avoid lost changes on selection switches.
- Param nodes use an IDE-style property grid for the selected port with Range/UI sections (collapsible).
- Include ports are read-only lists derived from the included graph interface (refreshable).
- Param nodes also define their tuning controls (slider/knob/checkbox) to enable direct tuning.

### Inputs, Outputs, Params Naming
- Use a structured dot-notation convention (not a path): `<FunctionName>.<Source>.<Signal>...`
- Outputs follow the same convention to keep routing consistent.
- Params are grouped by function and are shown in the associated function UI.
- Example mappings:
  - Output: `FlightStickPitch.SpringGain`
  - Input: `FlightStickRoll.Force`
  - Input: `XPlane.AeroTorques.Roll`
  - Input: `SimHub.CarId`

### Param UI Schema (Draft)
- Param nodes declare their control type and UI metadata.
- Proposed fields:
  - `widget`: `slider` | `knob` | `checkbox` | `enum` | `text`
  - `min`, `max`, `step`, `default`, `precision`
  - `units`: string (e.g., `N`, `N/mm`, `Hz`)
  - `label`: display label for the control
  - `group`: UI grouping key (function name or section)
  - `logScale`: bool for decades-spanning gains
  - `options`: enum values (`value`, `label`)

### Graph Hierarchy
- Tree panel showing root graph and include graphs.
- Open include from inspector or context menu.
- Library list for cached/embedded blocks.

## Data Model and Persistence
- JSON schema (versioned, currently v4) with nodes, ports, links, params.
- Includes are stored either as a file path (`IncludePath`, registered in the block library index) or as an embedded sub-graph (a nested `Inline` block on the Include node — recursive, private to the parent, never registered for reuse).
- Include node ports are not serialized; they are re-derived from the sub-graph's Input/Output nodes on load. Optional `InputPortOrder` / `OutputPortOrder` lists are serialized to fix cosmetic port display order.
- `IsLibraryGraph` (v4) flags a reusable sub-graph whose Input/Output nodes use freeform port names instead of signal-catalog binding.
- Paths can be stored relative to the root graph directory.
- Layout data persists in the JSON to preserve editor state.

### Three-Tier Parameter System
Parameters support cascading overrides at three levels (later overrides earlier):

1. **Include/Graph Default**: Defined in param definition (`defaultValue`)
2. **Graph Override**: Stored in root graph `paramValues` dictionary
3. **Vehicle Profile Override**: Stored in `AircraftFfbProfile.GraphParamValues`

**Rationale**: This allows:
- Reusable include graphs with sensible defaults
- Graph-level tuning shared across vehicles
- Per-vehicle fine-tuning without modifying shared graphs

**Storage**:
```json
{
  "version": 1,
  "nodes": [...],
  "params": {
    "FlightStickPitch.SpringGain": {
      "defaultValue": 1.0,
      "min": 0.0,
      "max": 10.0,
      "ui": {"widget": "slider", "group": "FlightStickPitch", ...}
    }
  },
  "paramValues": {
    "FlightStickPitch.SpringGain": 1.5  // Graph-level override
  }
}
```

Vehicle profile (in plugin settings):
```json
{
  "GraphParamValues": {
    "FlightStickPitch.SpringGain": 2.0  // Vehicle-specific override
  }
}
```

## Integration Boundaries
- Editor applies changes into the live FFB pipeline when the user clicks Apply (runtime evaluator is updated in-place).
- Runtime uses graph output values to feed spring/damper/friction/trim/buffet/load.
- Legacy X-Plane math paths are removed; graph outputs are the single source of flight FFB.
- Config persistence is separate from graph file storage.
- Graph params are surfaced in the function UI for tuning.
- A single top-level graph is resolved per vehicle `(GameId, CarId)` stored in the vehicle profile.

### Parameter UI Surfacing
Parameters are exposed in function configuration panels based on their `group` attribute:

- **Function Params**: Group = `"FlightStickPitch"`, `"FlightStickRoll"`, `"FlightPedals"`, etc.
  - Shown in respective function config tabs under "FFB Parameters" section
  - Controls dynamically generated from param UI metadata
  - Ordered by graph layout (includes ordered at the Include node position)
  - Replace legacy X-Plane FFB sliders with graph-driven params

- **System Params**: Group = `"System"`
  - Shown in Vehicle tab under "System Parameters" section
  - Global graph constants (e.g., Vref, rotor references) when present in the active graph

**UI Generation**:
- `GraphParamControlBuilder` creates WPF controls based on param metadata
- Supports: slider, knob, checkbox, enum, text widget types
- Changes update vehicle profile immediately (in-memory)
- Explicit save required to persist to profile file

**Future Enhancement**:
- Add tooltips with mini-curves and live cursors for visual feedback on param nodes

### Profile Browser Dialog
A unified dialog for template selection, profile management, and profile import:

- **Templates tab**: Graph templates from registry (no tuning data)
- **My Vehicles tab**: Stored profiles with graph path and tuning indicators
- **Import File**: Load exported profile files

Actions:

- "Use Graph Only": Apply graph without copying tuning
- "Use Graph + Tuning": Apply graph and copy param values
- Delete/Export profiles from My Vehicles tab

### Shared Graph Save Protection

When saving changes to a graph file used by multiple vehicles:

- Warning dialog lists affected vehicles
- Offers "Save As" to create vehicle-specific copy
- Prevents accidental changes to shared templates

### Parameter Override Preservation

When changing a vehicle's graph assignment:

- Param overrides are preserved if the param name and definition hash match
- Allows graph upgrades without losing per-vehicle tuning
- Non-matching params are discarded (graph structure changed)

### Vehicle Switch Workflow

When switching vehicles with unsaved param changes:

- Changes are snapshot before switch
- User prompted to Save or Discard
- Discard reverts to last-saved profile state

## Testing
- GraphTest (runtime model validation and evaluator checks).
- PluginTest (editor JSON roundtrip + preview evaluator).

## Open Questions

- Should we add typed ports or keep all numeric?
- Should we allow stateful nodes (e.g., integrator, delay)?

## Risks and Mitigations
- Param UI schema complexity: start with slider/knob/checkbox, add advanced widgets later.
- Backward compatibility: keep param IDs stable, provide migration for renamed params.
- Duplicate signal names: enforce uniqueness within a graph, warn on collisions.
- Unit mismatches: add optional unit metadata and validator warnings.
- Include versioning: allow pinning include hashes or explicit version tags.
- Runtime failure: fall back to last-known-good compiled graph.
- Performance: cache compiled graphs per vehicle and invalidate on edits.
