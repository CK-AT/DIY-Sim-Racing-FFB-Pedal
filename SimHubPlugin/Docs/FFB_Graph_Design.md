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
- Graph schema versioned (v1 for now).
- Nodes:
  - Input: pulls a named input value.
  - Param: pulls a tunable parameter.
  - Const: constant numeric value.
  - Op: arithmetic operations (add/sub/mul/div/min/max/abs/clamp/lerp).
  - Func: known functions (qhat_eff, torque_norm, rpm_norm, assist_loss).
  - Include: references another graph by path or embedded content.
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
- Include node can reference path or embedded graph content.
- Resolver caches graphs and populates a block library index.
- Include outputs are mapped into the parent graph with explicit port names.

## UI Editor Behavior
### Core UX
- Drag nodes, pan/zoom, snap-to-grid.
- Multi-select with Shift, box-select with Shift+drag.
- Delete nodes/edges with Delete.
- Zoom-to-fit with F or context menu.
- Save/Open: Ctrl+S to save, Ctrl+O to open.

### Inspector
- Displays selected node info; preview inputs/params and warnings live in a modeless preview window (toolbar: "Preview Inputs").
- Uses per-node templates (Input/Output/Param/Const/Op/Func/Include) instead of a generic inspector.
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
- JSON schema (versioned) with nodes, ports, links, params.
- Includes can be stored as paths or embedded graphs with block library index.
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
- Editor does not write into live FFB pipeline yet.
- Runtime uses graph output values to feed spring/damper/friction/load.
- Config persistence is separate from graph file storage.
- Graph params are surfaced in the function UI for tuning.
- A single top-level graph is resolved per vehicle `(GameId, CarId)` with a per-game fallback graph.

### Parameter UI Surfacing
Parameters are exposed in function configuration panels based on their `group` attribute:

- **Function Params**: Group = `"FlightStickPitch"`, `"FlightStickRoll"`, `"FlightPedals"`, etc.
  - Shown in respective function config tabs under "FFB Parameters" section
  - Controls dynamically generated from param UI metadata
  - Ordered by graph layout (includes ordered at the Include node position)
  - Replace legacy X-Plane FFB sliders with graph-driven params

- **System Params**: Group = `"System"`
  - Shown in System tab under "System Parameters" section
  - Global settings like Vref, nominal RPM, torque references

**UI Generation**:
- `GraphParamControlBuilder` creates WPF controls based on param metadata
- Supports: slider, knob, checkbox, enum, text widget types
- Changes update vehicle profile immediately (in-memory)
- Explicit save required to persist to profile file

**Future Enhancement**:
- Add tooltips with mini-curves and live cursors for visual feedback on param nodes

## Testing
- GraphTest (runtime model validation and evaluator checks).
- PluginTest (editor JSON roundtrip + preview evaluator).

## Open Questions
- Should we add typed ports or keep all numeric?
- Should we allow stateful nodes (e.g., integrator, delay)?
- How should per-vehicle graph overrides be surfaced and edited?

## Risks and Mitigations
- Param UI schema complexity: start with slider/knob/checkbox, add advanced widgets later.
- Backward compatibility: keep param IDs stable, provide migration for renamed params.
- Duplicate signal names: enforce uniqueness within a graph, warn on collisions.
- Unit mismatches: add optional unit metadata and validator warnings.
- Include versioning: allow pinning include hashes or explicit version tags.
- Runtime failure: fall back to last-known-good compiled graph.
- Performance: cache compiled graphs per vehicle and invalidate on edits.
