# FFB Graph Roadmap (Draft)

Status: iteration plan for the node-graph system and editor.

## Phase 1: Core Editor and Preview (Done)
- Graph editor control with pan/zoom, node creation, links.
- Inspector with live preview and basic editing.
- Include support with port add/remove and path selection.
- Hierarchy panel for root/include navigation.

## Phase 2: Runtime Integration
- Store graph selection per function and per axis.
- Map graph outputs into spring/damper/friction/load contributions.
- Provide default graph templates per aircraft type.
- Add validation and safe fallback to legacy tuning.
- Add compilation step for runtime evaluation (precompute topo order and node op tables).
- Introduce structured naming for inputs/outputs/params with function/grouping in the UI.
- Define param UI schemas (slider/knob/checkbox) owned by param nodes.

## Phase 3: Usability Enhancements
- Node search and quick-create palette.
- Undo/redo.
- Port type hints and unit labels.
- Graph diff/merge support for profiles.
- Multi-port input/output nodes to reduce graph clutter.
- Persist editor layout metadata in JSON and expose reset/auto-layout tools.

## Phase 4: Advanced Nodes
- Saturation, deadzone, and smoothing nodes.
- Curve nodes (spline, piecewise).
- State nodes (limited integration, delay) with guardrails.

## Phase 5: Tuning Workflow
- Wizard to auto-generate graphs from current tuning values.
- Reference flight capture hooks for tuning nodes.
- Per-aircraft graph library packs.

## Risks / Mitigations
- Performance: keep evaluator allocations minimal; cache topological order.
- Complexity creep: enforce node whitelist and schema version gating.
- User error: strong validation + safe defaults.
- Param UI schema: start with slider/knob/checkbox; add advanced widgets later.
- Compatibility: keep stable param IDs and provide migrations for renames.
- Naming collisions: enforce uniqueness within a graph and warn on conflicts.
- Units: add unit metadata and validator warnings for mismatches.
- Include versioning: allow pinning include hashes or explicit version tags.
- Runtime failures: fall back to last-known-good compiled graph.
