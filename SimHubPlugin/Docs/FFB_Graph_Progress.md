# FFB Graph Progress

Status: living progress document for graph editor/runtime integration.

## Done
- Graph editor surface (pan/zoom, add nodes, wiring).
- Inspector with live preview and node editing.
- Include support (ports, path picker, open include).
- Hierarchy tree and library browser.
- Edge selection, hover styling, context menu delete.
- Rounded nodes, snap-to-grid, zoom-to-fit.
- Edge routing (curved paths) and edge tooltips.
- Node context menu (delete/duplicate).
- Node alignment/distribution tools and edge curvature adjustment.
- Edge reroute handles (manual Bezier control).
- Edge rewiring by drag + preview edge while dragging.
- Auto-sized nodes based on content (title/ports).
- Edge context menu can reset reroute handles.
- Port handles appear on selected nodes and during edge drag for quick rewiring.
- Edge preview reinitializes after graph rebuilds.
- Op/Func nodes default their operation label and refresh titles on changes.
- Edge rewiring supports dragging to a new source (via context menu).
- Input ports now enforce a single incoming edge.
- Node sizing now uses measured label widths for tighter layouts.
- Edge drag rewires source vs target based on pickup proximity.
- Node padding/min width reduced for tighter layouts.
- Port stacking now aligns inputs/outputs by kind so edge anchors match ports.
- Func nodes show selected function in their titles.
- Edge preview uses the correct port anchor when starting from inputs.
- Port handles appear when hovering near a node for quick edge drawing.
- Input/param/output nodes can add multiple ports via inspector buttons.
- Vehicle-level graph selection model defined (per `(GameId, CarId)` with game fallback).
- Plugin resolves active vehicle graph path and loads/validates on vehicle changes.
- X-Plane system tab exposes vehicle/game graph path pickers with active graph status.
- Added sample test graphs (plane, heli collective, multi-function) for validation.
- Converted sample graphs to editor JSON schema for proper display.
- Graph editor auto-loads the active vehicle graph when opened.
- Graph editor refreshes the active graph when the selection changes.
- Runtime graph evaluation is wired (inputs/params + cached evaluator) with output mapping.
- Output port labels show live preview values in the editor.
- Runtime mapping now applies graph outputs to spring/damper/friction/load/trim per function.
- Reverted initial centering/zoom changes after pan/zoom regressions.
- Restored grid to canvas background and removed auto-centering while stabilizing pan/zoom.
- Signal catalog document with hierarchical keys.
- Input/output ports use a hierarchical selector with current-value display.
- Param nodes expose per-port default/min/max values in the inspector.
- Runtime mapping uses per-port IDs for input/param/output nodes.
- Sample graphs bind to real input/output signal names.
- Live inputs toggle feeds preview from latest telemetry snapshots.
- Right mouse button pans; left drag selects.
- Multi-select node drag moves all selected nodes.
- Func ports match function signatures (e.g. assist_loss uses a single input).
- Compiled evaluator precomputes node order/index and is used for preview/runtime with parity test.
- Param nodes store UI metadata (widget/label/group/units/etc.) per port and expose it in the inspector.
- Param UI editor dialog replaces inline fields (including default/min/max) for cleaner editing.
- Param nodes render widget-based controls for direct tuning in the canvas.
- Three-tier param resolution: include defaults → graph overrides → vehicle profile overrides.
- Param values stored in `GraphDefinition.ParamValues` (graph-level) and `AircraftFfbProfile.GraphParamValues` (vehicle-level).
- `GraphParamControlBuilder` utility for generating WPF controls from param metadata.
- FlightStickConfigControl "X-Plane FFB" section replaced with "FFB Parameters" showing graph params.
- System tab now displays "System Parameters" section for params with `group = "System"`.
- Function params filtered by group (`FlightStickPitch`, `FlightStickRoll`, etc.) and shown in respective tabs.
- Param changes update vehicle profile in-memory immediately; explicit save required for persistence.
- ActiveGraphChanged event notifies UI to refresh when active graph changes.
- Test infrastructure consolidated: GraphTest (runtime + editor tests), KinematicsTests (physics validation).
- GraphTest runs 23 tests: 15 runtime (evaluator/validation/includes) + 8 editor (serialization/params).
- Parameter resolution tests validate three-tier resolution logic (include default → graph override → vehicle override).
- ParamValues serialization tests verify JSON roundtrip of graph-level parameter overrides.
- KinematicsTests validates general kinematics solver (pin/bar validation, collinearity, coefficient calculations).
- FlightPedalsConfigControl replaced X-Plane FFB section with FFB Parameters showing filtered graph params.
- Port name editing without focus loss (UpdateSourceTrigger=LostFocus).
- Port remove button functionality in inspector.
- Multi-port param sync for shared parameters across functions.
- Graph template selector dialog prompts on first vehicle encounter.
- Template registry system filters templates by game ID.
- Default graph templates (plane_default.json, heli_default.json) replicating legacy X-Plane FFB.
- Template selection auto-copies to vehicle-specific path and saves to settings.
- Colored title bars distinguish node types (Input=Blue, Output=Orange, Param=Purple, Const=Gray, Op=Green, Func=Teal, Include=Magenta).
- Double-click Include nodes to navigate to included graph.
- Improved title bar spacing (8px gap between title and content).

## In Progress

- UX polish (orthogonal routing, mini-map).
- Grid padding/centering strategy (grid currently background brush; padding intent documented).

## Open

- Typed units and validation.
- Graph persistence in profiles and migrations.
- Future: Tooltips with mini-curves and live cursors on param nodes.
