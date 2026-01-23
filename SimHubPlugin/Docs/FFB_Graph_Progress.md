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

## In Progress
- UX polish (orthogonal routing, mini-map).
- Grid padding/centering strategy (grid currently background brush; padding intent documented).
- (none)

## Open
- Param UI schema controls in function UI.
- Typed units and validation.
- Graph persistence in profiles and migrations.
