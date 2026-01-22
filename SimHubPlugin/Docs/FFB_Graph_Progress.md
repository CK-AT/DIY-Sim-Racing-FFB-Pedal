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
- Runtime graph evaluation is wired (inputs/params + cached evaluator) without output mapping yet.
- Output port labels show live preview values in the editor.
- Reverted initial centering/zoom changes after pan/zoom regressions.
- Restored grid to canvas background and removed auto-centering while stabilizing pan/zoom.

## Open
- Restore grid padding left/top without breaking pan/zoom.
- Input/output ports can pick known signals via selector in the inspector.
- Param nodes expose per-port default/min/max values in the port list.
- Runtime mapping now uses per-port IDs for input/param/output nodes.
- Sample graphs now bind to real input/output signal names.
- Added a signal catalog document for input/output keys.
- Live inputs toggle feeds the preview from latest telemetry snapshots.
- Graph canvas now clips to bounds and extends its grid beyond the initial view.
- Right mouse button pans; selection rectangle uses left drag without modifiers.
- Multi-select node drag now moves all selected nodes.
- Signal selector now shows the current port name even if it is not in the list.
- Func ports now match function signatures (e.g. assist_loss uses a single input).
- Signal selector binds selected item/text to show the current signal reliably.
- Replaced signal ComboBox with a hierarchical popup picker.
- Editor keyboard shortcuts no longer intercept text input in fields.
- Func port changes now refresh the canvas after inspector edits.
- Grid extents padded by ~300 px around the graph bounds for panning.
- Initial view centering uses the visible graph column size (not full canvas).
- Grid background now moves with the graph transform for proper left/top space.
- Initial view recenters the graph within the viewport while honoring padding limits.

## In Progress
- UX polish (orthogonal routing, mini-map).

## Open
- Runtime integration (graph selection per function/axis).
- Compiled graph evaluation pipeline.
- Param UI schema controls in function UI.
- Typed units and validation.
- Graph persistence in profiles and migrations.
