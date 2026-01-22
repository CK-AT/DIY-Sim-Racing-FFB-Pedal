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

## In Progress
- UX polish (orthogonal routing, mini-map).

## Open
- Runtime integration (graph selection per function/axis).
- Compiled graph evaluation pipeline.
- Param UI schema controls in function UI.
- Typed units and validation.
- Graph persistence in profiles and migrations.
