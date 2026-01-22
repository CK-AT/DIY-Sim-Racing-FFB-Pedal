# FFB Graph Grid + Centering Intent

Purpose: capture the intended grid, padding, and initial view behavior so we can implement it in isolation.

## Goals
- Initial view shows the entire graph.
- Grid extends beyond the visible window in all directions (so panning reveals empty grid space).
- Grid padding around the graph is visible (target ~200px beyond graph bounds).
- No auto-zoom-in: keep scale at 1.0 if the graph already fits; only zoom out if needed.
- After the initial view is set, pan/zoom should be fully user-controlled (no recentering).
- Grid follows the graph transform (same scale/translate), not a fixed screen overlay.
- Performance stays stable (no massive background surface).

## Definitions
- Graph bounds: min/max X/Y of all nodes, including their rendered widths/heights.
- Padding: extra space beyond graph bounds; defaults to 200px.
- View bounds: size of the graph viewport (graph column dimensions).

## Desired Initial View Algorithm (Conceptual)
1. Measure graph bounds (include node size).
2. Expand bounds by padding (left/right/top/bottom).
3. Set grid extents to the padded bounds (not the entire canvas).
4. Compute scale:
   - scale = min(viewWidth / paddedWidth, viewHeight / paddedHeight, 1.0)
5. Compute translate so padded bounds are centered in the view.
6. Apply scale/translate once (only if the user has not panned/zoomed).

## Behavioral Notes
- If the graph is smaller than the view, the scale remains 1.0 and the view centers the padded bounds.
- If the graph is larger than the view, scale down until it fits (still centered).
- Grid should always be visible beyond the graph; the graph should never start at the top-left of the grid.

## Open Questions (Before Implementation)
- Exact padding value (default 200px) and whether it should be configurable.
- Whether zoom-to-fit should be exposed as a user command (separate from initial view).
