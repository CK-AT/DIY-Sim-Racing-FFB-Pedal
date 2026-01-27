# Parameter Control Layout Restructure Plan

## Goal
Restructure parameter control layout so:
- Function parameters also appear on the Vehicle tab.
- Parameter order within each group follows graph layout (top-to-bottom), not alphabetical.
- Group ordering remains as-is (alphabetical, with <unknown> last).

## Scope
- Vehicle tab parameter filtering and ordering.
- Shared parameter-order source derived from graph node layout.
- Function tab parameter ordering.

## Current Behavior
- Vehicle tab:
  - Filters out System and function-group params (FlightStick*, FlightPedals*, Automotive, Shifter).
  - Groups by `GraphParam.Ui.Group` and sorts groups alphabetically (`<unknown>` last).
  - Sorts params within groups by label/name alphabetically.
- System tab:
  - Filters to `Group == "System"` and sorts by label/name alphabetically.

## Proposed Behavior
- Vehicle tab:
  - Include function-group params (only exclude System group).
  - Group ordering stays alphabetical with `<unknown>` last.
  - Param ordering inside each group follows graph layout order of Param nodes (top-to-bottom, then left-to-right, then id), based on the active graph.
  - Include graph params are inserted at the Include node’s position in that same ordering, and are internally ordered by the included graph’s layout (recursive).
  - Params not represented by a Param node (e.g., defined only in `graph.Params`) append after ordered items, sorted by label/name for stability.
- Function tabs:
  - Param ordering follows the same graph layout order (including include ordering), not alphabetical.

## Ordering Rules
1. Build a global ordered list by walking layout-ordered nodes:
   - Sort nodes by `Y`, then `X`, then `Id` (stable layout order).
   - When a Param node is encountered, include its output ports in the order they appear in `node.Ports`.
   - When an Include node is encountered, resolve the included graph and append its ordered params by recursively applying these same rules to the included graph.
   - Build param name as `SignalGroup + "." + SignalSuffix` when both are present; otherwise use `port.Name` (matches editor/runtime param naming).
   - Use the runtime include resolver for include path resolution to match evaluation behavior.
2. For each UI group:
   - Take params present in the ordered list in that order.
   - Append remaining group params not in the ordered list, sorted by `Ui.Label` then `Name`.

## Implementation Steps
1. Add a shared ordering helper on the plugin side:
   - `GetActiveGraphParamOrder()` returns ordered param names using active graph node layout and include recursion.
2. Update Vehicle tab filtering:
   - Remove function-group exclusions; keep System exclusion.
3. Update Vehicle tab ordering:
   - Build grouped params using `GetActiveGraphParamOrder()` for order within each group.
   - Append remaining params per group in label/name order.
4. Update function tab ordering:
   - Use `GetActiveGraphParamOrder()` within each function group instead of alphabetical.
5. (Optional) Evaluate System tab ordering consistency:
   - Decide whether to adopt the same ordering for System group for consistency with graph layout.
6. Update docs/tests as needed:
   - Note ordering rules in a relevant design doc if graph UI behavior is documented.
   - Add/adjust tests if ordering becomes part of expected UI behavior.

## Async/Ordering Considerations
- No new async flows. Ordering uses the active graph snapshot at refresh time, and existing UI refresh guards (`isUpdatingVehicleParams`) prevent feedback loops.

## Files to Touch
- `SimHubPlugin/DiyFfbPlugin.cs` (expose ordered param list)
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs` (Vehicle tab filtering/ordering)
- (If adopted) `SimHubPlugin/DiyFfbPluginUI.xaml.cs` (System tab ordering)
- Docs/tests if required

## Validation
- Load a graph with mixed System, Vehicle, and function-group params.
- Verify Vehicle tab shows function params and ordering matches the graph top-to-bottom layout within each group.
- Verify groups remain alphabetical with `<unknown>` last.
- Confirm params without Param nodes appear after ordered ones.
