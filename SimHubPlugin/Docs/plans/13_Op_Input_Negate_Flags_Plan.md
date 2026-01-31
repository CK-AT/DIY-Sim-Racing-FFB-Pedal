# Op Input Negate Flags Plan

## Status (2026-01-28)
- Implemented

## Goal
Allow per-input negation for Op nodes where it is useful (add/mul), so users can flip individual inputs without extra Neg nodes.

## Scope
- Graph editor model + serialization (editor JSON + clipboard).
- Runtime conversion + evaluators (GraphTest + compiled evaluator).
- Validation (GraphLoader).
- Inspector UI for Op nodes.
- Docs and tests.

## Current Behavior
- Op inputs are fixed/variadic by op type, but there is no per-input negate state.
- Editor serialization only persists port name/kind/signal suffix.
- Runtime Op nodes carry `Args` only (no per-arg metadata).

## Proposed Behavior
- Add/mul inputs can be marked as negated via a toggle in the Op inspector.
- Negation is stored per input port and preserved through save/load and clipboard operations.
- Runtime evaluation applies the negation to each input before folding.
- Output label for variadic ops reflects negation (e.g., `a+b-c`, `a*b*-c`, `min(a,b,c)` stays unsigned).

## Data Model + Serialization
1) **Editor model**
   - Add `bool Negate` to `GraphPort` (only meaningful for Op input ports).
   - Default false; ignored for non-Op or output ports.
2) **Editor JSON**
   - Add `Negate` to `GraphPortDto` with `ShouldSerializeNegate()` (only when true).
   - Ensure `GraphClipboardSerializer` carries the flag.
3) **Compatibility**
   - Old graphs load with `Negate=false`.
   - When op changes away from add/mul, clear any existing negate flags on inputs.

## Runtime Conversion + Evaluation
1) **Runtime model**
   - Extend `GraphTest.GraphNode` to carry per-arg negate flags, e.g. `List<bool> ArgNegate`.
2) **Conversion**
   - `GraphRuntimeConverter.Convert()` should append `ArgNegate` entries alongside `Args` for Op nodes.
3) **Evaluation**
   - `GraphEvaluator.EvalOp()` and `GraphCompiledEvaluator.EvalOp()` apply negation per arg before folding.
4) **Validation**
   - `GraphLoader.GraphValidator` rejects negate flags for non-add/mul ops.
   - Keep arg-count rules intact (variadic still needs >=2).

## Editor UI
1) **Op inspector**
   - Show a per-input negate toggle (e.g., “−”) next to each input port.
   - Toggle only visible for Op nodes where negate is supported (add/mul).
2) **Port list**
   - For Op nodes: port names remain read-only; negate toggles are editable.
3) **Output label**
   - Update output label generation to incorporate negation for add/mul:
     - Add: `a+b-c` (for negated `c`).
     - Mul: `a*b*-c` (explicit minus).
     - Min/Max: no negate support; leave as `min(a,b,c)` / `max(...)`.

## Tests
1) **Runtime**
   - New evaluation test: add/mul with mixed negated inputs.
2) **Editor serialization**
   - JSON roundtrip preserves `Negate` on Op input ports.
3) **Conversion**
   - Conversion test ensures `ArgNegate` matches editor ports.
4) **Validation**
   - Reject negate on non-add/mul ops.

## Docs
- Update `SimHubPlugin/Docs/FFB_Graph_Design.md` and `SimHubPlugin/Docs/FFB_Graph_Progress.md`.
- Update `SimHubPlugin/Docs/Graph_Node_Types_Design.md`.
- Update `SimHubPlugin/Docs/FFB_Design_Current.md` (FFB behavior change).

## Async / Ordering Considerations
- No new async or out-of-order flows. Negate flags are read from the graph snapshot at evaluation time.

## Risks / Notes
- Ensure per-port negate doesn’t conflict with signal binding (only applies to Op input ports).
- Keep runtime/editor evaluation parity by updating both evaluators and converter together.
