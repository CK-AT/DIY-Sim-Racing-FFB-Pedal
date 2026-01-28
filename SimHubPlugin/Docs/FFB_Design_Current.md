# FFB Design Current

Status: active design summary for the graph-driven FFB pipeline.

## Overview
- The FFB pipeline is defined by the node graph system described in `FFB_Graph_Design.md`.
- Runtime evaluation uses the shared graph evaluator and compiled evaluator for parity.
- Op nodes support per-input negate toggles for add/mul, and output labels reflect negated inputs (for example, "a+b-c").

## References
- `FFB_Graph_Design.md`
- `Graph_Node_Types_Design.md`
