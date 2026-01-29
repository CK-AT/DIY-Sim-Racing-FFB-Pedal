# SimHub Plugin Review Report: Duplication & Evaluator Performance

Date: 2026-01-28
Scope: SimHubPlugin graph evaluation/runtime + editor preview paths

## Summary
- The compiled evaluator avoids dictionary lookups per node, but there are still per-evaluation linear scans and repeated mapping work that can be cached.
- Interpreter and compiled evaluators duplicate Op/Func logic and topological sorting, which is a drift risk.

## Findings (ordered by impact)
- [x] Output name resolution is O(outputs * nodes) during every evaluation.
  - GraphCompiledEvaluator builds output indices, but later resolves output node names via a linear scan.
  - File: `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

- [x] Include I/O mapping scans the subgraph repeatedly during each include evaluation.
  - `BuildShortToFullNameMap` iterates subgraph nodes and short names each call.
  - File: `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

- [x] Include input resolution does repeated dictionary lookups by id.
  - `ResolveById` is called for each include input mapping; this can be precompiled into indices like regular args.
  - File: `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

- [x] Preview evaluation always rebuilds runtime conversion and a compiled evaluator.
  - If `RefreshPreview` is called frequently (UI interactions), this causes avoidable allocation churn.
  - File: `SimHubPlugin/GraphEditor/GraphPreviewEvaluator.cs`

## Duplication Hotspots
- [ ] Op and Func evaluation logic duplicated in interpreter and compiled evaluator.
  - Files: `SimHubPlugin/GraphTest/GraphEvaluator.cs`, `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`
- [ ] Topological sort and include-output dependency mapping duplicated in both evaluators.
  - Files: `SimHubPlugin/GraphTest/GraphEvaluator.cs`, `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

## Mitigations
- [x] Cached output name list to remove per-eval linear scans.
- [x] Cached include I/O short-name mapping and include input bindings.
- [x] Cached preview evaluator/runtime conversion to avoid per-refresh rebuilds.
- [x] Added perf harness and observed ~8.4x improvement for 5,000 compiled-evaluator iterations (about 745.85 ms -> 88.57 ms).
  - Run: set `FFB_PERF_ONLY=1` and execute `SimHubPlugin/GraphTest/bin/Debug/net48/GraphTest.exe`.

## Preview Evaluation Call Sites
- `GraphPreviewEvaluator.Evaluate` is called from `GraphEditorControl.RefreshPreview`.
- `RefreshPreview` is invoked by UI actions (graph changes, preview value changes, context selection, etc.), not on a background timer in this file.
- File: `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
