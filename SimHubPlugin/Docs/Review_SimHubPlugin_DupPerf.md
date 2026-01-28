# SimHub Plugin Review Report: Duplication & Evaluator Performance

Date: 2026-01-28
Scope: SimHubPlugin graph evaluation/runtime + editor preview paths

## Summary
- The compiled evaluator avoids dictionary lookups per node, but there are still per-evaluation linear scans and repeated mapping work that can be cached.
- Interpreter and compiled evaluators duplicate Op/Func logic and topological sorting, which is a drift risk.

## Findings (ordered by impact)
1) Output name resolution is O(outputs * nodes) during every evaluation.
   - GraphCompiledEvaluator builds output indices, but later resolves output node names via a linear scan.
   - File: `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

2) Include I/O mapping scans the subgraph repeatedly during each include evaluation.
   - `BuildShortToFullNameMap` iterates subgraph nodes and short names each call.
   - File: `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

3) Include input resolution does repeated dictionary lookups by id.
   - `ResolveById` is called for each include input mapping; this can be precompiled into indices like regular args.
   - File: `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

4) Preview evaluation always rebuilds runtime conversion and a compiled evaluator.
   - If `RefreshPreview` is called frequently (UI interactions), this causes avoidable allocation churn.
   - File: `SimHubPlugin/GraphEditor/GraphPreviewEvaluator.cs`

## Duplication Hotspots
- Op and Func evaluation logic duplicated in interpreter and compiled evaluator.
  - Files: `SimHubPlugin/GraphTest/GraphEvaluator.cs`, `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`
- Topological sort and include-output dependency mapping duplicated in both evaluators.
  - Files: `SimHubPlugin/GraphTest/GraphEvaluator.cs`, `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

## Preview Evaluation Call Sites
- `GraphPreviewEvaluator.Evaluate` is called from `GraphEditorControl.RefreshPreview`.
- `RefreshPreview` is invoked by UI actions (graph changes, preview value changes, context selection, etc.), not on a background timer in this file.
- File: `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
