# Include Preview Value Display - Debug Plan

**Status**: RESOLVED
**Created**: 2026-01-26
**Last Updated**: 2026-01-26

## Problem Statement

**Symptom**: Runtime evaluation shows correct values, but preview evaluation doesn't show valid live values on Include node outputs in the top-level (active) graph.

**Root causes identified**:

1. Include ports not populated before conversion
2. Parameters not collected from included sub-graphs

## Fixes Applied

### Fix 1: PopulateIncludePorts Before Conversion

**File**: `GraphEditor/GraphPreviewEvaluator.cs`

**Issue**: Preview path didn't call `PopulateIncludePorts()` before conversion, so Include nodes had empty Ports lists and no OutputMap was built.

**Change**:

```csharp
// Populate Include ports so OutputMap is built correctly (matches runtime path)
if (!string.IsNullOrEmpty(_baseDirectory))
{
    GraphSerializer.PopulateIncludePorts(graph, _baseDirectory);
}

var runtime = GraphRuntimeConverter.Convert(graph);
```

### Fix 2: Pass baseDirectory to Evaluator

**File**: `GraphEditor/GraphPreviewEvaluator.cs`

**Issue**: Evaluator wasn't receiving baseDirectory, causing path resolution issues.

**Change**:

```csharp
// Before:
var evaluator = new GraphCompiledEvaluator(runtime, _resolver);

// After:
var evaluator = new GraphCompiledEvaluator(runtime, _resolver, null, _baseDirectory);
```

Also added `SetBaseDirectory()` method wired in `UpdatePreviewResolver()`.

### Fix 3: Collect Parameters from Included Sub-graphs

**File**: `GraphEditor/GraphEditorControl.xaml.cs`

**Issue**: Debug log showed `parameters:` was EMPTY in preview but had 23 entries at runtime. The preview only collected params from Param nodes in the current graph, but top-level graphs have no Param nodes - all params are defined in included sub-graphs.

**Change**: Added `CollectIncludeParams()` method that recursively loads included sub-graphs and extracts their parameter definitions:

```csharp
private void CollectIncludeParams(GraphDefinition graph, string baseDir, HashSet<string> paramNames)
{
    // For each Include node in the graph:
    //   1. Resolve the include path
    //   2. Load the included graph JSON
    //   3. Add params from includedGraph.Params to paramNames
    //   4. Recurse into nested includes
}
```

Called from `SyncPreviewEntries()` after collecting local Param nodes.

### Fix 4: Sync Live Parameter Values from Plugin

**Files**: `GraphEditor/GraphEditorControl.xaml.cs`, `GraphEditor/GraphEditorWindow.xaml.cs`

**Issue**: Preview showed **default values** (1000, 400, etc.) while runtime had **live configured values** (1800, 525, etc.). The `SyncParamsFromPlugin()` method only iterated over `graph.Params.Keys` which is empty for top-level graphs (all params are in includes).

**Changes**:

Added `GetCollectedParamNames()` method to `GraphEditorControl`:

```csharp
public IEnumerable<string> GetCollectedParamNames()
{
    return _previewParamLookup.Keys;
}
```

Modified `SyncParamsFromPlugin()` in `GraphEditorWindow` to use collected params:

```csharp
private void SyncParamsFromPlugin()
{
    // Get all collected param names (includes params from Include nodes)
    var paramNames = activeTab.EditorControl.GetCollectedParamNames();
    foreach (var paramName in paramNames)
    {
        double pluginValue = plugin.GetGraphParamValue(paramName);
        activeTab.EditorControl.UpdateParamValue(paramName, pluginValue);
    }
}
```

This syncs runtime parameter values (from three-tier resolution) to the preview when loading a graph.

## Debug Instrumentation

### Logging Control

Debug logging is controlled via checkbox in the graph editor inspector panel:

- **Checkbox**: "Debug Logging" in inspector panel
- **Tooltip**: Shows log file path, click to open in Explorer
- **Default**: Disabled (no performance overhead)

All `GraphDebugLogger` calls are wrapped with `if (GraphDebugLogger.Enabled)` checks to avoid string allocation overhead when logging is disabled.

### Log Location

```text
%LocalAppData%\DiyFfb\graph_debug.log
```

### Key Log Sections to Check

1. **GraphPreviewEvaluator.Evaluate**:
   - `parameters:` - Should now have entries (was empty before Fix 3)
   - `inputs:` - Should have input values

2. **EvalInclude**:
   - `OutputMap:` - Should show port mappings
   - `parent parameters:` - Should have all params from parent
   - `Param 'X': from parent = Y` - Shows parameter resolution
   - `subParams:` - Parameters passed to sub-graph
   - `outputs (from sub-graph):` - Sub-graph output values

### Sample Expected Log (After Fixes)

```text
============================================================
  GraphPreviewEvaluator.Evaluate
============================================================
  _baseDirectory: C:\Program Files (x86)\SimHub\graphs\vehicles
  _resolver: set
  inputs:
    [XPlane.MainRotor.Torque] = 1497.6478
    ...
  parameters:
    [Aircraft.Rotor.TorqueNom] = 1800.0000
    [Aircraft.Rotor.SpeedNom] = 525.0000
    [Cyclic.SpringGain] = 1.3000
    ...
```

## Architecture Overview

### Two Evaluation Paths

1. **Runtime Path** (DiyFfbPlugin.cs):
   - `CollectAllGraphParams()` recursively collects params from all includes
   - `BuildGraphParams()` resolves values using three-tier resolution
   - `PopulateIncludePorts()` called before conversion
   - Parameters passed to evaluator each frame

2. **Preview Path** (GraphEditorControl + GraphPreviewEvaluator):
   - `SyncPreviewEntries()` collects params for preview UI
   - `CollectIncludeParams()` recursively loads include params (NEW)
   - `RefreshPreview()` builds inputs/params dictionaries
   - `GraphPreviewEvaluator.Evaluate()` runs the graph

### Parameter Flow

```text
Top-level graph
  └── Include node (heli_collective.json)
        └── Param nodes: FlightStickCollective.DamperGain, etc.
  └── Include node (heli_cyclic_pitch.json)
        └── Include node (common/heli_scale.json)
              └── Param nodes: Aircraft.Rotor.TorqueNom, etc.
        └── Param nodes: Cyclic.SpringGain, etc.
```

For preview to work:

1. `CollectIncludeParams()` loads each include recursively
2. Extracts param names and defaults from `includedGraph.Params`
3. Adds them to `_previewParamEntries`
4. `RefreshPreview()` builds `parameters` dict from entries
5. `Evaluate()` passes params to evaluator
6. `EvalInclude()` matches param names to sub-graph Param nodes

## Testing Checklist

After restarting SimHub:

- [ ] Open graph editor with a top-level heli graph
- [ ] Check preview params panel - should show params from includes
- [ ] Check debug log - `parameters:` should have entries
- [ ] Include node outputs should show live values
- [ ] Values should match runtime evaluation

## Files Modified

| File | Changes |
|------|---------|
| `GraphEditor/GraphPreviewEvaluator.cs` | PopulateIncludePorts, baseDirectory, debug logging |
| `GraphEditor/GraphEditorControl.xaml.cs` | CollectIncludeParams, debug logging enable |
| `GraphTest/GraphCompiledEvaluator.cs` | Enhanced parameter resolution logging |
| `GraphTest/GraphDebugLogger.cs` | Debug logging utility (new) |

## Remaining Issues

If values still don't appear after testing:

1. **Check log for `parameters:` count** - Should be > 0
2. **Check if includes are loading** - Look for `CollectIncludeParams` to find include files
3. **Check param name matching** - Sub-graph Param node names must match exactly
4. **Check ParamValues (Tier 2)** - Graph-level overrides in `graph.ParamValues`

## Cleanup After Resolution

**Completed:**

1. ✓ Debug logging disabled by default (checkbox controls it)
2. ✓ Status updated to RESOLVED
3. ✓ All logging calls wrapped with `if (GraphDebugLogger.Enabled)` to avoid string allocation overhead
4. ✓ UI checkbox added to inspector panel with tooltip showing log path
