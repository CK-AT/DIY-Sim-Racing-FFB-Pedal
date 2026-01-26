# Plan: Fix "Nested includes via resolver" Test

**Status**: COMPLETED
**Created**: 2026-01-26
**Completed**: 2026-01-26
**Related**: `conversation_log.md` entry "Fix EditorFormatConverter Delegate Signature Mismatch"

## Problem Statement

The test `Nested includes via resolver` fails because `ConvertEditorJson` doesn't yet USE the `resolvedFilePath` parameter. The signature is correct, but the nested path resolution logic was removed in a previous revert.

When a parent graph includes a sub-graph that itself contains Include nodes, the nested includes resolve paths relative to the **parent's directory** instead of the **sub-graph's directory**.

### Example

- Parent: `graphs/templates/my_graph.json`
- Sub-graph: `graphs/_embedded/heli_cyclic_pitch.json`
- Nested include in sub-graph: `common/heli_scale.json`
- Expected resolution: `graphs/_embedded/common/heli_scale.json`
- Actual resolution: `graphs/templates/common/heli_scale.json` (wrong!)

## Current State

| Component | Status |
| --------- | ------ |
| `GraphIncludeResolver.EditorFormatConverter` | `Func<string, string, GraphDefinition>` (2-arg) ✅ |
| `GraphIncludeResolver.TryLoadEditorFormat` | Passes both args to delegate ✅ |
| `GraphRuntimeConverter.ConvertEditorJson` | Has 2-arg signature but ignores `resolvedFilePath` ❌ |
| Test `Nested includes via resolver` | FAILING |

## Implementation Plan

### Step 1: Understand the Failing Test

Read the test implementation to understand expected behavior:

```bash
# Search for the test
grep -n "Nested includes via resolver" SimHubPlugin/GraphTest/GraphTestRunner.cs
```

The test is around line 1967 in `GraphTestRunner.cs`.

### Step 2: Identify the Fix Location

The fix needs to happen in `GraphRuntimeConverter.ConvertEditorJson`:

```csharp
// Current (line 28 in GraphRuntimeConverter.cs):
private static DiyFfb.GraphTest.GraphDefinition ConvertEditorJson(string json, string resolvedFilePath)
{
    // resolvedFilePath is currently UNUSED
    ...
}
```

### Step 3: Implement the Fix

**Option A: Use resolvedFilePath for PopulateIncludePorts**

```csharp
private static DiyFfb.GraphTest.GraphDefinition ConvertEditorJson(string json, string resolvedFilePath)
{
    // ... existing detection code ...

    var editorGraph = GraphSerializer.Deserialize(json, out var validation);
    if (editorGraph != null && validation != null && validation.IsValid)
    {
        // Use the sub-graph's directory for resolving nested includes
        string subGraphDirectory = Path.GetDirectoryName(resolvedFilePath);
        if (!string.IsNullOrEmpty(subGraphDirectory))
        {
            // Populate include ports relative to sub-graph's directory
            PopulateIncludePorts(editorGraph, subGraphDirectory);
        }
        return Convert(editorGraph);
    }
    return null;
}
```

**WARNING**: This approach caused runtime regression before (absurd FFB values like damper=2349). The issue was double-resolution of paths.

**Option B: Store base directory in GraphDefinition**

Add a `BaseDirectory` property to `GraphDefinition` that the evaluator uses when resolving nested includes. This avoids modifying paths during conversion.

1. Add property to `GraphDefinition`:

   ```csharp
   public string BaseDirectory { get; set; } = "";
   ```

2. Set it in `ConvertEditorJson`:

   ```csharp
   var runtime = Convert(editorGraph);
   runtime.BaseDirectory = Path.GetDirectoryName(resolvedFilePath) ?? "";
   return runtime;
   ```

3. Use it in `GraphCompiledEvaluator.EvalInclude` when resolving nested paths.

### Step 4: Build and Test

```bash
# Build GraphTest
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\GraphTest.csproj" \
  /p:Configuration=Debug /v:minimal /nologo

# Run tests
"d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\bin\Debug\net48\GraphTest.exe"
```

Expected: 50/50 tests pass

### Step 5: Verify No Runtime Regression

```bash
# Build main plugin
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Debug /v:minimal /nologo
```

Then in SimHub:

1. Load a graph with nested includes
2. Verify FFB values are reasonable (e.g., damper ~1.5, not 2349)
3. Test with actual hardware if available

### Step 6: Run All Tests

```bash
# Build and run KinematicsTests
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\KinematicsTests\KinematicsTests.csproj" \
  /p:Configuration=Debug /v:minimal /nologo

"d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\KinematicsTests\bin\Debug\KinematicsTests.exe"
```

Expected: 18/18 tests pass

## Key Files

| File | Purpose |
| ---- | ------- |
| `GraphTest/GraphTestRunner.cs` | Failing test implementation (~line 1967) |
| `GraphEditor/GraphRuntimeConverter.cs` | Where `resolvedFilePath` needs to be used |
| `GraphTest/GraphCompiledEvaluator.cs` | Where include evaluation happens |
| `GraphTest/GraphIncludeResolver.cs` | Delegate definition and path resolution |
| `GraphEditor/GraphEditorControl.xaml.cs` | `PopulateIncludePorts` method |

## Warnings

1. **PopulateIncludePorts** - May modify include paths in ways that cause double-resolution when the evaluator also resolves paths
2. **ResolveToAbsolutePath in evaluator** - The evaluator may already resolve paths, so pre-resolving in the converter causes double-resolution
3. **Previous regression** - This exact fix was attempted before and reverted due to absurd FFB values (damper=2349 instead of ~1.5)

## Success Criteria

- [x] Test `Nested includes via resolver` passes
- [x] All 50 GraphTest tests pass
- [x] All 18 KinematicsTests pass
- [x] Runtime FFB values are correct (no regression)
- [x] Update `conversation_log.md` with fix details
- [ ] Update `FFB_Graph_Progress.md` if needed
