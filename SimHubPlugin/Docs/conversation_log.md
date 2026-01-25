# Conversation Log

## 2026-01-25: Add Stall Buffeting to FFB Graph System

### Summary

Added a `buffet` Func node and `BuffetAmplitude` output to restore stall buffeting capability that was lost when moving to graph-based FFB.

### Implementation

- **New Func node**: `buffet` with inputs: `alpha`, `start`, `full`, `gain`, `qhat_eff`
- **New outputs**: `BuffetAmplitude` for FlightStickPitch, FlightStickRoll, FlightPedals
- **UI**: Added "Buffet:" display in FFB Outputs panels
- Logic matches existing `XPlaneFfbMath.ComputeBuffet()` calculation

### Files Changed

- `GraphEditor/GraphEditorControl.xaml.cs`: Added buffet to _funcChoices and GetFuncInputNames()
- `GraphTest/GraphEvaluator.cs`: Added buffet case in EvalFunc()
- `GraphTest/GraphCompiledEvaluator.cs`: Added buffet case in EvalFunc()
- `GraphSignalCatalogData.cs`: Added BuffetAmplitude outputs
- `DiyFfbPlugin.cs`: Wired BuffetAmplitude through ApplyGraphOutputs()
- `FlightPedalsConfigControl.xaml/.cs`: Added Buffet to FFB Outputs panel
- `FlightStickConfigControl.xaml/.cs`: Added Buffet to FFB Outputs panel

### Commit Highlights

- Add buffet Func node (alpha, start, full, gain, qhat_eff inputs)
- Add BuffetAmplitude output for flight contexts
- Wire graph BuffetAmplitude to FlightFfbAction protocol
- Add Buffet display in FFB Outputs UI panels

---

## 2026-01-25: Fix Include Context Cache Cleared by Sub-Evaluators

### Summary

Fixed the context dropdown always showing "Found 0 contexts" even though contexts were being added to the cache. The cache was being cleared during Include node evaluation by sub-evaluators.

### Root Cause

`GraphCompiledEvaluator.EvaluateWithTrace()` called `_contextCache?.Clear()` at the start of each evaluation. When evaluating an Include node, the code:

1. Added context to the shared cache
2. Called `evaluator.Evaluate(subInputs, params)` on the sub-graph evaluator
3. Sub-evaluator's `Evaluate()` → `EvaluateWithTrace()` → `Clear()` wiped the just-added context

Since all evaluators share the same cache instance (by design, for nested includes), the sub-evaluator cleared what the parent just added.

### Fix

- Removed `_contextCache?.Clear()` from `EvaluateWithTrace()`
- Added `activeIncludeContextCache?.Clear()` in `DiyFfbPlugin.EvaluateGraph()` before top-level evaluation
- Added `_lastContextIds` tracking in `RefreshContextDropdown()` to only rebuild when contexts change
- Normalized `_filePath` in `RefreshPreview()` to match cache key format

### Files Changed

- `GraphTest/GraphCompiledEvaluator.cs`: Removed cache clearing from EvaluateWithTrace
- `GraphTest/IncludeContextCache.cs`: Removed debug logging
- `GraphEditor/GraphEditorControl.xaml.cs`: Smart dropdown refresh, path normalization in RefreshPreview
- `GraphEditor/GraphEditorWindow.xaml.cs`: Path normalization in UpdateTabContextLabel
- `DiyFfbPlugin.cs`: Added cache clearing before top-level evaluation
- `Docs/FFB_Graph_Progress.md`: Added fix entries

### Commit Highlights

- Fix context cache emptied by sub-evaluators during Include evaluation
- Fix dropdown selection with smart refresh (only rebuild when contexts change)
- Fix context inputs applied in preview (path normalization)
- Fix tab label context suffix (path normalization)

---

## 2026-01-25: Implement Include Context Preview

### Summary

Implemented Include Context Preview feature: sub-graphs can now preview with real parent context inputs. When viewing an included graph, a dropdown shows available Include call sites from the active graph evaluation, allowing preview to use actual parent-provided inputs instead of manual test values.

### Implementation

1. **Data Classes**: `IncludeCallContext` captures inputs/params passed to an Include node; `IncludeContextCache` stores contexts keyed by resolved path (case-insensitive).

2. **Evaluator Integration**: `GraphCompiledEvaluator` accepts optional cache and populates it during `EvalInclude()`. Cache cleared at start of each evaluation cycle.

3. **Plugin Wiring**: `DiyFfbPlugin` creates cache, passes to evaluator, exposes via `ActiveIncludeContextCache` property.

4. **Editor UI**: Context dropdown in inspector (hidden when no contexts available). When a context is selected, preview uses its inputs/params instead of manual entries.

5. **Tab Labels**: `ContextSuffix` property on `GraphEditorTab` shows " (via IncludeTitle)" when previewing with context.

### Files Changed

- `GraphTest/IncludeCallContext.cs`: New data class
- `GraphTest/IncludeContextCache.cs`: New cache class
- `GraphTest/GraphCompiledEvaluator.cs`: Added cache parameter and population logic
- `GraphTest/GraphTestRunner.cs`: Added 6 tests (4 cache + 2 evaluator)
- `DiyFfbPlugin.cs`: Create/wire cache, expose property
- `DiyFfbPlugin.csproj`: Added new files
- `GraphEditor/GraphEditorControl.xaml`: Context dropdown UI
- `GraphEditor/GraphEditorControl.xaml.cs`: Context selection logic, RefreshPreview updates
- `GraphEditor/GraphEditorTab.cs`: ContextSuffix property
- `GraphEditor/GraphEditorWindow.xaml.cs`: Wire context provider, update tab labels
- `Docs/FFB_Graph_Progress.md`: Updated progress

### Commit Highlights

- Add Include Context Preview for sub-graph previews
- Context dropdown shows available Include call sites
- Tab labels reflect active context

---

## 2026-01-25: Fix Pending Graph Params Not Saved Without Profile

### Summary

Fixed pending graph params not being saved when no aircraft profile exists yet for the current vehicle.

### Root Cause

In `SetGraphParamValue()`, `MarkProfileDirty()` (which triggers `SavePendingGraphParams()`) was only called if `GetCurrentAircraftProfile()` returned a non-null profile. For aircraft without a stored profile, changes were applied to `activeVehicleGraph.ParamValues` (Tier 2) but never persisted to the pending params file.

Additionally, `SavePendingGraphParams()` only read params from the profile's `GraphParamValues`, returning early if the profile was null.

### Fix

1. `SetGraphParamValue()` now always calls `MarkProfileDirty()` regardless of profile existence
2. `SavePendingGraphParams()` falls back to `activeVehicleGraph.ParamValues` (Tier 2) when no profile exists

### Files Changed

- `DiyFfbPlugin.cs`: Moved `MarkProfileDirty()` call outside profile check; added Tier 2 fallback in `SavePendingGraphParams()`

### Commit Highlights

- Fix pending params not saved without profile
- SavePendingGraphParams uses Tier 2 fallback

---

## 2026-01-25: Fix Preview/Runtime Resolver Divergence + Plan Include Context Preview

### Summary

Fixed include graphs working at runtime but failing in preview due to resolver configuration divergence. Designed and documented Include Context Preview feature for future implementation.

### Root Cause

Preview resolver created via `new GraphIncludeResolver(baseDir)` was missing the `EditorFormatConverter` delegate. Runtime resolver set it explicitly. When preview tried to load editor-format include graphs, the converter delegate was null, causing silent failure.

### Fix

Added `GraphRuntimeConverter.CreateResolver(baseDirectory)` factory method that configures the resolver with editor format support. Both preview and runtime now use this single factory, eliminating configuration divergence.

### Files Changed

- `GraphRuntimeConverter.cs`: Added `CreateResolver()` factory and `ConvertEditorJson()` helper
- `GraphEditorControl.xaml.cs`: Use factory in `UpdatePreviewResolver()`
- `DiyFfbPlugin.cs`: Use factory, removed duplicate `ConvertEditorFormatGraph()` method

### New Documentation

- `Docs/Include_Context_Preview_Plan.md`: Detailed implementation plan for Include Context Preview feature (live debugging of sub-graphs with caller's inputs)

### Commit Highlights

- Unify resolver config via CreateResolver() factory
- Add Include Context Preview plan

---

## 2026-01-25: Fix Include Node Input Evaluation Order

### Summary

Fixed include node inputs receiving zero values due to incorrect topological sort.

### Root Cause

`TopoSort` visits `node.Args` and `node.Src` for dependencies, but Include nodes use `InputMap.Values` for their input dependencies. The Include node's input sources weren't being visited before the Include node was evaluated.

### Fix

Added InputMap.Values traversal to TopoSort for Include nodes, ensuring input source nodes are evaluated before the Include node.

### Commit Highlights

- Fix TopoSort to visit Include InputMap dependencies

---

## 2026-01-25: Fix Editor-Format Include Graph Detection

### Summary

Fixed include graphs not being properly detected as editor-format JSON, causing zero outputs at runtime.

### Root Cause

`GraphIncludeResolver.GetGraph()` checked `graph.Nodes.Count == 0` to trigger editor-format conversion. However, `GraphLoader.LoadFromJson()` partially parsed editor-format JSON, creating 2 nodes with wrong Type (default=Input) and empty Name. The count check passed, so EditorFormatConverter was never called.

### Fix

Changed detection to check for valid nodes: Input/Param/Output nodes must have non-empty Name to be considered valid runtime format. Added `BuildShortToFullNameMap()` in evaluator to handle signal name mapping between short names (SignalSuffix) and full names (SignalGroup.SignalSuffix).

### Files Changed

- `GraphIncludeResolver.cs`: Added `hasValidNodes` check using LINQ Any()
- `GraphCompiledEvaluator.cs`: Added `BuildShortToFullNameMap()` for signal name mapping
- `GraphTestRunner.cs`: Fixed test port Name/SignalSuffix consistency, added JSON round-trip type bridging

### Commit Highlights

- Fix editor-format detection with hasValidNodes check
- Add signal name mapping for include evaluation

---

## 2026-01-25: Fix Include Input/Output Name Mismatch

### Summary

Fixed include graphs receiving zero inputs due to signal name mismatch.

### Root Cause

`ExtractInterface()` used `SignalSuffix` (e.g., "IAS") for non-library graph port names, but `GraphRuntimeConverter.BuildFullSignalName()` produces full names (e.g., "XPlane.Speed.IAS"). The mismatch caused `subInputs["IAS"]` to not match the child Input node's `Name = "XPlane.Speed.IAS"`.

### Fix

Modified `ExtractInterface()` to use `BuildFullSignalName()` for non-library graphs, ensuring interface names match the runtime Input node names exactly.

### Commit Highlights

- Fix include input name mismatch for non-library graphs

---

## 2026-01-25: Fix Include Graphs Not Evaluated at Runtime

### Summary

Fixed include graphs always outputting zero at runtime.

### Root Cause

`GraphIncludeResolver.GetGraph()` used `GraphLoader` which expects runtime-format JSON (with `Type`, `Args`, `Src`), but included graphs are saved by the editor in editor-format JSON (with `Kind`, `Ports`, `Links`). The format mismatch caused deserialization to fail silently.

### Fix

Modified `GraphIncludeResolver.GetGraph()` to detect the JSON format:

- If editor format (contains "links" or "kind"): use `GraphSerializer.Deserialize()` + `GraphRuntimeConverter.Convert()`
- If runtime format: use `GraphLoader.LoadFromJson()` as before

### Commit Highlights

- Detect and convert editor-format includes in resolver

---

## 2026-01-25: Fix Parameter Changes Not Affecting Runtime Evaluation

### Summary

Fixed bug where parameter changes in the graph editor only affected preview but not runtime evaluation.

### Root Cause

`BuildGraphParams()` is called every evaluation frame and rebuilds `graphParams` from scratch using a 3-tier system:

- Tier 1: Include/graph defaults
- Tier 2: `activeVehicleGraph.ParamValues`
- Tier 3: Vehicle profile `GraphParamValues`

`SetGraphParamValue()` updated Tier 3 (profile) but not Tier 2. When no vehicle profile existed (e.g., no aircraft loaded), the Tier 3 save silently failed, and the next frame's `BuildGraphParams()` overwrote the change with defaults.

### Fix

Modified `SetGraphParamValue()` to also update `activeVehicleGraph.ParamValues` (Tier 2), ensuring parameter changes persist across the per-frame `BuildGraphParams()` rebuild regardless of profile availability.

### Commit Highlights

- Fix param changes overwritten by per-frame BuildGraphParams

---

## 2026-01-25: Add FFB Outputs Panel to Function Tabs

### Summary

Added "FFB Outputs" panel below "FFB Parameters" on FlightPedalsConfigControl and FlightStickConfigControl to show live runtime graph output values.

### Changes

1. **DiyFfbPlugin.cs**: Added `GetGraphOutputValue()` and `GetAllGraphOutputs()` public methods to access runtime output values from `lastGraphEvaluation`

2. **FlightPedalsConfigControl.xaml**: Added FFB Outputs panel showing Spring, Damper, Friction, Load, and Trim Offset values

3. **FlightPedalsConfigControl.xaml.cs**: Added `UpdateFfbOutputs()` method called from timer to refresh output values using `FlightPedals.*` signal prefix

4. **FlightStickConfigControl.xaml**: Added same FFB Outputs panel

5. **FlightStickConfigControl.xaml.cs**: Added `UpdateFfbOutputs()` with function ID to prefix mapping (FlightStickPitch/Roll/Collective)

### Output Signals Displayed

- `{prefix}.SpringGain`
- `{prefix}.DamperGain`
- `{prefix}.Friction`
- `{prefix}.LoadForce`
- `{prefix}.TrimOffset`

### Commit Highlights

- Add FFB Outputs panel showing live runtime values
- Add GetGraphOutputValue() public accessor to DiyFfbPlugin

---

## 2026-01-25: Fix Include Node Preview Always Showing Zero

### Summary

Fixed include nodes always showing zero as preview output in the graph editor.

### Root Cause

`GraphPreviewEvaluator` created `GraphCompiledEvaluator` without passing a resolver. Without a resolver, `EvalInclude()` cannot load referenced graphs from disk and returns early with no computed outputs.

### Fix

1. Added `SetResolver()` method to `GraphPreviewEvaluator` to accept an `IGraphResolver`
2. Pass resolver to `GraphCompiledEvaluator` constructor
3. Changed `BaseDirectory` in `GraphEditorControl` from auto-property to full property
4. `BaseDirectory` setter now calls `UpdatePreviewResolver()` which creates a `GraphIncludeResolver` when a valid directory is set

### Commit Highlights

- Pass resolver to preview evaluator for include node support

---

## 2026-01-25: Fix Bi-directional Parameter Slider Sync

### Summary

Fixed broken bi-directional sync where moving a slider on the function tab changed the value in the graph editor but didn't update the slider on the parameter node.

### Root Cause

In `UpdateParamValue()`, line 674 compared `port.Name` (raw port name like "IAS_kts") against the parameter name from the plugin event (full hierarchical name like "XPlane.IAS_kts"). This mismatch caused the lookup to fail when hierarchical naming was used.

### Fix

Changed line 674 in `GraphEditorControl.xaml.cs` to use `GetPortSignalName(nodeVisual.Node, port)` instead of `port.Name`, ensuring the full hierarchical signal name is used for matching.

### Commit Highlights

- Fix UpdateParamValue to use hierarchical signal names for matching

---

## 2026-01-25: Code Review Bug Fixes and UX Improvements

### Summary

Code review identified several issues. Fixed high/medium priority bugs and added UX improvements.

### Changes

1. **High priority fix** (`GraphSerializer.cs`):
   - Fixed `ShouldSerializeTitle()` bug: library graph Input/Output/Param nodes now preserve titles

2. **Medium priority fixes** (`GraphEditorControl.xaml.cs`):
   - Added 300ms debounce timer for `EditIncludePath_TextChanged()` to prevent file I/O on every keystroke
   - Fixed `DuplicateNode()` to copy `SignalGroup` and port `SignalSuffix`

3. **Low priority improvements** (`GraphEditorControl.xaml/.cs`):
   - Added dirty indicator (orange bullet "•") next to "Inspector" header
   - `IsDirty` property, `DirtyChanged` event, `ClearDirty()` method

4. **Path normalization fix** (`GraphSerializer.cs`):
   - `ExtractInterfaceFromPath()` now normalizes forward slashes to backslashes on Windows

5. **BaseDirectory timing fix** (`GraphEditorTab.cs`):
   - Set `EditorControl.BaseDirectory` BEFORE `Graph = loadedGraph`
   - `SetGraph()` triggers `SyncIncludePorts()` which needs BaseDirectory for relative path resolution

6. **AddPort/RemovePort selection fix** (`GraphEditorControl.xaml.cs`):
   - Restore `_selectedNode` from `_nodeVisuals` after `RebuildSurface()`
   - `OnPortNameChanged` now calls `RebuildSurface()` for reliable visual updates after signal selection

7. **Include node port protection** (`GraphEditorControl.xaml.cs`, `GraphEditorControl.xaml`):
   - `ButtonRemovePort_Click` returns early for Include nodes
   - Added `AllowRemove` property to `PortEditEntry` class
   - Added `BooleanToVisibilityConverter` to XAML resources
   - Remove button visibility bound to `AllowRemove` (hidden for Include nodes)

8. **Extended test coverage** (`GraphTestRunner.cs`):
   - Added 4 new tests (32 total): Title serialization, runtime conversion, SignalGroup preservation, SignalSuffix preservation

### Commit Highlights

- Fix ShouldSerializeTitle() for library graph nodes
- Add debounce to include path editing
- Fix DuplicateNode() to preserve SignalGroup/SignalSuffix
- Add dirty indicator to inspector
- Normalize include path separators for Windows
- Fix BaseDirectory timing for include path resolution on load
- Fix port signal selection to update visuals via RebuildSurface
- Prevent port removal on Include nodes

---

## 2026-01-25: Library Graph Support (Schema v4)

### Summary

Added support for library graphs - reusable subgraphs where Input/Output nodes use freeform port names instead of binding to the signal catalog. This enables creating generic processing blocks that can be included in multiple top-level graphs.

### Changes

1. **Data model** (`GraphModel.cs`):
   - Added `IsLibraryGraph` property on `GraphDefinition`

2. **Schema v4** (`GraphSerializer.cs`):
   - Bumped `CurrentVersion` to 4
   - Added `IsLibraryGraph` to DTO with conditional serialization
   - Updated `FromModel`/`ToModel` to pass `isLibraryGraph` context
   - Updated `ExtractInterface` to use freeform port Names for library graphs

3. **Editor UI** (`GraphEditorControl.xaml.cs`):
   - Added "Library Graph" checkbox in inspector
   - Updated `SyncPortEntries` to skip signal dropdowns for library graph Input/Output
   - Updated `BuildNodeTitle` to show Title instead of SignalGroup for library graphs
   - Updated `GetPortDisplayLabel` to use Name for library graph ports
   - Updated `OnPortNameChanged` to not set SignalSuffix for library graph ports

4. **Tests** (`GraphTestRunner.cs`):
   - `TestLibraryGraphInterfaceExtraction` - verifies freeform names in interface
   - `TestLibraryGraphSerialization` - verifies IsLibraryGraph persists

### Files Modified

- `GraphModel.cs` - IsLibraryGraph property
- `GraphSerializer.cs` - v4 version, library graph handling in DTOs
- `GraphEditorControl.xaml` - Library Graph checkbox
- `GraphEditorControl.xaml.cs` - Library graph UI logic
- `GraphTestRunner.cs` - Two new library graph tests
- `FFB_Graph_Progress.md` - Updated Done section

### Commit Highlights

- Add IsLibraryGraph flag for reusable library blocks
- Bump schema to v4 with library graph serialization
- Library graph Input/Output nodes use freeform port names

---

## 2026-01-24: Include Node Auto-Surface Ports (Schema v3)

### Summary
Implemented auto-surface ports for Include nodes: ports are now automatically populated from the included graph's Input/Output nodes. Schema bumped to v3 with Include node ports excluded from serialization (derived at load time).

### Changes

1. **Data model** (`GraphModel.cs`):
   - Added `IncludedGraphInterface` class with `Inputs`, `Outputs`, `IsValid`, `Error`
   - Added `CachedInterface` property on `GraphNode`

2. **Interface extraction** (`GraphSerializer.cs`):
   - Added `ExtractInterface(GraphDefinition)` - extracts inputs/outputs from graph
   - Added `ExtractInterfaceFromPath(path, baseDir)` - loads graph and extracts interface
   - Bumped `CurrentVersion` to 3
   - Added `ShouldSerializePorts()` returning false for Include nodes

3. **Port synchronization** (`GraphEditorControl.xaml.cs`):
   - Added `SyncIncludePorts(GraphNode)` - clears and repopulates ports from interface
   - Wired to `EditIncludePath_TextChanged`, `SetGraph()`, new Refresh button
   - Removed manual `AddIncludePort`, `ButtonAddIncludeInput_Click`, `ButtonAddIncludeOutput_Click`

4. **UI updates** (`GraphEditorControl.xaml`):
   - Replaced editable port textboxes with read-only labels
   - Added "Refresh" button next to "Open Include"
   - Added `IncludeErrorText` for interface extraction errors
   - Removed "Add Input" and "Add Output" buttons

5. **Tests** (`GraphTestRunner.cs`):
   - `TestExtractInterfaceFromGraph` - verifies interface extraction
   - `TestV3IncludePortsNotSerialized` - verifies ports excluded from v3 JSON
   - `TestV2IncludePortsMigration` - verifies v2 Include ports load correctly

### Files Modified

- `GraphModel.cs` - IncludedGraphInterface class, CachedInterface property
- `GraphSerializer.cs` - v3 version, ExtractInterface methods, ShouldSerializePorts
- `GraphEditorControl.xaml` - Read-only port display, Refresh button, error text
- `GraphEditorControl.xaml.cs` - SyncIncludePorts, removed manual port handlers
- `GraphTestRunner.cs` - Three new tests for v3 Include behavior

### Commit Highlights

- Add Include auto-surface ports from included graph interface
- Bump schema to v3, exclude Include ports from serialization
- Replace manual port editors with read-only display and Refresh button

---

## 2026-01-24: Schema V2 Conditional Serialization & Graph JSON Updates

### Summary
Added conditional serialization for schema v2 to exclude kind-specific fields when not relevant. Updated all graph JSON files to use new hierarchical signal naming convention.

### Changes

1. **Conditional serialization in GraphNodeDto**:
   - `Op` only serialized for Op nodes
   - `Func` only serialized for Func nodes
   - `IncludePath` only serialized for Include/Func nodes
   - `ConstValue` only serialized for Const nodes
   - `SignalGroup` only serialized for Input/Output/Param nodes

2. **Conditional serialization in GraphPortDto**:
   - `SignalSuffix` only serialized when non-empty

3. **Updated all graph JSON files**:
   - `GraphTest/graphs/plane_basic.json`
   - `GraphTest/graphs/multi_function.json`
   - `GraphTest/graphs/heli_collective.json`
   - `graphs/_embedded/heli_collective.json`
   - `graphs/templates/plane_default.json`

### Files Modified

- `GraphSerializer.cs` - ShouldSerialize methods for conditional serialization
- Graph JSON files - Updated signal names and moved reference values to params

### Commit Highlights

- Add ShouldSerialize methods for kind-specific fields in schema v2
- Update all graph JSONs with hierarchical signal naming
- Move Vref and rotor reference values from Input to Param nodes

---

## 2026-01-24: Update Signal Catalog Naming Convention

### Summary
Updated signal catalog to use hierarchical dot notation for input signals and moved reference values from inputs to parameters.

### Changes

1. **Input signals renamed**:
   - `XPlane.IAS_kts` → `XPlane.Speed.IAS`
   - `XPlane.Alpha_deg` → `XPlane.Angle.Alpha`
   - `XPlane.Beta_deg` → `XPlane.Angle.Beta`
   - `XPlane.PRate` → `XPlane.Rate.Roll`
   - `XPlane.QRate` → `XPlane.Rate.Pitch`
   - `XPlane.RRate` → `XPlane.Rate.Yaw`
   - `XPlane.GNrml` → `XPlane.G_Nrml`
   - `XPlane.AeroTorque.RollNm` → `XPlane.AeroTorque.Roll`
   - `XPlane.AeroTorque.PitchNm` → `XPlane.AeroTorque.Pitch`
   - `XPlane.AeroTorque.YawNm` → `XPlane.AeroTorque.Yaw`
   - `XPlane.MainRotorTorqueNm` → `XPlane.MainRotor.Torque`
   - `XPlane.MainRotorRpm` → `XPlane.MainRotor.Speed`

2. **Moved from inputs to parameters** (per-aircraft tunable):
   - `XPlane.Vref_kts` → `Aircraft.Vref` (parameter)
   - `XPlane.NominalRpm` → `Aircraft.Rotor.SpeedNom` (parameter)
   - `XPlane.MrTorqueRefNm` → `Aircraft.Rotor.TorqueNom` (parameter)

3. **Updated plane_default.json template** with new signal names

### Files Modified

- `GraphSignalCatalogData.cs` - Renamed input signals
- `GraphSignals.cs` - Updated BuildXPlaneInputs mapping
- `graphs/templates/plane_default.json` - Updated signal names and added Aircraft.Vref param

### Commit Highlights

- Rename input signals to hierarchical dot notation (XPlane.Speed.IAS, etc.)
- Move reference values from telemetry inputs to tunable parameters
- Update plane_default.json template with new signal names

---

## 2026-01-24: Fix Param Signal Name Resolution

### Summary
Fixed side effects from schema v2 changes where param dictionary keys and preview signal names were using port suffixes instead of full signal names.

### Changes

1. **Added `GetPortSignalName` helper** - Builds full signal name from `node.SignalGroup` + `port.SignalSuffix`

2. **Fixed `SyncPreviewEntries`** - Uses `GetPortSignalName` instead of `port.Name` for input/param preview entries

3. **Fixed param dictionary keying**:
   - `GetOrCreateParam` calls now use full signal names
   - `BuildParamControl` passes full signal name for param lookups
   - `RenameParam` uses full signal names when renaming params
   - `GetParam` lookup after port rename uses full signal name

### Files Modified

- `GraphEditorControl.xaml.cs` - GetPortSignalName helper, SyncPreviewEntries fix, param name handling

### Commit Highlights

- Add GetPortSignalName helper for full signal name construction
- Fix param dictionary to use full signal names (group.suffix)
- Fix preview entries to use full signal names for runtime matching

---

## 2026-01-24: Graph Schema V2 with SignalGroup/SignalSuffix

### Summary
Added version 2 schema for graph JSON with SignalGroup on nodes and SignalSuffix on ports. Includes migration from v1.

### Changes

1. **Schema version 2** - New fields serialized:
   - `GraphNodeDto.SignalGroup` for Input/Output/Param nodes
   - `GraphPortDto.SignalSuffix` for signal suffix within group

2. **V1 to V2 migration** in `GraphSerializer.Deserialize()`:
   - Extracts group from legacy port names (e.g., "XPlane.IAS_kts" → group="XPlane", suffix="IAS_kts")
   - Defaults to first available group if not detectable

3. **UI fixes for Input/Output/Param nodes**:
   - Hide Title field (SignalGroup replaces it)
   - Show SignalGroup in node title bar
   - Show SignalSuffix in port labels
   - Rebuild node visual on group change

### Files Modified

- `GraphSerializer.cs` - V2 DTOs, migration logic
- `GraphModel.cs` - SignalGroup on GraphNode, SignalSuffix on GraphPort
- `GraphEditorControl.xaml` - PanelTitle wrapper
- `GraphEditorControl.xaml.cs` - BuildNodeTitle, GetPortDisplayLabel, group change handling

### Commit Highlights

- Add graph schema v2 with SignalGroup/SignalSuffix
- Migrate v1 graphs by extracting group from port names
- Show SignalGroup as node title, SignalSuffix as port label
