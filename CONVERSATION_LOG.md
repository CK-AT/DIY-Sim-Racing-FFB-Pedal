# Conversation Log
Purpose: keep cross-machine continuity for this repo.
Update policy: append new entries at the top; include date/time, machine, request, summary, key files, and open items.

## 2026-01-28 08:48:33 +01:00 (CODex)
Request: commit inspector header refinements and context dropdown move.
Summary:
- Streamlined the Selected Node header and removed obsolete live/info fields.
- Ensured library graphs keep a standalone context option with no live data.
- Relocated the context selector to sit directly above the Selected Node header.
- Standardized add-port buttons to the “Add Port” label.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Selected Node header now shows a contextual label (and optional title), without live/info fields.
- Library graphs always expose a standalone context fallback.
- Context dropdown now appears above Selected Node.
- Port add buttons now show “Add Port” across node types.

## 2026-01-27 22:58:00 +01:00 (CODex)
Request: add Ctrl+S and Ctrl+O shortcuts in the graph editor.
Summary:
- Added keyboard shortcuts to save (Ctrl+S) and open (Ctrl+O) alongside undo/redo.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
Open items:
- None.
Commit highlights:
- Graph editor supports Ctrl+S and Ctrl+O.

## 2026-01-27 23:04:00 +01:00 (CODex)
Request: update docs and prepare commit for save/open shortcuts.
Summary:
- Documented Ctrl+S/Ctrl+O in design/progress docs and updated the tabs plan checklist.
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
- `SimHubPlugin/Docs/plans/06_FFB_Graph_Editor_Tabs.md`
Open items:
- Ctrl+W close shortcut still pending.
Commit highlights:
- Docs updated for Ctrl+S/Ctrl+O graph editor shortcuts.

## 2026-01-27 23:12:00 +01:00 (CODex)
Request: rename add-port buttons to a generic label.
Summary:
- Updated Input/Output/Param add-port buttons to show “Add Port” instead of “+ In/+ Out”.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
Open items:
- None.
Commit highlights:
- Port add buttons now show “Add Port” across node types.

## 2026-01-27 23:20:00 +01:00 (CODex)
Request: improve selected-node header to avoid showing irrelevant titles.
Summary:
- Selected node header now shows a context label by node type, only using title for library Input/Output when non-empty.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Selected node header shows contextual info instead of always showing Title.
- Fixed include header path lookup to avoid Path namespace ambiguity.
- Op node titles now surface in the selected-node header when provided.
- Func node titles now surface in the selected-node header when provided.

## 2026-01-27 23:34:00 +01:00 (CODex)
Request: remove obsolete live value/info fields from inspector.
Summary:
- Removed live value and info fields from the inspector panel and cleaned code-behind references.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Inspector header now shows only selection context + template content.

## 2026-01-27 23:41:00 +01:00 (CODex)
Request: ensure standalone context exists for library graphs without live data.
Summary:
- Keep the context dropdown visible with a standalone entry for library graphs when no contexts are available.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Library graphs always have a standalone context fallback.

## 2026-01-27 23:47:00 +01:00 (CODex)
Request: remove redundant node type label under Selected Node.
Summary:
- Dropped the extra node type line since the selected-node header now includes context.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Selected-node header no longer has a separate node type line.

## 2026-01-27 23:52:00 +01:00 (CODex)
Request: show node titles alongside display labels in Selected Node header.
Summary:
- Appended node titles in parentheses after the contextual display label when present.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Selected-node header now shows contextual label plus title in parentheses.

## 2026-01-27 22:40:00 +01:00 (CODex)
Request: update docs and prep commit after inspector/property grid changes.
Summary:
- Updated graph design/progress docs and the inspector plan status to reflect template-only inspector and property grid layout.
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- Consider restoring hierarchical signal picker when time allows.
Commit highlights:
- Template-only inspector for all node types with property-grid style Param editor.
- Include inspector uses read-only interface lists and template-bound status.
- Param default edits preserve manual preview overrides.

## 2026-01-27 22:25:00 +01:00 (CODex)
Request: fix compile errors after removing legacy inspector controls.
Summary:
- Removed remaining code-behind references to legacy inspector fields and panel helpers.
- Updated include browse handler to work with template text box, and removed legacy include UI usage.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Verify IDE property grid layout feels right for Param nodes.
Commit highlights:
- Legacy inspector references fully removed to restore clean build.
- Param default edits now update preview parameter values immediately.
- Param default edits now preserve manual preview overrides.

## 2026-01-27 22:02:00 +01:00 (CODex)
Request: remove legacy inspector panel and keep a no-selection placeholder.
Summary:
- Removed the legacy inspector edit panel and replaced it with a simple no-selection message.
- Inspector now relies solely on per-node templates for all node types.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Consider table-based param layout for per-port defaults/UI metadata.
Commit highlights:
- Legacy inspector panel removed; templates are now the only inspector UI.
- Param inspector now uses a two-column property grid with Range/UI expanders.

## 2026-01-27 21:28:00 +01:00 (CODex)
Request: complete remaining inspector templates (Param/Include) and inline metadata.
Summary:
- Added Param template with inline defaults and UI metadata editing on the selected port.
- Added Include template with path/actions and read-only port lists backed by collections.
- Exposed template bindings and converters for selected port and include status.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Consider replacing the flat signal combo with the hierarchical picker when time allows.
Commit highlights:
- Param/Include inspector templates now handle inline editing and status lists.

## 2026-01-27 21:02:00 +01:00 (CODex)
Request: continue per-node inspector templates for Input/Output.
Summary:
- Added Input/Output inspector templates with signal group and port controls.
- Centralized port row template and added signal group options converter for templates.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Param/Include templates still pending; inline Param UI metadata still uses the dialog.
Commit highlights:
- Input/Output inspector templates now use the shared port editor list.
- Port edit entry types are now public to satisfy template binding access.
- Signal picker popup now anchors to its button to avoid phantom dropdown placement.
- Switched signal selection to a standard combo box to avoid duplicate popup lists.

## 2026-01-27 20:40:00 +01:00 (CODex)
Request: begin per-node inspector templates (Const/Op) after preview window changes.
Summary:
- Added DataTemplateSelector and const/op templates in the inspector panel.
- Wired new template controls to update node values and refresh preview.
- Recorded partial template rollout in graph progress doc.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Extend templates to remaining node types and remove remaining generic inspector fields.
Commit highlights:
- Const/Op inspector now uses templates with dedicated controls.
- Added a Func node template selector entry with a dedicated function picker.
- Added title fields for Const/Op/Func template inspectors.
- Template title fields now allow empty values (clears node title).

## 2026-01-27 20:29:00 +01:00 (CODex)
Request: keep the preview window following the selected tab without reopening a new instance.
Summary:
- Centralized preview window ownership in GraphEditorWindow and reattached it on tab switches.
- Added attach/detach helpers on GraphEditorControl to bind the shared preview window.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Added a modeless Preview Inputs window and toolbar toggle, removing preview lists from the inspector.
- Preview window uses scrollable input/param lists to avoid empty gaps when resized.
- Single preview window instance now follows the active editor tab.

## 2026-01-27 20:14:00 +01:00 (CODex)
Request: fix preview window scaling gap between inputs and params.
Summary:
- Changed the preview window layout so inputs auto-size and params take remaining space.
- Added scroll viewers so resizing reveals more parameters without large blank gaps.
Key files:
- `SimHubPlugin/GraphEditor/PreviewWindow.xaml`
Open items:
- None.
Commit highlights:
- Preview window lists now use scroll viewers with auto/remaining space layout to avoid empty gaps.

## 2026-01-27 20:18:00 +01:00 (CODex)
Request: prevent preview window from closing on tab changes.
Summary:
- Removed the Unloaded handler that was closing the preview window when switching tabs.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Preview window now stays open when switching editor tabs.

## 2026-01-27 20:22:00 +01:00 (CODex)
Request: make preview window follow the selected tab.
Summary:
- Added preview window state accessors on the editor control.
- Switched tabs now re-open the preview window for the newly selected editor when it was previously open.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
Open items:
- None.
Commit highlights:
- Preview window follows the active editor tab when switching tabs.

## 2026-01-27 20:05:00 +01:00 (CODex)
Request: start implementing the inspector panel restructure (phase 1 preview window).
Summary:
- Moved preview inputs/params UI into a new modeless PreviewWindow and removed it from the inspector panel.
- Added a toolbar toggle to open/close the preview window and synchronized live-inputs state/status updates.
- Documented the preview window move in graph design/progress docs.
Key files:
- `SimHubPlugin/GraphEditor/PreviewWindow.xaml`
- `SimHubPlugin/GraphEditor/PreviewWindow.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Per-node inspector templates and remaining inspector cleanup are still pending.
Commit highlights:
- Preview inputs/params now live in a modeless window with a toolbar toggle and shared live-inputs state.

## 2026-01-27 19:20:27 +01:00 (CODex)
Request: finalize inspector plan decisions (templates, preview window, op labels, warnings).
Summary:
- Captured decisions for DataTemplateSelector, preview window placement/behavior, include warning noise, param editing scope, and op input labels/arity.
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 19:08:02 +01:00 (CODex)
Request: expand inspector plan into an implementation hand-off.
Summary:
- Converted inspector plan into a phased implementation plan with ordering and developer notes.
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 19:05:31 +01:00 (CODex)
Request: lock func node ports to function definition in inspector plan.
Summary:
- Updated inspector plan to fix func node ports (no add/remove).
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 19:04:08 +01:00 (CODex)
Request: note future op-node per-input negation in inspector plan.
Summary:
- Added future improvement note for per-input negation toggle on Add/Mul ops.
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 19:00:09 +01:00 (CODex)
Request: note op node input naming rules in inspector plan.
Summary:
- Updated inspector plan to use op-specific input naming and fixed single output for op nodes.
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 18:55:47 +01:00 (CODex)
Request: update inspector plan to configure param UI metadata inline (no dialog).
Summary:
- Noted inline param UI metadata editing in the inspector plan.
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 18:51:16 +01:00 (CODex)
Request: add recommended must-have fields per node type for inspector plan.
Summary:
- Filled the inspector plan with suggested per-node must-have fields and optional advanced fields.
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 18:46:46 +01:00 (CODex)
Request: create inspector panel restructure planning doc (modeless preview window, per-node inspector).
Summary:
- Added plan doc template for inspector restructuring and preview window split.
Key files:
- `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md`
Open items:
- Fill must-have fields per node type.
Commit highlights:
- None (planning only).

## 2026-01-27 17:53:04 +01:00 (CODex)
Request: order function tab parameters by graph layout.
Summary:
- Updated FlightStick/FlightPedals parameter panels to follow graph layout order.
- Documented function tab ordering in FFB graph design and progress docs.
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- None.
Commit highlights:
- Add layout-ordered param list with recursive include ordering
- Use graph order for Vehicle/System/function param UI and surface function params on Vehicle
- Add GraphTest coverage for param ordering with includes

## 2026-01-27 17:50:07 +01:00 (CODex)
Request: update plan to order function tab params by graph layout.
Summary:
- Added function-tab ordering requirement to the param layout restructure plan.
Key files:
- `SimHubPlugin/Docs/plans/11_Param_Control_Layout_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 17:45:04 +01:00 (CODex)
Request: add a test for graph layout param ordering with includes.
Summary:
- Added GraphTest coverage for ordered param layout with nested includes and layout-based ordering.
- Documented the new behavior in graph progress.
Key files:
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- None.
Commit highlights:
- Add GraphTest for param ordering with include recursion

## 2026-01-27 17:41:22 +01:00 (CODex)
Request: implement parameter control layout restructuring (no commit yet).
Summary:
- Added ordered param traversal that respects graph layout and include node ordering for UI usage.
- Updated Vehicle and System tabs to use graph appearance ordering and to include function-group params on Vehicle.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- Consider tests or manual UI verification for include-ordering behavior.
Commit highlights:
- Add layout-ordered param list with recursive include ordering
- Use graph order for Vehicle/System param UI and surface function params

## 2026-01-27 17:35:58 +01:00 (CODex)
Request: add runtime resolver note to param ordering plan.
Summary:
- Documented using the runtime include resolver for include path ordering.
Key files:
- `SimHubPlugin/Docs/plans/11_Param_Control_Layout_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 17:33:38 +01:00 (CODex)
Request: update param layout plan (function params follow appearance order; include params ordered by include node position + recursive layout).
Summary:
- Updated plan to order include parameters relative to the include node and apply recursive layout ordering.
Key files:
- `SimHubPlugin/Docs/plans/11_Param_Control_Layout_Restructure_Plan.md`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 17:24:16 +01:00 (CODex)
Request: create a plan doc for parameter control layout restructuring.
Summary:
- Reverted in-progress code edits and captured the Vehicle tab parameter layout plan.
Key files:
- `SimHubPlugin/Docs/plans/11_Param_Control_Layout_Restructure_Plan.md`
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- None.
Commit highlights:
- None (planning only).

## 2026-01-27 17:03:10 +01:00 (CODex)
Request: plan parameter control layout restructuring (surface function params on Vehicle tab; keep graph top-to-bottom order).
Summary:
- Reviewed Vehicle/System param UI flows and graph param collection to plan ordering changes based on graph node layout.
- Identified current filtering/order behavior to adjust for function params and group ordering.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- Confirm how group ordering should be derived (alpha vs first-occurrence top-to-bottom).
Commit highlights:
- None (planning only).

## 2026-01-27 16:26:03 +01:00 (CODex)
Request: run GraphTest after undo/redo changes.
Summary:
- Built GraphTest with MSBuild (same warnings about System.Buffers and x86 reference conflicts).
- GraphTest: 51/51 passed.
Key files:
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- None.
Commit highlights:
- None (test run only).

## 2026-01-27 16:19:43 +01:00 (CODex)
Request: run tests for undo/redo work.
Summary:
- Built KinematicsTests via MSBuild (warnings about System.Buffers and architecture mismatch persisted).
- KinematicsTests: 18/18 passed.
Key files:
- `SimHubPlugin/KinematicsTests/Program.cs`
Open items:
- None.
Commit highlights:
- None (test run only).

## 2026-01-27 16:13:44 +01:00 (CODex)
Request: implement undo/redo for graph editor.
Summary:
- Added per-tab undo stack with snapshot/restore, debounce for text edits, and dirty tracking via baseline index.
- Wired toolbar buttons and Ctrl+Z/Ctrl+Y shortcuts, plus stack-aware dirty indicators.
- Added basic undo stack test and updated graph progress/plan docs.
Key files:
- `SimHubPlugin/GraphEditor/GraphUndoStack.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorTab.cs`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
- `SimHubPlugin/Docs/plans/10_Graph_Editor_Undo_Redo_Plan.md`
Open items:
- None.
Commit highlights:
- Add per-tab undo/redo stacks with debounced snapshots
- Wire toolbar + keyboard shortcuts for undo/redo
- Update tests and progress docs for undo/redo

## 2026-01-27 15:56:40 +01:00 (CODex)
Request: apply answers to undo/redo plan open questions.
Summary:
- Captured decisions: no extra selection restore, re-derive include ports, debounce text edits.
Key files:
- `SimHubPlugin/Docs/plans/10_Graph_Editor_Undo_Redo_Plan.md`
Open items:
- None.
Commit highlights:
- Record undo/redo plan decisions

## 2026-01-27 15:51:55 +01:00 (CODex)
Request: create undo/redo implementation plan for graph editor.
Summary:
- Added undo/redo plan document with data model, snapshot strategy, and build/test commands.
- Added undo/redo item to graph progress Open list.
Key files:
- `SimHubPlugin/Docs/plans/10_Graph_Editor_Undo_Redo_Plan.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- None.
Commit highlights:
- Add undo/redo plan for graph editor

## 2026-01-27 15:41:10 +01:00 (CODex)
Request: fix Input/Output signal dropdowns and update docs, prepare commit.
Summary:
- Defaulted signal dropdown options to the first group when a node's SignalGroup is empty.
- Documented the dropdown fix in graph progress.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Await user confirmation from running UI test.
Commit highlights:
- Populate Input/Output signal dropdowns without requiring a group change

## 2026-01-27 15:22:28 +01:00 (CODex)
Request: fix Input/Output signal dropdowns not populating until group changes.
Summary:
- Use effective signal group fallback for Input/Output port dropdown options without mutating node state.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- None.
Commit highlights:
- Populate Input/Output signal dropdowns using default group when node group is empty

## 2026-01-27 15:11:20 +01:00 (CODex)
Request: verify Include context auto-selection status and update progress doc.
Summary:
- Confirmed auto-select is implemented (double-click Include with live mode passes context id to new tab).
- Moved auto-select item from Open to Done in graph progress doc.
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- None.
Commit highlights:
- Mark Include context auto-select as done in progress doc

## 2026-01-27 15:09:32 +01:00 (CODex)
Request: check if `SimHubPlugin/Docs/FFB_Graph_Progress.md` is up to date.
Summary:
- Verified the plan reference points to `Docs/plans/04_Include_Preview_Debug_Plan.md` after doc moves.
- No additional progress updates needed for recent changes.
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- None.
Commit highlights:
- None (no changes).

## 2026-01-27 15:08:09 +01:00 (CODex)
Request: move plan docs into `SimHubPlugin/Docs/plans` with indexed filenames.
Summary:
- Created `SimHubPlugin/Docs/plans` and moved plan/design-plan docs with two-digit creation-order prefixes.
- Updated references in conversation log and graph progress doc to new plan paths.
Key files:
- `SimHubPlugin/Docs/plans/01_Vehicle_Tab_Plan.md`
- `SimHubPlugin/Docs/plans/02_FFB_Graph_Template_Rework_Plan.md`
- `SimHubPlugin/Docs/plans/03_Nested_Include_Fix_Plan.md`
- `SimHubPlugin/Docs/plans/04_Include_Preview_Debug_Plan.md`
- `SimHubPlugin/Docs/plans/05_Include_Context_Preview_Plan.md`
- `SimHubPlugin/Docs/plans/06_FFB_Graph_Editor_Tabs.md`
- `SimHubPlugin/Docs/plans/07_FFB_Graph_Grid_Centering.md`
- `SimHubPlugin/Docs/plans/08_Include_Context_Auto_Select_Plan.md`
- `SimHubPlugin/Docs/plans/09_FFB_Graph_CopyPaste_Plan.md`
- `CONVERSATION_LOG.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- None.
Commit highlights:
- Move plan docs into `SimHubPlugin/Docs/plans` with indexed filenames
- Update references to plan docs in log/progress docs

## 2026-01-27 14:57:33 +01:00 (CODex)
Request: update AGENTS.md rules (exclude auto-regenerated protocol build rule).
Summary:
- Added graph serialization/conversion rule and doc-update rule for graph behavior changes.
- Fixed unit-test rule typo/casing.
Key files:
- `AGENTS.md`
Open items:
- None.
Commit highlights:
- Clarify graph serialization/conversion test requirements
- Require design/progress doc updates for graph behavior changes

## 2026-01-27 14:55:42 +01:00 (CODex)
Request: review AGENTS.md.
Summary:
- Read AGENTS.md and identified potential additions/clarifications; no code changes.
Key files:
- `AGENTS.md`
Open items:
- Confirm whether to add suggested AGENTS.md rules (see assistant response).
Commit highlights:
- None (no code changes).

## 2026-01-26: Fix Nested Includes via Resolver Test

### Summary

Fixed the failing `Nested includes via resolver` test. The issue was that nested includes resolved paths relative to the root directory instead of the containing sub-graph's directory.

### Root Cause

When evaluating nested includes:
1. The evaluator passed relative paths to `_resolver.GetGraph(path)`
2. The resolver always resolved paths relative to its initial `_baseDirectory`
3. For nested includes, paths should resolve relative to the sub-graph's directory

Example: `parent.json` includes `sub/middle.json`, which includes `inner/inner.json`
- Expected: `inner/inner.json` resolves to `sub/inner/inner.json`
- Actual: `inner/inner.json` resolved to `inner/inner.json` (wrong directory)

### Fix

Modified `GraphCompiledEvaluator.EvalInclude`:
1. Resolve the include path using `ResolveToAbsolutePath()` BEFORE calling `GetGraph()`
2. Pass the resolved absolute path to the resolver
3. Use the resolved path as the cache key
4. Pass the sub-graph's directory to the sub-evaluator for further nested resolution

Also added `PopulateIncludePorts` call in `GraphRuntimeConverter.ConvertEditorJson` to populate nested include ports when loading editor-format sub-graphs.

### Files Changed

- `GraphTest/GraphCompiledEvaluator.cs`: Pre-resolve paths before calling resolver
- `GraphEditor/GraphRuntimeConverter.cs`: Call `PopulateIncludePorts` using `resolvedFilePath`

### Test Results

- **GraphTest**: 50/50 passed ✅
- **KinematicsTests**: 18/18 passed ✅

---

## 2026-01-26: Fix EditorFormatConverter Delegate Signature Mismatch

### Summary

Fixed GraphTest build failure caused by inconsistent revert of `EditorFormatConverter` delegate signature. Tests now pass 49/50 (was: build failure with 1160 errors).

### Root Cause

An earlier fix changed `EditorFormatConverter` from `Func<string, GraphDefinition>` to `Func<string, string, GraphDefinition>` to support nested include path resolution. A subsequent revert was **incomplete**:

| File | Before Fix | After Fix |
| ---- | ---------- | --------- |
| `GraphIncludeResolver.cs:89` | `Func<string, GraphDefinition>` | `Func<string, string, GraphDefinition>` ✅ |
| `GraphIncludeResolver.cs:93` | `Invoke(json)` | `Invoke(json, resolvedPath)` ✅ |
| `GraphRuntimeConverter.cs:26` | `ConvertEditorJson(string json)` | `ConvertEditorJson(string json, string resolvedFilePath)` ✅ |
| `GraphTestRunner.cs:714,1974` | `(json, resolvedPath) => ...` | unchanged (already 2-arg) |

The revert restored `GraphRuntimeConverter.cs` to 1-arg but left `GraphTestRunner.cs` using 2-arg lambdas, causing CS1593 build error.

### Fix

Updated delegate signature to 2-arg across all files for consistency.

### Files Changed

- `GraphTest/GraphIncludeResolver.cs`: Changed delegate to `Func<string, string, GraphDefinition>`, updated `TryLoadEditorFormat` to pass both args
- `GraphEditor/GraphRuntimeConverter.cs`: Added `resolvedFilePath` parameter to `ConvertEditorJson`

### Test Results

- **KinematicsTests**: 18/18 passed ✅
- **GraphTest**: 49/50 passed (1 failure: `Nested includes via resolver`)

### Remaining Work: Fix "Nested includes via resolver" Test

The test fails because `ConvertEditorJson` doesn't yet USE the `resolvedFilePath` parameter. The signature is correct, but the nested path resolution logic was removed in the revert.

**See dedicated plan:** `SimHubPlugin/Docs/plans/03_Nested_Include_Fix_Plan.md`

### Commit Highlights

- Fix EditorFormatConverter delegate signature mismatch (Func<string,string,GraphDefinition>)
- GraphTest now builds and passes 49/50 tests

---

## 2026-01-26: Revert Changes Breaking Runtime Evaluation

### Summary

Reverted changes that caused runtime graph evaluation to produce absurd values (damper=2349 instead of ~1.5). Multiple changes were contributing to the issue.

### Root Cause

Two problematic changes:
1. `GraphRuntimeConverter.ConvertEditorJson` called `PopulateIncludePorts` on sub-graphs
2. `GraphCompiledEvaluator.EvalInclude` used `ResolveToAbsolutePath` to resolve paths, causing double-resolution

### Fix

1. Renamed `ConvertEditorJsonWithIncludes` to `ConvertEditorJson` and removed `PopulateIncludePorts` call
2. Reverted `GraphCompiledEvaluator.cs` to HEAD (removed debug logging and path resolution changes)
3. Removed all debug logging from `GraphRuntimeConverter.cs`

### Files Changed

- `GraphEditor/GraphRuntimeConverter.cs`: Removed PopulateIncludePorts call and debug logging
- `GraphTest/GraphCompiledEvaluator.cs`: Reverted to HEAD

### Commit Highlights

- Revert PopulateIncludePorts in ConvertEditorJson
- Revert EvalInclude path resolution changes
- Remove debug logging from evaluator and converter

---

## 2026-01-26: Fix Nested Include Path Resolution in Sub-Graphs

### Summary

Fixed nested Include nodes in sub-graphs not evaluating correctly. When a parent graph included a sub-graph that itself contained Include nodes, the nested includes failed to resolve because paths were resolved relative to the parent's directory instead of the sub-graph's own directory.

### Root Cause

When sub-graphs were loaded via `EditorFormatConverter`:

1. `ConvertEditorJsonWithIncludes` called `PopulateIncludePorts(editorGraph, baseDirectory)`
2. `baseDirectory` was the **parent graph's** directory (e.g., `graphs/templates/`)
3. Nested includes in sub-graphs (e.g., `scale_include` with path `common/heli_scale.json` in `heli_cyclic_pitch.json`) resolved to wrong location
4. Expected: `graphs/_embedded/common/heli_scale.json`
5. Actual: `graphs/templates/common/heli_scale.json` (file not found)

### Fix

1. Changed `EditorFormatConverter` delegate signature from `Func<string, GraphDefinition>` to `Func<string, string, GraphDefinition>` to pass the resolved file path
2. `TryLoadEditorFormat` now passes `resolvedPath` to the converter
3. `ConvertEditorJsonWithIncludes` uses `Path.GetDirectoryName(resolvedFilePath)` for `PopulateIncludePorts` instead of the original base directory

### Files Changed

- `GraphTest/GraphIncludeResolver.cs`: Changed delegate signature to include resolved path
- `GraphEditor/GraphRuntimeConverter.cs`: Updated converter to use file's directory for path resolution
- `GraphTest/GraphTestRunner.cs`: Updated test to use new delegate signature

### Commit Highlights

- Fix nested Include paths resolved relative to wrong directory
- Pass resolved file path to EditorFormatConverter delegate
- Use sub-graph's directory for PopulateIncludePorts

---

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

- `SimHubPlugin/Docs/plans/05_Include_Context_Preview_Plan.md`: Detailed implementation plan for Include Context Preview feature (live debugging of sub-graphs with caller's inputs)

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

## 2026-01-24 (DESKTOP-6KO022D)

Request: fix graph editor parameter state not persisting on tab switches.

Summary:
- Removed `SyncParamsFromPlugin()` call from `OnSelectedTabChanged` that was overwriting the graph's in-memory parameter values when switching tabs.
- `SyncParamsFromPlugin` now only called when loading a new graph file (in `LoadGraphFromPath`).
- Removed all debug logging added during investigation from GraphEditorWindow, GraphEditorControl, GraphEditorTabManager, and GraphEditorTab.
Async/out-of-order note:
- No async/out-of-order impact; the fix ensures graph editor in-memory state persists independently of plugin state during tab switches.
Commit highlights:
- Fix parameter state preservation on graph editor tab switches.
- Remove debug logging from graph editor tab management.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorTabManager.cs`
- `SimHubPlugin/GraphEditor/GraphEditorTab.cs`
Open items:
- Multi-tab keyboard shortcuts (Ctrl+W close, Ctrl+S save).

## 2026-01-23 11:24:52 +01:00 (DESKTOP-6KO022D)
Request: add param settings dialog and polish param node widgets.
Summary:
- Added a param settings dialog (widget/label/group/units/step/precision/log/options + default/min/max).
- Replaced inline inspector fields with a Settings button and render widgets next to param outputs.
- Tuned port row spacing and per-widget offsets; BuildParamControl now returns control + offset.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI metadata.
Commit highlights:
- Add param settings dialog and hook it into the inspector.
- Render param widgets beside output labels with per-widget offsets and spacing tweaks.
Key files:
- `SimHubPlugin/GraphEditor/GraphParamUiDialog.xaml`
- `SimHubPlugin/GraphEditor/GraphParamUiDialog.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphModel.cs`
- `SimHubPlugin/GraphEditor/GraphSerializer.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
- `SimHubPlugin/PluginTest/Program.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 11:03:12 +01:00 (DESKTOP-6KO022D)
Request: add spacing between output ports so param sliders don’t overlap.
Summary:
- Increased port row spacing to separate output labels and sliders.
Async/out-of-order note:
- No async/out-of-order impact; editor-only layout.
Commit highlights:
- Increase port row spacing and reuse constant for anchors/labels.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 11:05:48 +01:00 (DESKTOP-6KO022D)
Request: align text box height with slider height in param nodes.
Summary:
- Set param text box height to match the slider height for consistent spacing.
Async/out-of-order note:
- No async/out-of-order impact; editor-only layout.
Commit highlights:
- Align param text box height with slider height.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 11:08:11 +01:00 (DESKTOP-6KO022D)
Request: reduce param text box padding.
Summary:
- Reduced text box padding to make param inputs less cramped.
Async/out-of-order note:
- No async/out-of-order impact; editor-only layout.
Commit highlights:
- Reduce param text box padding for tighter layout.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 11:12:27 +01:00 (DESKTOP-6KO022D)
Request: increase port row spacing for param controls.
Summary:
- Increased port row spacing to add vertical room for sliders/text boxes.
Async/out-of-order note:
- No async/out-of-order impact; editor-only layout.
Commit highlights:
- Increase port row spacing for param controls.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 11:15:02 +01:00 (DESKTOP-6KO022D)
Request: align param text boxes with port label center line.
Summary:
- Nudged param control Y offset to better align text boxes with output labels.
Async/out-of-order note:
- No async/out-of-order impact; editor-only layout.
Commit highlights:
- Align param controls vertically with port labels.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 11:16:58 +01:00 (DESKTOP-6KO022D)
Request: align param controls per widget type.
Summary:
- Apply a different Y offset for text boxes vs sliders to keep labels aligned.
Async/out-of-order note:
- No async/out-of-order impact; editor-only layout.
Commit highlights:
- Per-widget Y offsets for param controls.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 11:20:41 +01:00 (DESKTOP-6KO022D)
Request: move param control offset logic into BuildParamControl.
Summary:
- BuildParamControl now returns both the control and its widget-specific Y offset.
- UpdateNodeSize uses the stored offset instead of type checks.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI layout.
Commit highlights:
- Return param control + offset from BuildParamControl for cleaner layout.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 10:55:07 +01:00 (DESKTOP-6KO022D)
Request: place param widgets beside output labels on the node.
Summary:
- Moved param controls into the port row and aligned them next to output labels.
- Adjusted node sizing and label placement for param output rows.
- Rebuilt positioning logic to keep controls aligned on resize.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI layout.
Commit highlights:
- Render param controls beside output labels on param nodes.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 10:58:21 +01:00 (DESKTOP-6KO022D)
Request: fix duplicate GetOutputPortIndex definition.
Summary:
- Removed the duplicate instance method to resolve the CS0111 error.
Async/out-of-order note:
- No async/out-of-order impact.
Commit highlights:
- Delete duplicate GetOutputPortIndex method.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 10:36:31 +01:00 (DESKTOP-6KO022D)
Request: clean up param inspector fields and align node widgets to configured types.
Summary:
- Removed param min/max/default fields from the inspector and kept them in the settings dialog.
- “Settings...” button now only appears for param ports.
- Param nodes render widget-specific controls; knob uses a distinct slider style.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI metadata.
Commit highlights:
- Simplify inspector and keep param ranges in the settings dialog.
- Render widget-specific controls on param nodes.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 10:48:36 +01:00 (DESKTOP-6KO022D)
Request: refresh param node widgets after editing settings.
Summary:
- Rebuilds the node surface after param settings changes so widget updates render immediately.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI metadata.
Commit highlights:
- Rebuild graph surface after saving param settings.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 10:22:49 +01:00 (DESKTOP-6KO022D)
Request: move param UI settings into a dialog and add direct tuning on param nodes.
Summary:
- Moved param UI editing to a modal settings dialog and included default/min/max there.
- Replaced inline editor fields with a “Settings...” button per param port.
- Added direct tuning controls on param nodes based on widget type.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI metadata.
Commit highlights:
- Add param settings dialog with default/min/max and option list editing.
- Replace inline UI fields and add direct tuning controls on param nodes.
Key files:
- `SimHubPlugin/GraphEditor/GraphParamUiDialog.xaml`
- `SimHubPlugin/GraphEditor/GraphParamUiDialog.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 10:06:18 +01:00 (DESKTOP-6KO022D)
Request: move param UI editing into a dedicated dialog.
Summary:
- Added a param UI editor dialog (widget/label/group/units/step/precision/logscale/options).
- Replaced inline param UI fields with an “Edit UI...” dialog button in the inspector.
- Updated PluginTest to cover param UI schema JSON roundtrip.
- Updated graph progress to note the dialog.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI metadata.
Commit highlights:
- Add param UI editor dialog and hook it into the inspector.
- Keep param UI metadata in graph model/serializer and update PluginTest coverage.
Key files:
- `SimHubPlugin/GraphEditor/GraphParamUiDialog.xaml`
- `SimHubPlugin/GraphEditor/GraphParamUiDialog.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/PluginTest/Program.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 09:41:12 +01:00 (DESKTOP-6KO022D)
Request: add param UI schema for graph params and expose it in the editor.
Summary:
- Added UI metadata to graph params (widget/label/group/units/step/precision/logScale/options) and serialized it.
- Extended the port inspector to edit param UI metadata per param port.
- Added PluginTest coverage for UI schema JSON roundtrip.
- Updated graph design/progress docs to reflect param UI schema handling.
Async/out-of-order note:
- No async/out-of-order impact; editor-only UI metadata.
Commit highlights:
- Add param UI schema to graph model/serializer and editor inspector.
- Add PluginTest UI schema roundtrip.
- Update graph design/progress docs.
Key files:
- `SimHubPlugin/GraphEditor/GraphModel.cs`
- `SimHubPlugin/GraphEditor/GraphSerializer.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/PluginTest/Program.cs`
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Surface param UI schema controls in function UI panels.

## 2026-01-23 09:12:41 +01:00 (DESKTOP-6KO022D)
Request: fix GraphTest build error (duplicate compile item).
Summary:
- Removed the explicit GraphCompiledEvaluator compile include from GraphTest to avoid SDK duplicate compile items.
Async/out-of-order note:
- No async/out-of-order impact; build-only change.
Commit highlights:
- Fix GraphTest compile item duplication.
Key files:
- `SimHubPlugin/GraphTest/GraphTest.csproj`
Open items:
- Re-run GraphTest to confirm NETSDK1022 is resolved.

## 2026-01-23 08:54:11 +01:00 (DESKTOP-6KO022D)
Request: start compiled graph evaluation pipeline.
Summary:
- Added a compiled evaluator that precomputes node order and index lookups, including include handling.
- Wired compiled evaluation into preview and runtime evaluation.
- Added a test to compare compiled vs. legacy evaluator outputs.
Async/out-of-order note:
- Runtime uses compiled evaluator with the same input snapshot timing as before (`lastGraphEvaluation`), so ordering remains consistent.
Commit highlights:
- Add compiled evaluator and hook it into preview/runtime.
- Add compiled-vs-legacy parity test.
Key files:
- `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`
- `SimHubPlugin/GraphEditor/GraphPreviewEvaluator.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
- `SimHubPlugin/GraphTest/GraphTest.csproj`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Run GraphTest to validate compiled evaluator parity.

## 2026-01-23 08:46:59 +01:00 (DESKTOP-6KO022D)
Request: sync the graph progress document with current state.
Summary:
- Collapsed duplicated open sections and aligned Done/In Progress/Open lists to current features.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Continue roadmap (compiled evaluator, param UI schema, persistence).

## 2026-01-23 08:39:36 +01:00 (DESKTOP-6KO022D)
Request: fix GraphSignalCatalogData reference in plugin build.
Summary:
- Added the shared signal catalog source file to the SimHub plugin project.
Commit highlights:
- Wire graph outputs into X-Plane FFB spring/damper/friction/load/trim.
- Split graph signal catalog for shared use in plugin and GraphTest.
- Add output-name uniqueness test using the shared catalog.
Key files:
- `SimHubPlugin/DiyFfbPlugin.csproj`
Open items:
- Rebuild the SimHub plugin to confirm the namespace error is resolved.

## 2026-01-23 08:33:35 +01:00 (DESKTOP-6KO022D)
Request: fix GraphTest build error for graph output name test.
Summary:
- Split graph signal lists into a shared, dependency-free catalog.
- Linked the catalog into GraphTest and re-used it in the uniqueness test.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphSignalCatalogData.cs`
- `SimHubPlugin/GraphSignals.cs`
- `SimHubPlugin/GraphTest/GraphTest.csproj`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- Re-run GraphTest build to confirm the namespace error is resolved.

## 2026-01-23 00:56:32 +01:00 (DESKTOP-6KO022D)
Request: continue roadmap with graph output mapping.
Summary:
- Applied graph output values (spring/damper/friction/load/trim) per function during X-Plane FFB processing.
- Added a small graph test to ensure output signal names are unique.
- Updated graph progress tracking to reflect output mapping now wired.
Async/out-of-order note:
- Uses `lastGraphEvaluation` from the latest DataUpdate tick; ProcessXPlaneFfb reads the most recent outputs without blocking.
Commit highlights:
- Wire graph outputs into X-Plane FFB spring/damper/friction/load/trim.
- Add output-name uniqueness test for the graph runtime.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Verify graph outputs override legacy values as expected per function.

## 2026-01-23 00:44:31 +01:00 (DESKTOP-6KO022D)
Request: keep the grid behavior as-is and commit.
Summary:
- Kept the graph editor grid behavior and clipping adjustment as-is.
- Added a dedicated grid/centering intent doc for future work.
Commit highlights:
- Restore graph canvas clipping behavior.
- Add grid/centering intent doc for future implementation.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/Docs/plans/07_FFB_Graph_Grid_Centering.md`
Open items:
- Revisit grid padding/centering once pan/zoom behavior is locked.

## 2026-01-22 21:22:07 +01:00 (DESKTOP-6KO022D)
Request: revert grid/zoom/pan changes made after 15:34.
Summary:
- Removed the grid background canvas child and restored the grid brush on the canvas.
- Dropped auto-centering logic from `UpdateCanvasExtent` to avoid overwriting pan/zoom.
- Updated graph progress tracking.
Commit highlights:
- Graph editor UX improvements (edge rewiring/preview, tighter nodes, inspector live previews).
- Signal catalog + hierarchical picker with per-port mapping and sample graphs.
- Live input preview plumbing and active graph loading hooks.
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm pan/zoom behavior is back to baseline and decide on a safe grid padding approach.

## 2026-01-22 21:09:15 +01:00 (DESKTOP-6KO022D)
Request: revert graph centering/zoom changes after pan/zoom broke.
Summary:
- Restored the previous centering/clamping behavior and grid background positioning.
- Reverted zoom-to-fit back to the pre-padding version.
- Updated the graph progress document to reflect the rollback and open item.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Reintroduce left/top grid padding without breaking pan/zoom.

## 2026-01-22 20:59:08 +01:00 (DESKTOP-6KO022D)
Request: center graph with grid padding on all sides and keep the initial view fully visible.
Summary:
- Reworked initial centering to compute zoom-to-fit (capped at 1.0) with 200px padding around bounds.
- Sized and positioned the grid background to the padded bounds so it renders left/top padding.
- Updated graph progress tracking.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm grid shows on all four sides with ~200px padding and panning works as expected.

## 2026-01-22 16:12:18 +01:00 (DESKTOP-6KO022D)
Request: grid missing to the left/top of the graph.
Summary:
- Moved the grid into a canvas child so it follows the graph transform.
- Ensured the grid rectangle sizes with the canvas extents.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm grid is visible on all sides after panning.

## 2026-01-22 16:06:48 +01:00 (DESKTOP-6KO022D)
Request: graph still shows top-left; want true centering in the visible area.
Summary:
- Centering now uses the visible graph column size instead of full canvas.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm initial view centers the graph in the visible panel.

## 2026-01-22 16:00:52 +01:00 (DESKTOP-6KO022D)
Request: initial view still hugs top-left; graph not centered.
Summary:
- Apply initial centering once after layout sizing, clamped to padding limits.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm initial view centers the graph without huge empty space.

## 2026-01-22 15:56:55 +01:00 (DESKTOP-6KO022D)
Request: center graph in view with space on all sides.
Summary:
- Centered the graph bounds within the viewport, clamped to padding limits.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm initial view centers the graph without pushing the grid out of view.

## 2026-01-22 15:50:43 +01:00 (DESKTOP-6KO022D)
Request: avoid enormous grid while keeping padding around the graph.
Summary:
- Reworked canvas sizing to pad ~300 px on all sides based on graph bounds.
- Initial translate now uses the same padded bounds, avoiding huge extents.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm performance improves and padding feels sufficient.

## 2026-01-22 15:55:07 +01:00 (DESKTOP-6KO022D)
Request: build error after refactor (missing ApplyInitialTranslate).
Summary:
- Removed stale ApplyInitialTranslate call after folding translate logic into UpdateCanvasExtent.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Rebuild to confirm CS0103 is gone.

## 2026-01-22 15:41:54 +01:00 (DESKTOP-6KO022D)
Request: center graph with more space to the left/top.
Summary:
- Added an initial translate offset to provide top/left padding until the user pans.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm initial view gives sufficient top/left margin without disrupting manual panning.

## 2026-01-22 15:46:08 +01:00 (DESKTOP-6KO022D)
Request: initial translate still shows graph in top-left corner.
Summary:
- Initial translate now aligns the graph bounds to a fixed top/left padding.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm the graph starts centered with visible padding.

## 2026-01-22 15:37:56 +01:00 (DESKTOP-6KO022D)
Request: extend grid far beyond graph bounds.
Summary:
- Increased canvas extents to add ~1000 px padding beyond graph bounds.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm grid extends far enough in all directions while panning.

## 2026-01-22 15:34:44 +01:00 (DESKTOP-6KO022D)
Request: func IO changes in inspector do not reflect on the canvas.
Summary:
- Rebuild the surface after func changes so ports update on the node visuals.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm func port changes update on the canvas immediately.

## 2026-01-22 15:30:42 +01:00 (DESKTOP-6KO022D)
Request: allow typing "l" in node title edits (keyboard shortcut conflict).
Summary:
- Ignored editor shortcuts when focus is in a text field or combo box.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm node title editing accepts "l" and other shortcut letters.

## 2026-01-22 15:25:02 +01:00 (DESKTOP-6KO022D)
Request: make signal picker show the current value and be hierarchical.
Summary:
- Replaced the signal ComboBox with a custom popup tree picker.
- Added hierarchical signal grouping and selection logic.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm the popup picker shows the current signal and updates the port name.

## 2026-01-22 15:14:38 +01:00 (DESKTOP-6KO022D)
Request: selector still doesn’t show selected signal.
Summary:
- Switched to SelectedValue binding and disabled text search to show the current signal.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm the selected signal now renders in the closed selector.

## 2026-01-22 15:19:12 +01:00 (DESKTOP-6KO022D)
Request: selector still not showing the selected signal.
Summary:
- Reverted to SelectedItem binding and kept Text binding for editable display.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm selector now shows current port name when closed.

## 2026-01-22 15:10:42 +01:00 (DESKTOP-6KO022D)
Request: fix func port counts/names (assist_loss uses one input; names match functions).
Summary:
- Added function-specific input port schemas (qhat_eff, torque_norm, rpm_norm, assist_loss).
- Func nodes now update ports on load and when the function changes.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm func port names update correctly for existing graphs.

## 2026-01-22 15:05:26 +01:00 (DESKTOP-6KO022D)
Request: fix group drag and missing selection text in signal selector.
Summary:
- Keep multi-selection when dragging a selected node so group moves together.
- Signal selector now binds text to show the current port name even if not in list.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm group drag moves all selected nodes.
- Confirm selector shows current port name in closed state.

## 2026-01-22 14:59:33 +01:00 (DESKTOP-6KO022D)
Request: pan with right mouse and move groups of selected nodes.
Summary:
- Right mouse drag pans; left drag now always selects without needing Shift.
- Multi-select dragging moves all selected nodes together.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm right-drag pan doesn’t interfere with context menus.

## 2026-01-22 14:44:12 +01:00 (DESKTOP-6KO022D)
Request: add a "Live inputs" toggle to drive preview values from telemetry.
Summary:
- Added a Live inputs checkbox with a timer to pull latest input snapshots into the preview.
- Wired the editor window to request live inputs from the plugin.
- Logged progress update in the graph progress document.
Async/out-of-order note:
- Live inputs use the latest X-Plane packet snapshot via `xplaneLock` to avoid stale/out-of-order data.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Verify live inputs populate preview values when enabled.

## 2026-01-22 14:50:20 +01:00 (DESKTOP-6KO022D)
Request: stop the graph from drawing over the inspector and extend the grid.
Summary:
- Clipped the editor surface to its column and resized the canvas to cover larger panning extents.
- Logged progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm the grid background continues during pan/zoom.

## 2026-01-22 14:18:24 +01:00 (DESKTOP-6KO022D)
Request: make graph inputs/outputs selectable so port values map to real signals.
Summary:
- Added signal catalog for known inputs/outputs and wired graph input building to it.
- Switched runtime mapping to per-port IDs for input/param/output nodes.
- Inspector now offers signal selectors on input/output ports; param ports expose default/min/max per port.
- Updated sample graphs to use real signal names for inputs/params/outputs.
- Logged progress updates in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphSignals.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
- `SimHubPlugin/GraphTest/graphs/multi_function.json`
- `SimHubPlugin/GraphTest/graphs/plane_basic.json`
- `SimHubPlugin/GraphTest/graphs/heli_collective.json`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm live X-Plane inputs resolve to non-zero values after mapping.
- Decide if output signal list needs expansion beyond current defaults.

## 2026-01-22 14:23:06 +01:00 (DESKTOP-6KO022D)
Request: document the signal catalog for easier tweaking.
Summary:
- Added a dedicated signal catalog document covering current input/output keys.
- Noted where to update the catalog in code and the naming conventions.
- Logged the progress update in the graph progress document.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm the catalog stays in sync when we add new graph signals.

## 2026-01-22 14:31:36 +01:00 (DESKTOP-6KO022D)
Request: fix missing values on input nodes after signal mapping changes.
Summary:
- Output value labels now use per-port IDs for input/param nodes.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Verify input node value labels show live data after re-open.

## 2026-01-22 14:34:05 +01:00 (DESKTOP-6KO022D)
Request: selector list shows no current selection when closed.
Summary:
- Switched port signal selector to bind `SelectedItem` so chosen names display.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
Open items:
- Confirm selected signal now renders in the closed combo box.

## 2026-01-22 14:38:57 +01:00 (DESKTOP-6KO022D)
Request: make inspector width adjustable.
Summary:
- Added a draggable splitter between the graph canvas and inspector.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
Open items:
- Confirm the inspector can be resized to reveal full port/param fields.

## 2026-01-22 14:27:48 +01:00 (DESKTOP-6KO022D)
Request: fix accessibility errors for graph signal helpers.
Summary:
- Exposed the XPlane UDP packet type to match the internal graph helpers.
Commit highlights:
- (pending)
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- Rebuild to confirm accessibility errors are gone.

## 2026-01-22 13:32:11 +01:00 (DESKTOP-6KO022D)
Request: show current values near each output port in the editor.
Summary:
- Added per-output value labels next to output ports.
- Output labels update from the preview evaluator node values.
- Logged the progress update in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Confirm output values render for include outputs and regular nodes.

## 2026-01-22 13:25:12 +01:00 (DESKTOP-6KO022D)
Request: implement Phase 2 runtime pieces without applying outputs.
Summary:
- Added shared graph runtime converter and compiled evaluator cache in the plugin.
- Wired input/param collection and evaluation per update (no output mapping yet).
- Logged progress for runtime evaluation pipeline.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs`
- `SimHubPlugin/GraphEditor/GraphPreviewEvaluator.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Surface evaluation results in UI/debug and map outputs into FFB later.

## 2026-01-22 13:17:13 +01:00 (DESKTOP-6KO022D)
Request: keep graph editor in sync when active graph changes while open.
Summary:
- Graph selection refresh now reloads the active graph in an open editor window.
- Cached last loaded path to avoid redundant reloads.
- Logged the update in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Verify editor updates when active graph path changes.

## 2026-01-22 13:11:41 +01:00 (DESKTOP-6KO022D)
Request: graph editor should load the active graph path automatically.
Summary:
- Exposed active graph path from the plugin.
- Graph editor window now loads the active graph on open.
- Logged the progress update in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Verify that the editor loads the active graph when opened.

## 2026-01-22 13:04:42 +01:00 (DESKTOP-6KO022D)
Request: convert test graphs to editor schema to avoid empty input nodes.
Summary:
- Rewrote sample graphs in editor JSON format (nodes/ports/links/params).
- Updated the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/graphs/plane_basic.json`
- `SimHubPlugin/GraphTest/graphs/heli_collective.json`
- `SimHubPlugin/GraphTest/graphs/multi_function.json`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Reload graphs in the editor to confirm proper node wiring.

## 2026-01-22 12:56:53 +01:00 (DESKTOP-6KO022D)
Request: add test graphs for the graph editor/runtime.
Summary:
- Added sample graphs for plane, heli collective, and multi-function spring outputs.
- Logged the additions in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/graphs/plane_basic.json`
- `SimHubPlugin/GraphTest/graphs/heli_collective.json`
- `SimHubPlugin/GraphTest/graphs/multi_function.json`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Validate graphs in GraphTest or editor load flow.

## 2026-01-22 12:51:39 +01:00 (DESKTOP-6KO022D)
Request: add UI for per-vehicle/per-game graph selection.
Summary:
- Added graph pickers and active graph status to the X-Plane system tab.
- Exposed graph path getters/setters and active graph status in the plugin.
- Logged the progress update in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Verify UI updates with active vehicle/game changes.

## 2026-01-22 12:44:19 +01:00 (DESKTOP-6KO022D)
Request: start Phase 2 with vehicle-level graph resolution.
Summary:
- Added per-vehicle/per-game graph path settings and runtime graph resolution.
- Active graph loads on game/vehicle changes with validation and fallback.
- Logged the progress update in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Wire UI for graph path selection and surface active graph status.

## 2026-01-22 12:36:01 +01:00 (DESKTOP-6KO022D)
Request: switch to a single top-level graph per vehicle with per-game fallback.
Summary:
- Updated graph design and roadmap to reflect per-vehicle graph resolution.
- Noted the selection model in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Roadmap.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- (none)

## 2026-01-22 12:33:42 +01:00 (DESKTOP-6KO022D)
Request: prep commit for graph editor UX advancements on ck_ffb_graph.
Summary:
- Graph editor UX improvements (edge preview/rewire, sizing, port visibility, single-input enforcement).
- Graph docs/roadmap/progress added and maintained.
- Graph test harness/editor projects integrated into solution and UI.
Commit highlights:
- Graph editor: edge preview/rewire UX, port hover handles, tighter sizing, and input single-link enforcement.
- Docs: graph design/roadmap/progress added and kept current.
- Tooling: GraphEditor/GraphTest/PluginTest added to solution/UI integration.
Key files:
- `.gitignore`
- `AGENTS.md`
- `SimHubPlugin/GraphEditor/`
- `SimHubPlugin/GraphTest/`
- `SimHubPlugin/PluginTest/`
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
- `SimHubPlugin/Docs/FFB_Graph_Roadmap.md`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
- `SimHubPlugin/DiyFfbPlugin.sln`
Open items:
- Confirm commit highlights before committing.

## 2026-01-22 12:27:23 +01:00 (DESKTOP-6KO022D)
Request: allow multiple ports on input/output/param nodes.
Summary:
- Added inspector buttons to add input/output ports for those node types.
- Wired port creation through unique-name helper and surface rebuild.
- Logged the change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm port add buttons show on the expected node types.

## 2026-01-22 12:21:08 +01:00 (DESKTOP-6KO022D)
Request: show connectors when cursor is close to allow edge drawing without selection.
Summary:
- Added hover-based port visibility when the cursor is near a node.
- Kept selection/drag visibility logic intact.
- Logged the change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to verify hover-based port handles appear and do not interfere with drag.

## 2026-01-22 12:17:29 +01:00 (DESKTOP-6KO022D)
Request: edge preview starts from top-left when dragging from inputs.
Summary:
- Preview anchor now uses the pending port kind instead of forcing output anchors.
- Logged the UX fix in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm preview originates from input port anchors.

## 2026-01-22 12:14:49 +01:00 (DESKTOP-6KO022D)
Request: show selected function on func nodes like op nodes.
Summary:
- Appended the selected function name to func node titles.
- Logged the change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm func node titles show the selected function.

## 2026-01-22 12:12:19 +01:00 (DESKTOP-6KO022D)
Request: output edges for op/func/include nodes start from wrong vertical position.
Summary:
- Switched port layout to separate input/output stacks so anchors match visuals.
- Node height now uses max(input, output) port count.
- Logged the change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm edge anchors align with output ports.

## 2026-01-22 12:08:47 +01:00 (DESKTOP-6KO022D)
Request: make node sizing much tighter.
Summary:
- Reduced minimum width, padding, and label spacing for tighter nodes.
- Trimmed extra width slack in sizing calculation.
- Logged the change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm node sizing matches expectations.

## 2026-01-22 12:04:35 +01:00 (DESKTOP-6KO022D)
Request: fix C# 7.3 shadowing error in port connection logic.
Summary:
- Renamed the local link variable to avoid CS0136 shadowing.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Rebuild to confirm CS0136 is resolved.

## 2026-01-22 12:02:29 +01:00 (DESKTOP-6KO022D)
Request: direct edge drag should rewire source or target based on pickup position.
Summary:
- Added pickup-distance logic to choose source vs target rewiring on link drag.
- Logged the UX change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to verify pickup-based source/target rewiring works as expected.

## 2026-01-22 11:58:47 +01:00 (DESKTOP-6KO022D)
Request: node widths still too wide, likely due to label sizing.
Summary:
- Switched node sizing to measured label widths using FormattedText.
- Right-aligned output labels based on measured width and updated sizing logic.
- Logged the change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm tighter node widths with long output labels.

## 2026-01-22 11:54:11 +01:00 (DESKTOP-6KO022D)
Request: prevent multiple edges to a single input.
Summary:
- Enforced single incoming link per input on new links and target rewires.
- Logged the change in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm input ports reject multiple edges.

## 2026-01-22 11:50:38 +01:00 (DESKTOP-6KO022D)
Request: allow dragging edges to a new source.
Summary:
- Added source rewiring mode and preview path for source-side drag.
- Added a context menu option to start source rewiring.
- Updated graph progress tracking.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm source rewiring works and preview path behaves correctly.

## 2026-01-22 11:46:02 +01:00 (DESKTOP-6KO022D)
Request: op labels missing and nodes not resizing to content.
Summary:
- Defaulted Op/Func nodes to a starting operation/function on creation.
- Refresh node title/size when op/func selection changes.
- Noted the update in the graph progress log.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm op labels and sizing updates are visible.

## 2026-01-22 11:41:20 +01:00 (DESKTOP-6KO022D)
Request: edge preview not showing after first use.
Summary:
- Reset preview/selection visuals on graph rebuild so the edge preview is reattached to the canvas.
- Logged the fix in the graph progress document.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm preview edge persists across consecutive attempts.

## 2026-01-22 11:36:57 +01:00 (DESKTOP-6KO022D)
Request: restore edge drop targets while keeping port handles on selected nodes.
Summary:
- Port handles now show while dragging edges or rewiring so drop targets are hittable.
- Kept selected-only handles outside drag/rewire.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Rebuild to confirm edge drop works with handle visibility changes.

## 2026-01-22 11:32:10 +01:00 (DESKTOP-6KO022D)
Request: fix missing edge preview symbols in graph editor.
Summary:
- Added missing edge preview source field and preview start helper.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Rebuild to confirm edge preview fixes the CS0103 errors.

## 2026-01-22 11:06:39 +01:00 (DESKTOP-6KO022D)
Request: show port handles on selected nodes and improve edge drawing feedback.
Summary:
- Added port handle visibility toggles for selected nodes.
- Added preview edge while drawing/reconnecting edges.
- Updated graph progress document with port handle behavior.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Validate port handle visibility if node selection is empty.

## 2026-01-22 10:54:17 +01:00 (DESKTOP-6KO022D)
Request: add a reset option for reroute handles.
Summary:
- Added a "Reset Reroute" option to the edge context menu.
- Updated the graph progress document with the new UX polish item.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- (none)

## 2026-01-22 10:53:07 +01:00 (DESKTOP-6KO022D)
Request: add edge rewiring and auto-sized nodes.
Summary:
- Added edge drag rewiring with preview path during drag.
- Nodes now size to fit titles/ports and Op nodes show their operation.
- Updated graph progress document with new UX polish items.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Consider persisting edge reroute handles in layout metadata.

## 2026-01-22 10:38:13 +01:00 (DESKTOP-6KO022D)
Request: make reroute handles visible.
Summary:
- Reroute handles now appear on edge hover as well as selection/drag.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Consider increasing handle size or adding a toggle to always show them.

## 2026-01-22 10:35:43 +01:00 (DESKTOP-6KO022D)
Request: add rule for correct C#/.NET versions.
Summary:
- Added an AGENTS rule to adhere to the project language/framework versions.
Commit highlights:
- (none yet)
Key files:
- `AGENTS.md`
Open items:
- (none)

## 2026-01-22 10:34:11 +01:00 (DESKTOP-6KO022D)
Request: fix C# 7.3 incompatibility in GraphEditor reroute handle.
Summary:
- Replaced `is not` pattern with C# 7.3-compatible checks.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- (none)

## 2026-01-22 10:30:16 +01:00 (DESKTOP-6KO022D)
Request: add edge reroute handles and update progress tracking.
Summary:
- Added draggable edge reroute handles to adjust Bezier routing.
- Updated the graph progress document with reroute handle status.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Consider persisting manual edge control points in layout metadata.

## 2026-01-22 10:27:22 +01:00 (DESKTOP-6KO022D)
Request: continue UX polish (alignment, distribution, edge curvature).
Summary:
- Added alignment/distribution tools and edge curvature adjustment controls.
- Updated graph progress document with the new UX polish items.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Add visual handles for rerouting or orthogonal edges.

## 2026-01-22 10:23:03 +01:00 (DESKTOP-6KO022D)
Request: add AGENTS rule for graph progress document updates.
Summary:
- Added a rule to keep the graph progress doc updated with graph work.
Commit highlights:
- (none yet)
Key files:
- `AGENTS.md`
Open items:
- (none)

## 2026-01-22 10:21:03 +01:00 (DESKTOP-6KO022D)
Request: continue UX polish and add a progress document.
Summary:
- Added edge tooltips and node context menu actions (delete/duplicate).
- Added a graph progress document and updated it with current status.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Graph_Progress.md`
Open items:
- Keep the progress doc updated as UX work progresses.

## 2026-01-22 10:17:19 +01:00 (DESKTOP-6KO022D)
Request: add mitigations to the graph roadmap.
Summary:
- Added mitigation tasks to the roadmap for schema, compatibility, units, and runtime fallback.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Roadmap.md`
Open items:
- (none)

## 2026-01-22 10:00:49 +01:00 (DESKTOP-6KO022D)
Request: add risks/mitigations to the graph design doc.
Summary:
- Added a risks and mitigations section covering schema, compatibility, and runtime concerns.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
Open items:
- (none)

## 2026-01-22 09:57:46 +01:00 (DESKTOP-6KO022D)
Request: switch structured naming to dot-notation.
Summary:
- Clarified dot-notation convention for inputs/outputs/params naming.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
Open items:
- (none)

## 2026-01-22 09:55:46 +01:00 (DESKTOP-6KO022D)
Request: add param UI schema and naming examples to graph design.
Summary:
- Documented param UI schema fields for graph param nodes.
- Added examples for structured input/output naming.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
Open items:
- Decide on the final naming convention format (dot vs slash).

## 2026-01-22 09:49:12 +01:00 (DESKTOP-6KO022D)
Request: align roadmap with graph design requirements.
Summary:
- Added compilation step, structured naming, param UI schema, multi-port nodes, and layout persistence to the roadmap.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Roadmap.md`
Open items:
- Decide on param UI schema details.

## 2026-01-22 09:47:22 +01:00 (DESKTOP-6KO022D)
Request: incorporate graph design requirements (compiled runtime, naming, params, layout).
Summary:
- Added compilation requirement for fast runtime evaluation.
- Documented structured naming for inputs/outputs/params and param UI grouping.
- Added multi-port nodes, param-driven controls, and layout persistence notes.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
Open items:
- Confirm control types for param nodes (slider/knob/checkbox schema).

## 2026-01-22 09:31:28 +01:00 (DESKTOP-6KO022D)
Request: formalize FFB graph development with design docs.
Summary:
- Added draft design and roadmap documents for the FFB graph system.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/Docs/FFB_Graph_Design.md`
- `SimHubPlugin/Docs/FFB_Graph_Roadmap.md`
Open items:
- Review and refine scope/requirements before next implementation phase.

## 2026-01-21 09:05:04 +01:00 (DESKTOP-6KO022D)
Request: resolve Path ambiguity in include path helpers.
Summary:
- Qualified System.IO.Path usages in MakeRelativePath helpers.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- (none)

## 2026-01-21 09:03:43 +01:00 (DESKTOP-6KO022D)
Request: fix remaining Path ambiguity in GraphEditor.
Summary:
- Qualified Path usages in link creation and LinkVisual to avoid System.IO.Path conflicts.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- (none)

## 2026-01-21 09:01:31 +01:00 (DESKTOP-6KO022D)
Request: fix Path ambiguity in GraphEditor edge handlers.
Summary:
- Qualified Path references in edge event handlers to resolve System.IO.Path vs Shapes.Path.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- (none)

## 2026-01-21 08:58:32 +01:00 (DESKTOP-6KO022D)
Request: improve edge routing visuals.
Summary:
- Switched edges from straight lines to curved Bezier paths with consistent styling.
- Updated edge selection/hover handlers to use path geometry.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Consider configurable curve tension or orthogonal routing.

## 2026-01-21 08:11:49 +01:00 (DESKTOP-6KO022D)
Request: add include path picker and library browser.
Summary:
- Added include path browse button with relative path support.
- Added a library list under the hierarchy tree with open actions.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
Open items:
- Consider persisting include cache across sessions.

## 2026-01-21 07:56:06 +01:00 (DESKTOP-6KO022D)
Request: add hierarchy panel and include navigation entry points.
Summary:
- Added a hierarchy tree panel for root/include graphs with selection-based navigation.
- Added inspector and context menu actions to open include graphs.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- (none)

## 2026-01-21 07:38:41 +01:00 (DESKTOP-6KO022D)
Request: add edge hover cues and context actions.
Summary:
- Added edge hover styling and a context menu to delete a selected edge.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- (none)

## 2026-01-21 07:36:27 +01:00 (DESKTOP-6KO022D)
Request: add edge selection/removal and rounded node boxes.
Summary:
- Added selectable edges with delete support and visual highlighting.
- Switched nodes to rounded borders while keeping port layout intact.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Consider adding edge hover cues or context menu actions.

## 2026-01-21 07:27:13 +01:00 (DESKTOP-6KO022D)
Request: fix GraphEditorControl build error from shadowed locals.
Summary:
- Renamed selection/pan locals in CanvasSurface_MouseMove to avoid shadowing.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- (none)

## 2026-01-20 23:30:11 +01:00 (DESKTOP-6KO022D)
Request: add include port editing and node/port renaming support.
Summary:
- Added include input/output editors with add/remove and rename support.
- Added ports panel with editable port names for all nodes, updating links and visuals.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Consider exposing add/remove ports for non-include nodes if desired.

## 2026-01-20 23:20:16 +01:00 (DESKTOP-6KO022D)
Request: add inspector editing for graph nodes.
Summary:
- Added inspector fields to edit node title, const value, op/func, include path, and param ranges.
- Wired inspector edits to live preview, unique naming, and node label updates.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Add include input/output port editing if needed.

## 2026-01-20 20:22:09 +01:00 (DESKTOP-6KO022D)
Request: improve graph editor ergonomics.
Summary:
- Added grid background, selection rectangle, multi-select, delete, and zoom-to-fit.
- Added snap-to-grid for node dragging and selection visuals.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Add inspector editing for node properties (op/func/const/include).

## 2026-01-20 20:03:05 +01:00 (DESKTOP-6KO022D)
Request: add include/func node support in the graph editor UI.
Summary:
- Added context menu entries and default ports for Const/Func/Include nodes.
- Wired include output ports into preview evaluation via synthetic output IDs.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphPreviewEvaluator.cs`
Open items:
- Add UI for selecting op/func/const/include settings.

## 2026-01-20 19:56:19 +01:00 (DESKTOP-6KO022D)
Request: fix plugin/PluginTest build errors (GraphIncludeResolver + access levels).
Summary:
- Included GraphIncludeResolver/GraphLoader/GraphSaver in the plugin build to satisfy GraphEvaluator dependencies.
- Made GraphPreviewEvaluator public so PluginTest can use it.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/DiyFfbPlugin.csproj`
- `SimHubPlugin/GraphEditor/GraphPreviewEvaluator.cs`
Open items:
- Re-run PluginTest after rebuilding the plugin assembly.

## 2026-01-20 19:51:48 +01:00 (DESKTOP-6KO022D)
Request: add PluginTest to the SimHub solution.
Summary:
- Added the PluginTest project to `DiyFfbPlugin.sln` for easier build/run.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/DiyFfbPlugin.sln`
Open items:
- (none)

## 2026-01-20 19:48:51 +01:00 (DESKTOP-6KO022D)
Request: start a plugin test harness.
Summary:
- Added a net48 PluginTest console project referencing the SimHub plugin.
- Added tests for GraphEditor JSON roundtrip and graph preview evaluation.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/PluginTest/PluginTest.csproj`
- `SimHubPlugin/PluginTest/Program.cs`
Open items:
- Consider adding this project to a solution for one-command builds.

## 2026-01-20 19:42:36 +01:00 (DESKTOP-6KO022D)
Request: fix GraphTest build error from GraphEditor types.
Summary:
- Removed GraphEditor JSON roundtrip test to keep GraphTest isolated from SimHub plugin types.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- If needed, move GraphEditor JSON tests into the plugin test harness.

## 2026-01-20 19:41:26 +01:00 (DESKTOP-6KO022D)
Request: add a rule to keep graph runtime logic shared.
Summary:
- Added an AGENTS rule to reuse shared graph evaluators for editor previews/tests.
Commit highlights:
- (none yet)
Key files:
- `AGENTS.md`
Open items:
- (none)

## 2026-01-20 19:36:54 +01:00 (DESKTOP-6KO022D)
Request: add live evaluation preview for selected graph nodes and integrate the editor into SimHub UI.
Summary:
- Added preview inputs/params in the inspector with live evaluation results for selected nodes.
- Added a graph preview evaluator that converts editor graphs into runtime graphs for evaluation traces.
- Wired graph editor sources into the SimHub plugin project and added a Graph Editor window handle.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
- `SimHubPlugin/GraphEditor/GraphPreviewEvaluator.cs`
- `SimHubPlugin/GraphTest/GraphEvaluator.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
Open items:
- Confirm preview input defaults and extend node types (include/func) in the UI.

## 2026-01-20 19:14:13 +01:00 (DESKTOP-6KO022D)
Request: add an inspector panel for the graph editor.
Summary:
- Added a right-side inspector panel showing selected node metadata and live values.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Integrate the editor in a dedicated window from the SimHub UI.

## 2026-01-20 19:06:18 +01:00 (DESKTOP-6KO022D)
Request: prototype the node graph editor surface.
Summary:
- Added a minimal WPF graph editor control with pan/zoom, node creation, and basic port-to-port wiring.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`
- `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`
Open items:
- Add an inspector panel or live evaluation preview for nodes.

## 2026-01-20 19:02:06 +01:00 (DESKTOP-6KO022D)
Request: draft the graph editor model/binding layer and expand tests.
Summary:
- Added graph editor model/serializer (nodes/ports/links/params) and JSON roundtrip validation.
- Added a GraphTest check covering the editor JSON roundtrip.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphEditor/GraphModel.cs`
- `SimHubPlugin/GraphEditor/GraphSerializer.cs`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- None.

## 2026-01-20 18:54:29 +01:00 (DESKTOP-6KO022D)
Request: start planning the node graph UI for FFB tuning.
Summary:
- Laid out a plan covering framework selection, model/UI binding, editor prototype, and live evaluation previews.
Commit highlights:
- (none yet)
Key files:
- (none)
Open items:
- Choose WPF GraphX vs NodeNetwork (or WebView) for the editor surface.

## 2026-01-20 18:51:57 +01:00 (DESKTOP-6KO022D)
Request: fix include warning test failure due to missing path rule.
Summary:
- Allowed include nodes to omit path when inline graph is present and updated the missing-path test accordingly.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- None.

## 2026-01-20 18:49:12 +01:00 (DESKTOP-6KO022D)
Request: fix failing include mapping warnings test.
Summary:
- Adjusted the include warning test to avoid referencing unmapped outputs while still exercising warnings.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- None.

## 2026-01-20 18:47:03 +01:00 (DESKTOP-6KO022D)
Request: user added a new AGENTS rule.
Summary:
- Recorded the new AGENTS rule: add unit tests for new features whenever feasible.
Commit highlights:
- (none yet)
Key files:
- `AGENTS.md`
Open items:
- None.

## 2026-01-20 18:45:33 +01:00 (DESKTOP-6KO022D)
Request: add additional graph validator checks and tests.
Summary:
- Added op arg-count validation and clamp-bound warnings, plus corresponding tests.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- None.

## 2026-01-20 18:43:36 +01:00 (DESKTOP-6KO022D)
Request: expand GraphTest suite with additional validation scenarios.
Summary:
- Added tests for schema version mismatch, unknown functions, missing output sources, include missing path, and include mapping warnings.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- None.

## 2026-01-20 18:41:04 +01:00 (DESKTOP-6KO022D)
Request: add a printed unit test suite for graph features.
Summary:
- Replaced the single throw-on-fail check with a multi-test runner that prints pass/fail for core features.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
Open items:
- None.

## 2026-01-20 18:36:07 +01:00 (DESKTOP-6KO022D)
Request: add block library index and include validation tests.
Summary:
- Added a block library index for cached includes and validation for include input/output names.
- Added a small validation test runner for include output mapping.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphIncludeResolver.cs`
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/GraphTestRunner.cs`
- `SimHubPlugin/GraphTest/Program.cs`
Open items:
- None.

## 2026-01-20 18:31:49 +01:00 (DESKTOP-6KO022D)
Request: support inline include graphs with block library caching.
Summary:
- Added inline include support, JSON export of embedded graphs, and a resolver that writes embedded graphs to a local library.
- Updated the test harness to exercise inline includes alongside file paths.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphEvaluator.cs`
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/GraphSaver.cs`
- `SimHubPlugin/GraphTest/GraphIncludeResolver.cs`
- `SimHubPlugin/GraphTest/Program.cs`
Open items:
- None.

## 2026-01-20 18:12:15 +01:00 (DESKTOP-6KO022D)
Request: fix include output validation for JSON graph loading.
Summary:
- Allowed output nodes to reference include output mappings without requiring a local node.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphLoader.cs`
Open items:
- None.

## 2026-01-20 18:07:58 +01:00 (DESKTOP-6KO022D)
Request: add include nodes to the graph test harness.
Summary:
- Added include-node support with path/input/output mappings and a simple file resolver.
- Added a sample actuator subgraph and wired it into the test graph + JSON sample.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphEvaluator.cs`
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/GraphSaver.cs`
- `SimHubPlugin/GraphTest/GraphIncludeResolver.cs`
- `SimHubPlugin/GraphTest/graphs/actuator.json`
- `SimHubPlugin/GraphTest/Program.cs`
Open items:
- None.

## 2026-01-20 17:53:24 +01:00 (DESKTOP-6KO022D)
Request: fix GraphTest JSON dependency for net48.
Summary:
- Switched GraphTest JSON handling to Newtonsoft.Json and added the package reference.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTest.csproj`
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/GraphSaver.cs`
Open items:
- None.

## 2026-01-20 17:50:40 +01:00 (DESKTOP-6KO022D)
Request: fix GraphTest build error due to nullable/implicit usings on C# 7.3.
Summary:
- Disabled nullable and implicit usings in GraphTest to keep compatibility with .NET Framework 4.8/C# 7.3.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTest.csproj`
Open items:
- None.

## 2026-01-20 16:43:52 +01:00 (DESKTOP-6KO022D)
Request: align GraphTest target framework with SimHub plugin.
Summary:
- Switched GraphTest to target .NET Framework 4.8 to match the plugin project.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTest.csproj`
Open items:
- None.

## 2026-01-20 09:09:25 +01:00 (DESKTOP-6KO022D)
Request: add schema versioning, strict validation, and JSON export for graph tests.
Summary:
- Added schema version tracking, strict enum/function validation, and JSON export utilities for graph definitions.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphEvaluator.cs`
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/GraphSaver.cs`
- `SimHubPlugin/GraphTest/Program.cs`
Open items:
- None.

## 2026-01-20 09:06:43 +01:00 (DESKTOP-6KO022D)
Request: add JSON parsing and validation to the graph test harness.
Summary:
- Added a JSON loader and validator plus a sample JSON run in the GraphTest console app.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphLoader.cs`
- `SimHubPlugin/GraphTest/Program.cs`
Open items:
- None.

## 2026-01-20 09:00:06 +01:00 (DESKTOP-6KO022D)
Request: add a runnable graph evaluator test harness.
Summary:
- Added a minimal `GraphTest` console app with a sample FFB graph and evaluator.
Commit highlights:
- (none yet)
Key files:
- `SimHubPlugin/GraphTest/GraphTest.csproj`
- `SimHubPlugin/GraphTest/GraphEvaluator.cs`
- `SimHubPlugin/GraphTest/Program.cs`
Open items:
- Decide whether to add JSON parsing and validation for graph definitions.

## 2026-01-20 08:45:12 +01:00 (DESKTOP-6KO022D)
Request: propose a minimal node-graph DSL for flexible FFB tuning.
Summary:
- Sketched a JSON schema for graph nodes (inputs/params/ops/outputs) and a lightweight evaluation model.
Commit highlights:
- (none yet)
Key files:
- (none)
Open items:
- Decide on UI framework (WPF GraphX/NodeNetwork vs WebView + LiteGraph/Rete).

## 2026-01-20 08:32:24 +01:00 (DESKTOP-6KO022D)
Request: prepare a commit for the latest UI/system FFB refinements.
Summary:
- Staged the latest SimHub UI/FFB settings changes and AGENTS/log updates for a new commit.
Commit highlights:
- Added aircraft-change save/discard prompt and removed implicit profile auto-save on new aircraft.
- Added system-wide torque capture toggle and per-function aero torque reference inputs.
- Broadened aero moment gain slider range and fixed auto-tune checkbox syncing per function.
- Hid buffet controls in heli mode and fixed missing buffet panel names in pedals UI.
- Promoted future FFB design doc to current and updated AGENTS rules (FFB design updates, CAN payload guidance, commit highlights).
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/Docs/FFB_Design_Current.md`
- `AGENTS.md`
Open items:
- None.

## 2026-01-20 08:29:26 +01:00 (DESKTOP-6KO022D)
Request: add commit-highlights tracking rule to avoid missing big changes in commit messages.
Summary:
- Added an AGENTS rule to maintain a short "Commit highlights" list in the latest conversation log entry and confirm it before committing.
Commit highlights:
- Added aircraft-change save/discard prompt and removed implicit profile auto-save on new aircraft.
- Added system-wide torque capture toggle and per-function aero torque reference inputs.
- Broadened aero moment gain slider range and fixed auto-tune checkbox syncing per function.
- Hid buffet controls in heli mode and fixed missing buffet panel names in pedals UI.
- Promoted future FFB design doc to current and updated AGENTS rules (FFB design updates, CAN payload guidance, commit highlights).
Key files:
- `AGENTS.md`
Open items:
- None.

## 2026-01-20 08:20:48 +01:00 (DESKTOP-6KO022D)
Request: extend AGENTS rules for CAN payload changes.
Summary:
- Added guidance to document CAN frame sizing and update pack/unpack/cache handling when payloads change.
Key files:
- `AGENTS.md`
Open items:
- None.

## 2026-01-20 08:18:43 +01:00 (DESKTOP-6KO022D)
Request: promote future FFB design to current and add policy to keep it updated.
Summary:
- Replaced `SimHubPlugin/Docs/FFB_Design_Current.md` with the future design content.
- Added an AGENTS rule to update the current FFB design doc whenever the design changes.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Current.md`
- `AGENTS.md`
Open items:
- None.

## 2026-01-20 08:11:56 +01:00 (DESKTOP-6KO022D)
Request: fix auto-tune checkbox state per function in flight stick/pedals.
Summary:
- Synced the auto-tune toggle and internal flag with per-function settings when switching functions.
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-20 08:08:04 +01:00 (DESKTOP-6KO022D)
Request: add system-wide torque capture toggle and manual aero torque refs with broader gain range.
Summary:
- Added a system-level torque capture toggle to gate reference-flight torque tracking.
- Added per-function aero torque reference text boxes and expanded aero moment gain slider ranges.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-20 01:36:17 +01:00 (DESKTOP-6KO022D)
Request: fix aircraft-change prompt timing and add manual aero torque references with better gain range.
Summary:
- Added the save/discard prompt to aircraft changes (previously only on shutdown).
- Added per-function aero torque ref text boxes and widened aero moment gain slider range.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-19 23:05:40 +01:00 (DESKTOP-6KO022D)
Request: update commit scope to reflect full FFB design and include autotune helper.
Summary:
- Staged `SimHubPlugin/Tools.cs` for the X-Plane autotune helper so it ships with the new FFB design work.
- Reframed the pending commit message to cover the full future-state FFB implementation rather than incremental UI updates.
Key files:
- `SimHubPlugin/Tools.cs`
Open items:
- None.

## 2026-01-19 23:00:08 +01:00 (DESKTOP-6KO022D)
Request: prepare a commit for the recent FFB changes to test.
Summary:
- Collected staged changes for FFB tuning updates, UI tweaks, heli buffet hiding, and per-aircraft save prompts.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `proto/diy_ffb_protocol.proto`
- `ESP32/src/CANManager.cpp`
- `ESP32/include/CANManager.h`
- `ESP32/src/Main.cpp`
Open items:
- None.

## 2026-01-19 22:53:04 +01:00 (DESKTOP-6KO022D)
Request: prompt to save/discard per-aircraft FFB edits on aircraft change.
Summary:
- Added a save/discard dialog on aircraft switch when current settings differ from the stored profile or defaults.
- Removed implicit profile creation on unknown aircraft and reset to default FFB values until explicitly saved.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-19 22:23:09 +01:00 (DESKTOP-6KO022D)
Request: finish UI wiring and update AGENTS guidance.
Summary:
- Synced the FlightPedals X-Plane system reference display to show Nominal RPM for helis and Vref for planes.
- Added an AGENTS rule to avoid committing build artifacts unless explicitly requested.
Key files:
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `AGENTS.md`
Open items:
- Verify regenerated protobuf/C# bindings if the new KFriction field is not yet available during builds.

## 2026-01-19 22:36:50 +01:00 (DESKTOP-6KO022D)
Request: hide buffet settings in helicopter mode.
Summary:
- Hid buffet sliders in heli mode for both FlightStick and FlightPedals X-Plane panels.
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-19 22:41:54 +01:00 (DESKTOP-6KO022D)
Request: fix missing buffet panel names in FlightPedals XAML.
Summary:
- Added `Panel_xplane_buffet_*` names so the heli/plane visibility logic can target those panels.
Key files:
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
Open items:
- None.

## 2026-01-19 22:21:30 +01:00 (DESKTOP-6KO022D)
Request: resume after future FFB design edits; confirm rotor auto-selection and begin implementation.
Summary:
- Reviewed the future FFB design doc and confirmed the rotor auto-selection rule (lowest sustained RPM, only when on_ground=false) matches the current implementation.
- No new code changes yet; ready to proceed with any remaining UI polish or firmware/proto regeneration steps.
- Async/out-of-order dependency: rotor auto-selection and max tracking remain gated by 200 ms freshness with a 3 s RPM window to avoid stale data.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- Verify UI wiring for new tuning sliders in `SimHubPlugin/FlightPedalsConfigControl.xaml.cs` and regenerate protobuf/C# bindings if needed.

## 2026-01-19 21:17:47 +01:00 (DESKTOP-6KO022D)
Request: continue implementation (load sign, damping blend, rotor window, friction scaling transport).
Summary:
- Added friction scaling settings, load clamp settings, and main-rotor torque ref persistence.
- Implemented rotor RPM windowed auto-selection, heli damping blend, load sign correction, and load clamp in SimHub processing.
- Extended FlightFfbAction with dynamic friction and applied it on ESP32, including CAN payload updates.
- Async/out-of-order dependency: rotor selection and max tracking use a 200 ms freshness gate with a 3 s RPM window; stale packets do not update maxima.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `proto/diy_ffb_protocol.proto`
- `ESP32/src/CANManager.cpp`
- `ESP32/include/CANManager.h`
- `ESP32/src/Main.cpp`
Open items:
- Add UI controls and labels for the new tuning fields and confirm regeneration of protobuf/C# bindings.

## 2026-01-19 21:10:48 +01:00 (DESKTOP-6KO022D)
Request: start implementing the future-state FFB behavior.
Summary:
- Added new X-Plane FFB tuning fields (k_center, torque/rpm blend, friction scaling knobs, load clamp) and persisted main-rotor torque refs per aircraft profile.
- Implemented telemetry freshness gating, 3 s rotor RPM history, lowest-RPM auto-selection, and reference-flight gating for torque tracking.
- Updated load sign convention, heli damper scaling blend, and load clamp handling in FFB processing.
- Async/out-of-order dependency: max-torque tracking and rotor auto-selection skip stale packets older than 200 ms.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- Implement friction scaling delivery to ESP32 (requires protocol/firmware changes) and add UI controls for new tuning fields.

## 2026-01-19 20:49:32 +01:00 (DESKTOP-6KO022D)
Request: update future design doc with sign convention, telemetry gating, and rotor window, then start implementation.
Summary:
- Added explicit `f_load = -(k * trq_aero_norm)` sign convention, 200 ms telemetry freshness, on-ground gating, and 3 s rotor window to the future doc.
- Async/out-of-order dependency: max tracking and rotor auto-selection must ignore stale telemetry beyond 200 ms.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Begin implementation of the future-state FFB behavior.

## 2026-01-19 20:47:48 +01:00 (DESKTOP-6KO022D)
Request: define rotor auto-selection gating and window duration.
Summary:
- Confirmed rotor RPM auto-selection should use on_ground gating and a 3-second window.
- Async/out-of-order dependency: rotor selection must ignore stale UDP packets beyond 200 ms while accumulating the 3-second window.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- None.

## 2026-01-19 20:44:50 +01:00 (DESKTOP-6KO022D)
Request: confirm remaining implementation inputs for the future FFB design.
Summary:
- Confirmed telemetry freshness uses the same 200 ms budget as ESP, load sign is `f_load = -(k * trq_aero_norm)`, rotor auto-selection should use a few-second window, and multi-rotor datarefs already exist (up to 4 rotors).
- Async/out-of-order dependency: rotor auto-selection and max tracking should ignore stale telemetry older than the 200 ms window.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Choose a concrete auto-selection window duration (e.g., 2–3 s).

## 2026-01-19 20:39:03 +01:00 (DESKTOP-6KO022D)
Request: clarify that only the total load force is clamped.
Summary:
- Updated the future design doc to state that only a total force clamp applies, and only to load terms.
- Async/out-of-order dependency: total clamp should still guard against stale telemetry spikes.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- None.

## 2026-01-19 20:36:43 +01:00 (DESKTOP-6KO022D)
Request: check for any remaining gaps in the future FFB design doc.
Summary:
- Suggested minor doc clarifications (sign convention examples, stale-telemetry timeouts, and a note on per-term clamps vs total clamp).
- Async/out-of-order dependency: telemetry freshness gating should be defined for max tracking and auto-selection.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Decide on the exact telemetry freshness timeout and clamp strategy wording.

## 2026-01-19 20:35:47 +01:00 (DESKTOP-6KO022D)
Request: update rotor auto-selection to pick the lowest RPM rotor (main rotor).
Summary:
- Clarified auto-selection to choose the lowest sustained RPM as the main rotor, avoiding tail rotor selection.
- Async/out-of-order dependency: rotor selection depends on stable RPM telemetry; consider a windowed average to avoid flapping.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- None.

## 2026-01-19 20:32:48 +01:00 (DESKTOP-6KO022D)
Request: add conventions and safety notes to the future FFB design doc.
Summary:
- Added sign conventions, unit conversion notes, rotor auto-selection behavior, and output safety clamps.
- Async/out-of-order dependency: auto-selection and safety clamping should consider stale telemetry to avoid oscillations.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Confirm the desired rotor auto-selection heuristic (highest sustained RPM vs torque).

## 2026-01-19 20:30:01 +01:00 (DESKTOP-6KO022D)
Request: add definitions and clarify gaps in the future FFB design doc.
Summary:
- Added a Definitions/Inputs table and clarified assist_loss clamping, trim vs k_center behavior, and max-torque tracking rules.
- Async/out-of-order dependency: reference-flight max tracking depends on fresh telemetry; gated updates and explicit save flow are required.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Confirm the exact datarefs for `mr_torque` and `rpm` if multiple rotor indices are present.

## 2026-01-19 20:27:41 +01:00 (DESKTOP-6KO022D)
Request: identify blind spots or logical gaps in the future FFB design doc.
Summary:
- Flagged gaps around clamp behavior for assist_loss when rpm_norm > 1.0, missing explicit dataref/unit mapping, and undefined low_rpm_factor/ramp shapes.
- Async/out-of-order dependency: max-torque tracking and profile staging depend on fresh telemetry; needs gating and user-confirmed save flow.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Clarify assist_loss clamp behavior and define low_rpm_factor/ramp function.
- Add a short mapping table for datarefs, units, and normalization inputs.

## 2026-01-19 20:26:18 +01:00 (DESKTOP-6KO022D)
Request: fill in the remaining placeholder in the future design doc rationale sentence.
Summary:
- Replaced the placeholder with “aerodynamic hinge moments” in the non-assisted controls rationale.
- Async/out-of-order dependency: not applicable (documentation only).
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- None.

## 2026-01-19 20:24:58 +01:00 (DESKTOP-6KO022D)
Request: replace the rationale sentence placeholder in the future design doc.
Summary:
- Rewrote the general rationale sentence to avoid servo-valve specificity and cover aerodynamic/rotor loads plus assist flow limits.
- Async/out-of-order dependency: not applicable (documentation only).
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- None.

## 2026-01-19 20:19:37 +01:00 (DESKTOP-6KO022D)
Request: review the new general rationale sentence in the future FFB design doc.
Summary:
- Flagged the sentence as slightly too specific to hydraulic servo-valve systems and suggested broadening it to cover aerodynamic hinge moments and linkage friction, with hydraulics as a contributing factor.
- Async/out-of-order dependency: not applicable (documentation review).
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Decide whether the rationale should be framed as “typical hydraulic” or as a general control‑load explanation.

## 2026-01-19 20:13:10 +01:00 (DESKTOP-6KO022D)
Request: clarify trq_aero_norm clamping and add friction/damper scaling rationales.
Summary:
- Documented that trq_aero_norm is not clamped and added rationale notes for heli/plane friction and damping scaling choices.
- Async/out-of-order dependency: not applicable (documentation only).
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- None.

## 2026-01-19 20:10:56 +01:00 (DESKTOP-6KO022D)
Request: clamp only torque_norm_mr/rpm_norm to 1.1 and make k_rpm_blend tunable.
Summary:
- Updated the future design doc to clamp rotor torque and RPM normalization to 0..1.1 and marked k_rpm_blend as a tunable parameter.
- Async/out-of-order dependency: reference-flight max tracking must be gated on fresh samples before updating maxima.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- None.

## 2026-01-19 20:07:30 +01:00 (DESKTOP-6KO022D)
Request: allow 10% overtorque/overspeed before clamping in the future-state doc.
Summary:
- Noted the clamp range adjustment to 0..1.1 for normalized values to avoid premature clipping.
- Async/out-of-order dependency: not applicable (documentation note).
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Confirm whether the same 1.1 clamp applies to both aero and main-rotor torque normalization.

## 2026-01-19 20:03:58 +01:00 (DESKTOP-6KO022D)
Request: apply future-doc updates (k_center, torque_norm_mr, damping blend, save prompt).
Summary:
- Updated the future design doc with `k_center` naming, `torque_norm_mr` normalization, a heli damping blend formula, and explicit save/discard prompting on aircraft change.
- Async/out-of-order dependency: reference-flight max tracking must be gated on fresh samples before updating maxima.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Decide whether `k_rpm_blend` is a fixed constant or a tunable parameter.

## 2026-01-19 20:01:04 +01:00 (DESKTOP-6KO022D)
Request: clarify heli spring naming, damper blend strategy, torque_norm naming, and profile-save confirmation.
Summary:
- Agreed to rename heli spring to `k_center` (trim-affected centering) and to use `torque_norm_mr` for main-rotor normalization.
- Discussed blending RPM- and torque-based damping and explicit user confirmation before saving modified profiles.
- Async/out-of-order dependency: max-torque tracking must be gated on fresh reference-flight samples before updating saved maxima.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Choose a specific RPM/torque blend function for heli damping.

## 2026-01-19 19:52:05 +01:00 (DESKTOP-6KO022D)
Request: review edits to the future-state FFB design doc.
Summary:
- Reviewed the updated future-state doc and noted open questions on heli spring naming (`k_trim`), damper scaling (torque vs RPM), and naming alignment for torque normalization.
- Async/out-of-order dependency: the tracked max torque references rely on reference-flight sampling; ensure stale/out-of-order telemetry is gated before updating maxima.
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Confirm heli cyclic/pedals spring term naming and whether it maps to trim or centering.
- Decide on damper scaling source for helis (torque vs RPM or blend).

## 2026-01-19 19:08:25 +01:00 (DESKTOP-6KO022D)
Request: rewrite the future-state FFB design doc to reflect the full target state.
Summary:
- Replaced the future doc with a full future-state definition (not just additions), mirroring the current-state structure with planned terms integrated.
- Async/out-of-order dependency: not applicable (documentation only).
Key files:
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Review variable naming in the future doc and align with actual config field names before implementation.

## 2026-01-19 19:00:31 +01:00 (DESKTOP-6KO022D)
Request: split the FFB design doc into current and future state files.
Summary:
- Renamed the original design doc to a current-state version and removed planned items.
- Added a future-state doc capturing agreed additions and ongoing design notes.
- Async/out-of-order dependency: not applicable (documentation only).
Key files:
- `SimHubPlugin/Docs/FFB_Design_Current.md`
- `SimHubPlugin/Docs/FFB_Design_Future.md`
Open items:
- Decide whether to link these docs from `SimHubPlugin/Docs/XPlane_FFB.md`.

## 2026-01-19 18:55:22 +01:00 (DESKTOP-6KO022D)
Request: add heli low-RPM friction to the design list and write an FFB design document.
Summary:
- Created an FFB design doc that consolidates agreed spring/damper/friction/load terms per aircraft type/axis/function.
- Captured the planned heli low-RPM friction term as a future addition.
- Async/out-of-order dependency: not applicable (documentation only).
Key files:
- `SimHubPlugin/Docs/FFB_Design.md`
Open items:
- Decide if the doc should be linked from `SimHubPlugin/Docs/XPlane_FFB.md`.
- Implement the heli low-RPM friction term in code.

## 2026-01-19 17:45:45 +01:00 (DESKTOP-6KO022D)
Request: discuss RPM impact on hydraulic feel (damping/friction).
Summary:
- Noted that lower RPM reduces assist/flow, which can feel heavier but also reduces true viscous damping; suggested separating assist scaling from flow‑limit damping.
- Async/out-of-order dependency: not applicable (design guidance).
Key files:
- None.
Open items:
- None.

## 2026-01-19 16:41:40 +01:00 (DESKTOP-6KO022D)
Request: fix missing torqueRefNm argument in X-Plane diagnostics calls.
Summary:
- Corrected fixed-wing diagnostics calls to pass the new torque reference parameter.
- Async/out-of-order dependency: not applicable (bug fix).
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- None.

## 2026-01-19 14:40:41 +01:00 (DESKTOP-6KO022D)
Request: remove per-function Vref and migrate to system Vref using mean of used functions.
Summary:
- Dropped per-function Vref from profile settings and migrated to system Vref/Nominal RPM using the mean of enabled functions and collective fallback.
- Cleared legacy per-function Vref values after migration; UI now shows system-only references.
- Async/out-of-order dependency: not applicable (settings migration).
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- Decide if legacy Vref values should be retained for audit instead of zeroed.

## 2026-01-19 14:29:53 +01:00 (DESKTOP-6KO022D)
Request: move Vref/RPM reference to system tab with per-aircraft plane/heli selection.
Summary:
- Added system-level plane/heli selector and reference fields (Vref kts, nominal RPM) and saved them per aircraft profile.
- Routed qhat scaling to the system Vref and collective RPM scaling to the system nominal RPM; per-function Vref sliders are now read-only.
- Async/out-of-order dependency: not applicable (UI/settings wiring).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- Decide if per-function Vref fields should be removed entirely from settings.

## 2026-01-19 14:13:46 +01:00 (DESKTOP-6KO022D)
Request: clarify friction/damping scaling: rotor torque for helis, qhat_eff for planes.
Summary:
- Confirmed the plan to scale friction/damping with rotor torque on helicopters and qhat_eff on fixed‑wing aircraft.
- Async/out-of-order dependency: not applicable (design clarification).
Key files:
- None.
Open items:
- None.

## 2026-01-19 14:09:03 +01:00 (DESKTOP-6KO022D)
Request: discuss servo-like feel via torque/IAS-dependent friction + damping.
Summary:
- Confirmed the friction/damping split is a good approximation of servo feel; advised gentle IAS scaling and torque-based friction for static breakaway.
- Async/out-of-order dependency: not applicable (design guidance).
Key files:
- None.
Open items:
- None.

## 2026-01-19 13:29:48 +01:00 (DESKTOP-6KO022D)
Request: provide guidance on torque/IAS-dependent friction.
Summary:
- Discussed that friction should be torque-dependent for static hold and optionally IAS-dependent for aerodynamic feel, with guards to avoid creep/latch.
- Async/out-of-order dependency: not applicable (design guidance).
Key files:
- None.
Open items:
- None.

## 2026-01-19 09:12:18 +01:00 (DESKTOP-6KO022D)
Request: add an AGENTS rule for normalized FFB tuning references.
Summary:
- Added a requirement to normalize FFB tuning parameter references for cross-model consistency.
- Async/out-of-order dependency: not applicable (documentation change).
Key files:
- `AGENTS.md`
Open items:
- None.

## 2026-01-19 09:11:28 +01:00 (DESKTOP-6KO022D)
Request: normalize load gains by per-aircraft max torque and gate auto-tune below Vref.
Summary:
- Added per-function torque reference tracking and normalized load force to “gain @ max torque”.
- Updated load labels to show the torque reference and gated auto-tune updates below a Vref/RPM threshold.
- Async/out-of-order dependency: torque references and diagnostics update with the latest X-Plane packets; auto-tune skips stale/low-speed samples.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- Decide whether to expose a reset for torque references in the UI.

## 2026-01-19 09:03:28 +01:00 (DESKTOP-6KO022D)
Request: propose torque-normalized tuning using per-aircraft max torque references.
Summary:
- Proposed normalizing load/friction/damping gains to an observed per-aircraft max torque and showing it as a reference in the UI.
- Async/out-of-order dependency: not applicable (design discussion).
Key files:
- None.
Open items:
- Decide whether to persist max torque per function/aircraft and how to reset it.

## 2026-01-19 08:46:58 +01:00 (DESKTOP-6KO022D)
Request: confirm load-dependent friction/damping for collective feel.
Summary:
- Agreed to add torque/load-dependent friction and damping alongside load force to avoid creep and tune for hydraulics vs light helis.
- Async/out-of-order dependency: not applicable (design agreement).
Key files:
- None.
Open items:
- Add tuning knobs for load-scaled friction and damping.

## 2026-01-19 08:38:06 +01:00 (DESKTOP-6KO022D)
Request: note that auto-tuning below Vref is unreliable due to low load.
Summary:
- Captured the constraint that auto-tuning should avoid low-speed regimes where load is too small.
- Async/out-of-order dependency: not applicable (design note).
Key files:
- None.
Open items:
- Consider gating auto-tune updates on IAS/RPM vs Vref to avoid low-load samples.

## 2026-01-19 08:25:00 +01:00 (DESKTOP-6KO022D)
Request: add reference-flight auto tuning to freeze aero moment gains.
Summary:
- Added auto-tune toggles to the X-Plane FFB panels and a shared auto-tune helper for load-gain updates.
- Exposed pedal diagnostics in the plugin so the pedals panel can use load-force data for tuning.
- Async/out-of-order dependency: diagnostics updates depend on the latest X-Plane packets; the tuner skips updates when telemetry is stale or axis force is too small.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/Tools.cs`
Open items:
- None.

## 2026-01-19 08:14:45 +01:00 (DESKTOP-6KO022D)
Request: discuss adaptive tuning workflow for aero load balancing.
Summary:
- Proposed a “reference flight” calibration mode to adjust k_aero, then freeze the resulting gain for normal use.
- Async/out-of-order dependency: not applicable (design discussion).
Key files:
- None.
Open items:
- None.

## 2026-01-18 19:12:53 +01:00 (DESKTOP-6KO022D)
Request: collapse unused collective sliders to remove empty gaps.
Summary:
- Added named stack panels for the X-Plane buffet/weathervane/aero-moment controls.
- Collapsed those panels when the collective function is selected so they take no height.
- Async/out-of-order dependency: not applicable (UI-only change).
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-18 19:07:06 +01:00 (DESKTOP-6KO022D)
Request: add load-force support to FlightPedalsFunction.
Summary:
- Added a const force element to the flight pedals function to apply `load_force` from flight FFB.
- Reset load-force state on config updates and timeout recovery.
- Async/out-of-order dependency: not applicable (single function update).
Key files:
- `ESP32/include/FlightPedalsFunction.h`
- `ESP32/src/FlightPedalFunction.cpp`
Open items:
- None.

## 2026-01-18 19:04:17 +01:00 (DESKTOP-6KO022D)
Request: move aero moment gain slider below damper gain and hide unused sliders for collective.
Summary:
- Reordered the aero moment gain slider directly under the damper gain slider in both stick and pedals panels.
- Collapsed buffet, weathervane, and aero moment controls for the collective function to reduce clutter.
- Async/out-of-order dependency: not applicable (UI-only change).
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
Open items:
- None.

## 2026-01-18 18:54:36 +01:00 (DESKTOP-6KO022D)
Request: add aero-moment load forces for non-collective flight FFB, plus a dedicated CAN load-force frame.
Summary:
- Added a per-function aero moment gain slider (signed) and wired L/M/N aero moments into pitch/roll/pedals load-force output.
- Split flight FFB CAN into base + load frames and removed the collective buffet/load repurposing.
- Async/out-of-order dependency: flight load frames can arrive before base frames; the axis caches load and base values to merge them when both are present.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `ESP32/include/CANManager.h`
- `ESP32/src/CANManager.cpp`
Open items:
- None.

## 2026-01-17 22:31:03 +01:00 (DESKTOP-6KO022D)
Request: tighten collective load gain slider range to 0.0–0.1 with 0.0001 steps.
Summary:
- Added a collective-specific slider range for load gain and adjusted label precision for finer tuning.
- Async/out-of-order dependency: not applicable (UI tuning).
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-17 22:27:55 +01:00 (DESKTOP-6KO022D)
Request: fix CAN load force sign loss and persist X-Plane FFB settings per aircraft.
Summary:
- Switched CAN flight FFB payload buffet/load field to signed int16 so negative load forces survive; saved active aircraft profile on shutdown to persist tuning.
- Async/out-of-order dependency: not applicable (serialization + save timing).
Key files:
- `ESP32/src/CANManager.cpp`
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- None.

## 2026-01-17 22:13:43 +01:00 (DESKTOP-6KO022D)
Request: show collective cursor in RPM view while tuning.
Summary:
- The telemetry readout now switches the cursor display to RPM for the collective function.
- Async/out-of-order dependency: diagnostics are gated by fresh X-Plane packets; RPM cursor only updates when diagnostics are available.
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-17 21:54:39 +01:00 (DESKTOP-6KO022D)
Request: add observable readouts for collective/X-Plane parameters while tuning.
Summary:
- Added X-Plane FFB diagnostics capture in the plugin and exposed it to the flight stick tuning UI with a new diagnostics grid.
- Async/out-of-order dependency: telemetry is gated by packet freshness; diagnostics update alongside the X-Plane telemetry timer.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-17 21:16:15 +01:00 (DESKTOP-6KO022D)
Request: invert collective load force so resistance opposes increasing collective.
Summary:
- Flipped the sign on collective load force based on torque so positive torque resists upward collective motion.
- Async/out-of-order dependency: not applicable (force sign change).
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- None.

## 2026-01-17 20:24:36 +01:00 (DESKTOP-6KO022D)
Request: adjust nominal RPM slider range for collective.
Summary:
- Set the collective nominal RPM slider range to 100–600 RPM with 5 RPM steps; restored the fixed airspeed range for non-collective functions.
- Async/out-of-order dependency: not applicable (UI range change).
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-17 20:21:45 +01:00 (DESKTOP-6KO022D)
Request: express collective nominal rotor speed in RPM instead of rad/s.
Summary:
- Converted rotor speed to RPM for collective damping scaling and relabeled the UI to show nominal RPM in rpm units.
- Async/out-of-order dependency: not applicable (unit change only).
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-17 20:17:38 +01:00 (DESKTOP-6KO022D)
Request: update collective UI label to use “Nominal RPM” terminology.
Summary:
- Renamed the collective damper reference label and Vref label to “Nominal RPM” in the flight stick X‑Plane panel.
- Async/out-of-order dependency: not applicable (UI text change).
Key files:
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-17 20:09:12 +01:00 (DESKTOP-6KO022D)
Request: add realistic helicopter collective FFB with rotor selection and load-force support.
Summary:
- Protocol: added `load_force` to `FlightFfbAction` and new `FlightStickCollectiveConfig`.
- ESP32: collective uses `FlightStickFunction` with constant load force; CAN path maps load force via buffet slot for collective frames.
- SimHub/X-Plane: extended UDP packet v2 with torque/omega/prop_ratio arrays, added rotor selector (Auto or specific), and computed collective load/trim/damper from rotor telemetry.
- Async/out-of-order dependency: X-Plane UDP packets are timestamp-gated; collective FFB uses the latest packet within the 500 ms freshness window, and rotor auto selection is based on the newest torque sample.
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/include/FlightStickFunction.h`
- `ESP32/src/FlightStickFunction.cpp`
- `ESP32/src/CANManager.cpp`
- `ESP32/src/Main.cpp`
- `XPlanePlugin/DiyFfbDataProvider.cpp`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
Open items:
- Regenerate protobuf outputs (`SimHubPlugin/DiyFfbProtocol.cs`, ESP32 nanopb) after proto changes.

## 2026-01-17 19:32:44 +01:00 (DESKTOP-6KO022D)
Request: add helicopter collective pitch support reusing the flight stick model.
Summary:
- Protocol: added FUNCTION_ID_FLIGHT_STICK_COLLECTIVE and FlightStickCollectiveConfig, with sizing in options and FunctionConfig wiring.
- ESP32: reused FlightStickFunction for collective, wired config handling and element registration in Main.cpp.
- SimHub: collective support in function defaults, FlightStickConfigControl, function list, and aircraft FFB profile persistence.
- Async/out-of-order dependency: not applicable (config-driven behavior).
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/include/FlightStickFunction.h`
- `ESP32/src/FlightStickFunction.cpp`
- `ESP32/src/Main.cpp`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FunctionConfigControl.xaml.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
Open items:
- Regenerate `SimHubPlugin/DiyFfbProtocol.cs` and firmware nanopb outputs from the updated proto.

## 2026-01-17 19:18:43 +01:00 (DESKTOP-6KO022D)
Request: read project .md files to sync after work on another machine.
Summary:
- Reviewed `AGENTS.md` rules and scanned recent entries in `CONVERSATION_LOG.md` for continuity.
- Async/out-of-order dependency: not applicable (status sync).
Key files:
- `AGENTS.md`
- `CONVERSATION_LOG.md`
Open items:
- None.

## 2026-01-17 02:40:12 +01:00 (DESKTOP-6KO022D)
Request: confirm log spacing fix via negative margins.
Summary:
- User noted negative top/bottom margins resolved the spacing; no code change applied in this step.
- Async/out-of-order dependency: not applicable (status update).
Key files:
- None.
Open items:
- If needed, capture the exact margin values to bake into XAML.

## 2026-01-17 02:38:31 +01:00 (DESKTOP-6KO022D)
Request: reduce log row height to 6 px.
Summary:
- Dropped log font size to 6 px, set fixed 6 px height for fixed columns, and zeroed minimum heights.
- Async/out-of-order dependency: not applicable (UI spacing).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
Open items:
- None.

## 2026-01-17 02:34:10 +01:00 (DESKTOP-6KO022D)
Request: reduce log row height to 6 px.
Summary:
- Set the log row TextBox heights to 6 px for tighter spacing.
- Async/out-of-order dependency: not applicable (UI layout).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
Open items:
- None.

## 2026-01-17 02:31:56 +01:00 (DESKTOP-6KO022D)
Request: fix XAML errors from LineHeight/LineStackingStrategy on TextBox.
Summary:
- Removed unsupported LineHeight/LineStackingStrategy from log TextBoxes and tightened row height using fixed Height/VerticalContentAlignment.
- Async/out-of-order dependency: not applicable (XAML fix).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
Open items:
- None.

## 2026-01-17 02:30:01 +01:00 (DESKTOP-6KO022D)
Request: find origin of "Abort previous message, transmission in progress."
Summary:
- Located in isotp-c library used by the axis firmware; string includes a trailing newline.
- Async/out-of-order dependency: not applicable (investigation).
Key files:
- `ESP32/.pio/libdeps/a6-servo-ffb-axis-controller-v10-ck-at/isotp-c/isotp.c`
Open items:
- None.

## 2026-01-17 02:29:11 +01:00 (DESKTOP-6KO022D)
Request: locate origin of "Abort previous message, transmission in progress."
Summary:
- Searched repo (SimHub + ESP32) for the exact text; no matches found, likely emitted by firmware or an external dependency.
- Async/out-of-order dependency: not applicable (investigation).
Key files:
- None.
Open items:
- Confirm where the log line appears (gateway vs axis) to narrow down.

## 2026-01-17 02:27:45 +01:00 (DESKTOP-6KO022D)
Request: tighten log line spacing again after making lines selectable.
Summary:
- Set fixed line height and removed margins on log TextBoxes to reduce row height.
- Async/out-of-order dependency: not applicable (UI spacing).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
Open items:
- None.

## 2026-01-17 02:25:05 +01:00 (DESKTOP-6KO022D)
Request: make log lines selectable and remove stray trailing line breaks.
Summary:
- Switched log columns to read-only TextBoxes for selection and trimmed trailing newlines when adding log entries.
- Async/out-of-order dependency: not applicable (UI presentation).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 02:16:49 +01:00 (DESKTOP-6KO022D)
Request: normalize log line spacing.
Summary:
- Locked log row padding to zero and set a fixed line height for log columns to avoid uneven spacing.
- Async/out-of-order dependency: not applicable (layout change).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
Open items:
- None.

## 2026-01-17 02:14:02 +01:00 (DESKTOP-6KO022D)
Request: fix autoconnect when the port appears after SimHub starts.
Summary:
- UI now attaches the gateway message handler and updates connection state when auto-reconnect opens the port.
- Async/out-of-order dependency: handled via auto-reconnect timer; UI attach is dispatched to the UI thread when the port becomes available.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 02:08:21 +01:00 (DESKTOP-6KO022D)
Request: keep source checkboxes sorted (Plugin, Gateway, Axis) and prevent autoconnect fallback to COM1.
Summary:
- Source filters now insert in sorted order as they appear online; serial port list refresh no longer overwrites the saved ESPNow port when missing, and autoconnect only runs if the saved port exists.
- Async/out-of-order dependency: not applicable (UI sorting and port selection).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 02:02:36 +01:00 (DESKTOP-6KO022D)
Request: only show source filters for axes/gateways seen online; tighten log line spacing.
Summary:
- Filter list now adds axis/gateway sources only after they come online; log row spacing reduced.
- Async/out-of-order dependency: not applicable (UI filtering and layout).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 01:53:49 +01:00 (DESKTOP-6KO022D)
Request: add a source column and filtering for the log drawer.
Summary:
- Added per-entry source metadata (Plugin/Gateway/Axis), a source column, and filter checkboxes backed by a CollectionView filter.
- Async/out-of-order dependency: not applicable (UI filtering).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 01:50:53 +01:00 (DESKTOP-6KO022D)
Request: add source column and filtering for log entries.
Summary:
- Discussed approach: add Source to log entries, track per message (Plugin/Gateway/Axis), and filter via CollectionView.
- Async/out-of-order dependency: not applicable (UI filtering).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- Decide desired filter UI (single select vs multi-select) and default visibility.

## 2026-01-17 01:45:59 +01:00 (DESKTOP-6KO022D)
Request: move the main UI gateway log into the new log drawer location.
Summary:
- Moved the log drawer UI from the axis tab into the system tab in place of the gateway log panel.
- Async/out-of-order dependency: not applicable (UI relocation).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
Open items:
- None.

## 2026-01-17 01:43:24 +01:00 (DESKTOP-6KO022D)
Request: replace main UI gateway log with the log drawer solution.
Summary:
- Removed the main UI Gateway Log TextBox and routed axis/gateway log messages into the new log drawer with level mapping.
- Async/out-of-order dependency: not applicable (UI log routing).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 01:41:22 +01:00 (DESKTOP-6KO022D)
Request: consider replacing the gateway log with the new log drawer approach.
Summary:
- Pending scope confirmation; need to know which log view to replace and whether to keep OTA dialog logs separate.
- Async/out-of-order dependency: not applicable (UI-only decision).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- Confirm target log view and desired behavior.

## 2026-01-17 01:17:07 +01:00 (DESKTOP-6KO022D)
Request: replace TextBox_debugOutput with a usable log drawer (#2 option).
Summary:
- Added UI log drawer bindings (status line + capped log list) and centralized debug logging with severity levels.
- Async/out-of-order dependency: not applicable (UI-only changes).
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 01:05:34 +01:00 (DESKTOP-6KO022D)
Request: improve debug logging beyond TextBox_debugOutput.
Summary:
- Logged request to design a more useful debug logging UX; awaiting scope/requirements.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- Confirm desired log behavior (history, severity, filtering, export, etc.).

## 2026-01-17 01:00:44 +01:00 (DESKTOP-6KO022D)
Request: stage layout tweaks; include dedicated download buttons and bump FW version in commit message.
Summary:
- Staged UI/layout and download button changes plus conversation log; commit attempt was rejected by user.
Key files:
- `SimHubPlugin/AxisConfigControl.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `CONVERSATION_LOG.md`
Open items:
- Re-run commit with message including "bump ESP32 FW version" when approved.

## 2026-01-17 00:39:54 +01:00 (DESKTOP-6KO022D)
Request: add dedicated download buttons for axis and function configs.
Summary:
- Added Download buttons for axis/function configs that request configs from connected axes.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 00:29:27 +01:00 (DESKTOP-6KO022D)
Request: fix OTA version comparison to handle multi-digit segments.
Summary:
- Replaced string comparison with numeric per-segment comparison in OTA pull update check.
Key files:
- `ESP32/include/ESP32OTAPull.h`
Open items:
- None.

## 2026-01-17 00:22:47 +01:00 (DESKTOP-6KO022D)
Request: redo homing commit without tracking generated protobuf outputs.
Summary:
- Preparing to re-stage and commit homing changes while leaving ignored generated files untracked.
Key files:
- `proto/diy_ffb_protocol.proto`
- `ESP32/src/Main.cpp`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/AxisRequestQueue.cs`
- `CONVERSATION_LOG.md`
Open items:
- Commit the homing changes without adding ignored generated files.

## 2026-01-17 00:12:05 +01:00 (DESKTOP-6KO022D)
Request: add a homing button to the axis tab.
Summary:
- Added a Home button on the axis tab that sends a start_homing axis action to the selected online axis.
- Extended the protocol with start_homing and regenerated protobuf outputs; firmware now handles homing requests.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/AxisRequestQueue.cs`
- `SimHubPlugin/DiyFfbProtocol.cs`
- `proto/diy_ffb_protocol.proto`
- `ESP32/src/Main.cpp`
- `ESP32/sim/diy_ffb_protocol_pb2.py`
Open items:
- None.

## 2026-01-17 00:02:43 +01:00 (DESKTOP-6KO022D)
Request: limit Restart All Axes to online axes only.
Summary:
- Skip offline axes when sending restart actions from the System tab.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- None.

## 2026-01-17 00:01:23 +01:00 (DESKTOP-6KO022D)
Request: add a "Restart All Axes" button in the System tab.
Summary:
- Added a System tab button that sends restart actions to all axes and reports reachability.
- Wired restart requests through the existing axis message send path.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/AxisRequestQueue.cs`
Open items:
- None.

## 2026-01-17 19:06:24 +01:00 (DESKTOP-6KO022D)
Request: improve static balance calibration range, logging, sampling, and reply channel handling.
Summary:
- Calibration now uses contact-point min/max, defaults to 32 samples via auto step sizing, and averages 10 force samples per step.
- Added start/complete logs and ensured results are sent back on the same CommChannel as the request.
Key files:
- `ESP32/include/StaticBalancer.h`
- `ESP32/src/StaticBalancer.cpp`
- `ESP32/src/Main.cpp`
Notes:
- No tests run.

## 2026-01-17 18:42:19 +01:00 (DESKTOP-6KO022D)
Request: route config uploads through the new request queue and make the dispatcher payload-aware.
Summary:
- Added a shared axis request queue with retries and a payload-aware dispatcher to route upload messages alongside request/response traffic.
- Wired axis config uploads and function config uploads to use the queue, keeping direct-send fallbacks intact.
Key files:
- `SimHubPlugin/AxisRequestQueue.cs`
- `SimHubPlugin/Axis.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.csproj`
Notes:
- No tests run.

## 2026-01-16 23:57:56 +01:00 (DESKTOP-6KO022D)
Request: add AGENTS guidance about async/out-of-order data arrival.
Summary:
- Added a rule to call out async/out-of-order data dependencies and their handling.
Key files:
- `AGENTS.md`
- `CONVERSATION_LOG.md`
Open items:
- None.

## 2026-01-16 23:56:47 +01:00 (DESKTOP-6KO022D)
Request: stage and commit updated AGENTS.md.
Summary:
- Read AGENTS.md update and prepared to stage it alongside the required conversation log entry.
Key files:
- `AGENTS.md`
- `CONVERSATION_LOG.md`
Open items:
- Commit the AGENTS update and log entry.

## 2026-01-16 21:53:46 +01:00 (DESKTOP-6KO022D)
Request: defer function range clamping until axis config is present.
Summary:
- Added axis-config-ready tracking so kinematic ranges are only applied after real axis configs arrive.
- Added fallback travel range handling in function controls to avoid early range clamping.
Key files:
- `SimHubPlugin/Axis.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/SplineForceCurve.xaml.cs`
- `SimHubPlugin/AutomotivePedalConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-16 21:07:15 +01:00 (DESKTOP-6KO022D)
Request: fix conversation log ordering to keep newest entries on top.
Summary:
- Reordered recent entries to newest-first and added the latest log entry at the top per AGENTS rule.
Key files:
- `CONVERSATION_LOG.md`
Open items:
- None.

## 2026-01-16 21:04:24 +01:00 (DESKTOP-6KO022D)
Request: add N/mm units and place static balance tick labels outside the plot.
Summary:
- Appended "mm" and "N" units to static balance tick labels and moved labels outside the plot area.
Key files:
- `SimHubPlugin/AxisConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-16 21:01:22 +01:00 (DESKTOP-6KO022D)
Request: show axes with labeled ticks and grid lines for the static balance plot.
Summary:
- Added grid lines, axis lines, and labeled ticks to the static balance canvas.
Key files:
- `SimHubPlugin/AxisConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-16 20:59:15 +01:00 (DESKTOP-6KO022D)
Request: make static balance x range match current kinematic motion range.
Summary:
- When no calibration samples exist, the static balance plot now uses kinematic contact-point min/max (from KinematicParameters) for the x range.
Key files:
- `SimHubPlugin/AxisConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-16 20:55:32 +01:00 (DESKTOP-6KO022D)
Request: plot static balance fit curve even without calibration samples.
Summary:
- Updated static balance plotting to render the fit curve from stored coeffs when no sample data is present.
Key files:
- `SimHubPlugin/AxisConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-16 20:50:31 +01:00 (DESKTOP-6KO022D)
Request: investigate why static balance config is not shown after connecting.
Summary:
- Reviewed axis config/UI flow for static balance and confirmed UI only shows stored coeffs/center/half-range plus live samples after calibration.
Key files:
- `SimHubPlugin/AxisConfigControl.xaml`
- `SimHubPlugin/AxisConfigControl.xaml.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `ESP32/src/ConfigManager.cpp`
Open items:
- Confirm whether the device is returning `AxisConfig.static_balance_config` and whether the user expects stored coeffs or live sample plots.

## 2026-01-16 20:34:46 +01:00 (DESKTOP-6KO022D)
Request: pick up from agents.md and conversation_log-md.
Summary:
- Loaded AGENTS.md and CONVERSATION_LOG.md; awaiting clarification on next task.
Key files:
- `AGENTS.md`
- `CONVERSATION_LOG.md`
Open items:
- Confirm the specific task to continue, and any expected changes in `ESP32/src/ShifterFunction.cpp`.

## 2026-01-16 10:45:00 +01:00 (DESKTOP-6KO022D)
Request: add static balance calibration + tuning across ESP32 + SimHub, with a new AxisConfig tab.
Summary:
- Added StaticBalanceConfig/StaticBalanceResult protocol support (polynomial coeffs + 32 sample cap) and a calibration flow on ESP32 using a state machine.
- Moved StaticBalancer and OscillationGuard into dedicated sources and integrated calibration-driven limit control.
- Added Static Balance tab on AxisConfig with calibration trigger, raw/fit plot, coeff editor, and clear button.
- Added common Static Balance tuning panel (enable + gain) above the FunctionConfig tabs.
- Routed StaticBalanceResult to the correct axis config even when a different axis is selected.
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/include/StaticBalancer.h`
- `ESP32/src/StaticBalancer.cpp`
- `ESP32/include/OscillationGuard.h`
- `ESP32/src/OscillationGuard.cpp`
- `ESP32/src/Main.cpp`
- `SimHubPlugin/AxisConfigControl.xaml`
- `SimHubPlugin/AxisConfigControl.xaml.cs`
- `SimHubPlugin/FunctionConfigControl.xaml`
- `SimHubPlugin/FunctionConfigControl.xaml.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
Open items:
- Validate calibration flow end-to-end on hardware and review fit quality before manual upload.

## 2026-01-16 09:05:10 +01:00 (DESKTOP-6KO022D)
Request: fix trim center markers to reflect absolute travel center.
Summary:
- Store trim-only offsets (no weather-vaning) for UI markers.
- Place trim markers at travel center + trim offset instead of plotting relative offset.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-15 22:12:19 +01:00 (DESKTOP-6KO022D)
Request: rework SimHub OTA flow to match .ffbota + public JSON logic, add target selection and retries, and move OTA diagnostics into the dialog.
Summary:
- Added an OTA target selection dialog (gateway + online axes) with live version/log readouts.
- Implemented an OTA coordinator to update axes first, retry until expected version, then update gateways.
- Updated local OTA hosting to use a lightweight TCP server (no URL ACL) and surfaced the binding URL in the dialog; trimmed noisy debug output.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/OtaSelectionDialog.xaml`
- `SimHubPlugin/OtaSelectionDialog.xaml.cs`
- `SimHubPlugin/OtaUpdateCoordinator.cs`
Open items:
- Validate OTA update flow end-to-end on hardware.

## 2026-01-14 20:44:34 +01:00 (DESKTOP-6KO022D)
Request: finish SimHub plugin tweaks for X-Plane FFB UI and profile handling.
Summary:
- Added pending-profile flow so profiles can be loaded without an active aircraft and applied on aircraft change with a confirmation dialog.
- Updated flight stick/pedals UI with alpha/beta readouts and revised readout layout.
- Switched range slider markers to triangle/diamond shapes and centralized marker scaling to keep position/trim inside selected ranges.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/Tools.cs`
Open items:
- None.

## 2026-01-14 20:40:04 +01:00 (DESKTOP-6KO022D)
Request: refine X-Plane FFB documentation plots.
Summary:
- Smoothed the q_scale curve and gain panel plots with denser path segments.
- Added tick labels and adjusted the q_scale plot to reach 2.0 at 2*Vref.
Key files:
- `SimHubPlugin/Docs/images/xplane_qscale_curve.svg`
- `SimHubPlugin/Docs/images/xplane_gain_panel_mock.svg`
Open items:
- None.

## 2026-01-14 18:01:08 +01:00 (DESKTOP-6KO022D)
Request: refactor physics updates to use a state/accumulator context and make damping/friction order-agnostic.
Summary:
- Replaced SimElement update signatures with `SimState`/`SimAccumulators` and moved damping/friction to a post-pass.
- Aggregated damping contributions and applied the stability clamp once per update.
- Implemented stick/slip friction with static/kinetic parameters and no-creep behavior; friction now runs order-agnostic.
- Added accumulator-based soft limit override for shifter update.
Key files:
- `ESP32/include/Physics.h`
- `ESP32/src/Physics.cpp`
- `ESP32/include/ForceCurve.h`
- `ESP32/src/ForceCurve.cpp`
- `ESP32/include/ShifterFunction.h`
- `ESP32/src/ShifterFunction.cpp`
- `ESP32/include/FlightStickFunction.h`
- `ESP32/src/FlightStickFunction.cpp`
- `ESP32/include/FlightPedalsFunction.h`
- `ESP32/src/FlightPedalFunction.cpp`
Open items:
- Run ESP32 physics unit tests and verify runtime stability on hardware.

## 2026-01-13 20:05:57 +01:00 (DESKTOP-6KO022D)
Request: expand X-Plane FFB UI and documentation, add graph polish, and refactor shared helpers.
Summary:
- Added shared X-Plane math/graph helpers and reused them across SimHub + UI; gain graph now includes axes, gridlines, legend values, and uses the new MaxQScale (2.0) scaling.
- Updated q_scale saturation to 2.0 in the shared helper and aligned UI graph scaling with that cap.
- Added gain panel mock and SVG diagrams; expanded docs with quick navigation, output signal descriptions, tuning-by-feel tips, troubleshooting, and a tuning checklist.
- Added gain cursor, IAS/trim/weathervane readouts, and external tick labels in the X-Plane graphs for stick/pedals.
- Simplified range marker normalization using the shared Tools helper.
Key files:
- `SimHubPlugin/XPlaneFfbMath.cs`
- `SimHubPlugin/XPlaneFfbGraph.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/Docs/XPlane_FFB.md`
- `SimHubPlugin/Docs/images/xplane_ffb_architecture.svg`
- `SimHubPlugin/Docs/images/xplane_qscale_curve.svg`
- `SimHubPlugin/Docs/images/xplane_gain_panel_mock.svg`
- `SimHubPlugin/Docs/images/xplane_tuning_flow.svg`
Open items:
- Build/test the SimHub plugin and verify graph rendering, legend values, and X-Plane telemetry updates.

## 2026-01-12 23:17:47 +01:00 (DESKTOP-6KO022D)
Request: capture the condensed final FFB design decisions.
Summary:
- X-Plane native plugin forwards selected datarefs via UDP; SimHub computes kq/krate/trim/buffet.
- SimHub sends compact FLIGHT_FFB frames per function (pitch/roll/pedals) with CAN ID nibble targeting (no axis_id field).
Key files:
- `XPlanePlugin/DiyFfbDataProvider.cpp`
- `SimHubPlugin/DiyFfbPlugin.cs`
- `ESP32/src/CANManager.cpp`
Open items:
- None.

## 2026-01-12 23:13:37 +01:00 (DESKTOP-6KO022D)
Request: make X-Plane FFB tunable per function with UI controls, add trim/position pointers, persist per-aircraft settings using CarId, add per-function FFB enable toggle, add ESP timeout to restore defaults, and show active aircraft name in System.
Summary:
- Added per-function X-Plane FFB settings in SimHub with UI controls on flight stick/pedals plus range slider markers for live position and trim center.
- Implemented per-aircraft profile save/load keyed by CarId and surfaced CarName in the System tab.
- Added per-function X-Plane FFB enable toggle and gated sends; ESP32 flight functions now restore default damping/spring after 200ms without FFB updates.
Key files:
- `SimHubPlugin/DiyFfbPlugin.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs`
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/FlightStickConfigControl.xaml`
- `SimHubPlugin/FlightStickConfigControl.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml`
- `SimHubPlugin/FlightPedalsConfigControl.xaml.cs`
- `SimHubPlugin/FunctionConfigControl.xaml.cs`
- `ESP32/include/FlightStickFunction.h`
- `ESP32/src/FlightStickFunction.cpp`
- `ESP32/include/FlightPedalsFunction.h`
- `ESP32/src/FlightPedalFunction.cpp`
Open items:
- Build/test SimHub plugin and verify CarId-based switching, UI updates, and ESP timeout behavior.

## 2026-01-11 15:08:33 +01:00 (DESKTOP-6KO022D)
Request: make oscillation guard configurable via the UI (new OscillationGuard in AxisConfig, new AxisConfigControl tab, ms/Hz units).
Summary:
- Logged the scope for adding oscillation guard configuration across protocol, firmware, and UI (ms/Hz units).
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/include/Physics.h`
- `ESP32/src/Main.cpp`
- `SimHubPlugin/AxisConfigControl.xaml`
- `SimHubPlugin/AxisConfigControl.xaml.cs`
- `SimHubPlugin/DiyFfbProtocol.cs`
Open items:
- Add OscillationGuard to the protocol/AxisConfig, apply it in firmware, add the UI tab, and regenerate protobuf outputs.

## 2026-01-11 14:56:00 +01:00 (DESKTOP-6KO022D)
Request: continue UI cleanup without changing the layout; restore gateway auto-reconnect + OTA tab; implement auto-reconnect every 2s.
Summary:
- Restored the axis/system layouts while removing unused legacy controls; moved debug output back into the axis left column.
- Reintroduced the gateway auto-reconnect toggle and OTA tab; OTA now sends StartOtaUpdate (gateway-only) with saved SSID/PASS and channel URL.
- Added a plugin-level gateway auto-reconnect loop (2s) so reconnect works even with the UI closed.
Key files:
- `SimHubPlugin/DiyFfbPluginUI.xaml`
- `SimHubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimHubPlugin/DiyFfbPlugin.cs`
Open items:
- Build the SimHub plugin and verify gateway auto-reconnect + OTA on hardware.

## 2026-01-10 13:33:21 +01:00 (notebook-ckrenn)
Request: add per-folder pinned requirements for ESP32/sim.
Summary:
- Removed root requirements.txt and added pinned dependencies in ESP32/sim/requirements.txt.
Key files:
- `ESP32/sim/requirements.txt`
- `requirements.txt`
Open items:
- None.

## 2026-01-10 12:06:00 +01:00 (notebook-ckrenn)
Request: note OTA pull dependency removal and local header integration.
Summary:
- Removed ESP32OTAPull as a lib dependency and added the modified header to sources.
Key files:
- `include/ESP32OTAPull.h`
- `platformio.ini`
Open items:
- Ensure build includes the local header and no stale library references remain.

## 2026-01-10 12:03:53 +01:00 (notebook-ckrenn)
Request: assess FW versions on OTA CLI start and only update axes that responded.
Summary:
- Added a startup DeviceInfo assessment for all axes and gateways with a printed summary.
- OTA now targets/retries only axes that responded during assessment (skips missing axes or sends per-axis messages as needed).
Key files:
- `ESP32/sim/ota_update_cli.py`
Open items:
- Validate OTA flow on hardware with some axes offline.

## 2026-01-09 10:38:43 +01:00 (notebook-ckrenn)
Request: add DeviceInfo with unique device identifier.
Summary:
- Added DeviceInfo/DeviceInfoRequest to the protocol, including a device UID from eFuse MAC.
- Implemented boot-time DeviceInfo broadcast and request handling in CommManager.
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/src/CommManager.cpp`
- `ESP32/include/CommManager.h`
Open items:
- Regenerate protobuf outputs (C#/Python) if needed by tools.

## 2026-01-08 19:07:49 +01:00 (notebook-ckrenn)
Request: add MD5 verification to OTA updates.
Summary:
- Added `MD5` parsing in ESP32-OTA-Pull and wired it into Update.setMD5.
- Updated OTA JSON and CLI generator to emit the MD5 hash.
Key files:
- `ESP32/.pio/libdeps/a6-servo-ffb-axis-controller-v10-ck-at/ESP32-OTA-Pull/src/ESP32OTAPull.h`
- `OTA/update_info.json`
- `ESP32/sim/ota_update_cli.py`
Open items:
- Validate OTA succeeds with correct MD5 and fails on mismatch.

## 2026-01-08 00:43:21 +01:00 (notebook-ckrenn)
Request: extend OTA CLI to host JSON + firmware binary.
Summary:
- OTA CLI now spins up a local HTTP server that serves `update_info.json` and `firmware.bin`, and sends the generated URL to the device.
Key files:
- `ESP32/sim/ota_update_cli.py`
Open items:
- None.

## 2026-01-08 00:32:18 +01:00 (notebook-ckrenn)
Request: add CLI for OTA updates.
Summary:
- Added an OTA CLI that sends StartOtaUpdate over USB and optionally tails log messages.
Key files:
- `ESP32/sim/ota_update_cli.py`
Open items:
- None.

## 2026-01-07 19:23:24 +01:00 (DESKTOP-PUK6UGO)
Request: optimize ShifterFunction lane selection and fix native PI build error.
Summary:
- Added fallback `PI` definition in `Physics.h` to fix native builds.
- Added seg-to-lane lookup tables and inside-mask reuse (bitmask) in `ShifterGateRuntime` to reduce per-update scans.
Key files:
- `ESP32/include/Physics.h`
- `ESP32/include/ShifterFunction.h`
Open items:
- None.

## 2026-01-07 19:02:48 +01:00 (DESKTOP-PUK6UGO)
Request: shifter detent plotting and cam tuning; fix shifter test signature; add centering to plots.
Summary:
- Updated `test_shifter_native` to call the new `ShifterFunction::update_config` signature (with detect config) and set a usable centering spring in the simple gate config.
- Enhanced detent visualizer `plot_shifter_detents.py` with centering spring support, per-lane force curves, and robust config loading for sparse configItems.
- Added `cam_test.py` to explore lane cam profiles, roller follower effects, centering spring, and more realistic fork-style pocket tuning.
Key files:
- `ESP32/test/test_shifter_native/test_shifter.cpp`
- `ESP32/sim/plot_shifter_detents.py`
- `ESP32/sim/cam_test.py`
Open items:
- Native tests not run here (PlatformIO/gcc unavailable); run `pio test -e native -f test_shifter_native` after installing toolchain.

## 2026-01-03 21:01:21 +01:00 (DESKTOP-6KO022D)
Request: debug USB gateway receive; add logging on ISOTP errors.
Summary:
- Increased PacketSerial receive buffer to 1024 bytes to allow larger USB packets.
- Added throttled SerialManager overflow logging for oversized packets.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `ESP32/include/SerialManager.h`
- `ESP32/src/SerialManager.cpp`
Open items:
- Verify gateway now receives shifter function configs over USB.

## 2026-01-03 20:46:38 +01:00 (DESKTOP-6KO022D)
Request: add ISOTP error logging for CAN path.
Summary:
- Added throttled ISOTP send/receive error logs with return codes and payload lengths.
- Logged CAN ISOTP errors for gateway/axis traffic and log-ack messages.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `ESP32/src/CANManager.cpp`
Open items:
- Check SimHub logs for CAN ISOTP errors during shifter uploads.

## 2026-01-03 18:04:21 +01:00 (DESKTOP-6KO022D)
Request: plan cleanup of legacy pre-protobuf struct-based protocol and unsafe code in the plugin.
Summary:
- Identified the main legacy surface area (DAP_config_st/payload structs, unsafe marshaling, raw checksum helpers) in the plugin.
- Outlined a cleanup plan focusing on removing legacy structs/handlers and consolidating on protobuf-based config flow.
Key files:
- `SimhubPlugin/DiyFfbPlugin.cs`
- `SimhubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimhubPlugin/OnlineProfile.xaml.cs`
Open items:
- Confirm which legacy import/export paths (if any) must remain before removal.

## 2026-01-03 16:28:43 +01:00 (DESKTOP-6KO022D)
Request: shifter UI polish (live marker + axis range limits) and fix build error.
Summary:
- Added live shifter position marker driven by AxisState (with sequential fallback to X midpoint).
- Unlocked X range in sequential mode and clamped X/Y range sliders to selected axis travel ranges.
- Fixed CS0206 by avoiding ref on protobuf properties in range clamp helper.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-03 16:10:28 +01:00 (DESKTOP-6KO022D)
Request: shifter detection/visualization updates, sequential demo, and X-midpoint centering.
Summary:
- Renamed shifter detection schema to gear-based slots (ShifterGear enum + gear_slots) and updated firmware/tests/UI + demo configs.
- Fixed empty gear dropdown by setting the DataGridComboBoxColumn ItemsSource in code-behind.
- Added shifter force-field plotter, sequential demo config, and sequential X-midpoint sampling derived from the X motion range.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `proto/diy_ffb_protocol.proto`
- `proto/diy_ffb_protocol.options`
- `ESP32/src/ShifterDetect.cpp`
- `ESP32/test/test_shifter_native/test_shifter.cpp`
- `ESP32/include/ShifterFunction.h`
- `ESP32/src/ShifterFunction.cpp`
- `ESP32/sim/plot_shifter_force_fields.py`
- `SimhubPlugin/ShifterConfigControl.xaml`
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
- `SimhubPlugin/shifter_hpattern_demo_config.json`
- `SimhubPlugin/shifter_hpattern_demo_config_modified.json`
- `SimhubPlugin/shifter_sequential_demo_config.json`
Open items:
- None.

## 2026-01-03 02:42:55 +01:00 (DESKTOP-6KO022D)
Request: remove shifter controller-axis output UI (buttons only).
Summary:
- Removed controller axis selector from shifter config UI and force the output axis to Undefined.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/ShifterConfigControl.xaml`
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
Open items:
- None.

## 2026-01-03 02:38:05 +01:00 (DESKTOP-6KO022D)
Request: add SimHub UI for shifter config editing.
Summary:
- Added ShifterConfigControl with axis selection, range inputs, force settings, and editable gate/detent tables.
- Wired shifter tab into FunctionConfigControl and function list population.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/ShifterConfigControl.xaml`
- `SimhubPlugin/ShifterConfigControl.xaml.cs`
- `SimhubPlugin/FunctionConfigControl.xaml`
- `SimhubPlugin/FunctionConfigControl.xaml.cs`
- `SimhubPlugin/DiyFfbPluginUI.xaml.cs`
- `SimhubPlugin/DiyFfbPlugin.csproj`
Open items:
- None.

## 2026-01-03 02:14:03 +01:00 (DESKTOP-6KO022D)
Request: implement shifter physics + config (gate geometry, detents, sequential).
Summary:
- Protocol: added shifter function ID and ShifterConfig with gate segments and detent points.
- Firmware: added ShifterFunction with 2D LUT-based force field and axis role handling.
- Regenerated C# and Python protobuf outputs.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `proto/diy_ffb_protocol.proto`
- `ESP32/include/ShifterFunction.h`
- `ESP32/src/ShifterFunction.cpp`
- `ESP32/src/Main.cpp`
- `proto/diy_ffb_protocol.options`
- `SimHubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`
Open items:
- None.

## 2026-01-03 01:54:23 +01:00 (DESKTOP-6KO022D)
Request: add shifter detection config + 8 joystick buttons.
Summary:
- Protocol: added shifter slot detection config and wired it into AuxFunctionConfig; updated nanopb options.
- Firmware: added ShifterDetect aux function to set joystick buttons from slot regions.
- Regenerated C# and Python protobuf outputs.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `proto/diy_ffb_protocol.proto`
- `ESP32/src/ShifterDetect.cpp`
- `ESP32/include/ShifterDetect.h`
- `ESP32/src/Main.cpp`
- `proto/diy_ffb_protocol.options`
- `SimHubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`
Open items:
- None.

## 2026-01-03 01:42:04 +01:00 (DESKTOP-6KO022D)
Request: increase joystick output to 8 buttons for shifter detection.
Summary:
- Firmware joystick now advertises 8 buttons and applies button states alongside axis outputs.
- Added CommManager helper to set button values for aux functions.
Key files:
- `ESP32/src/CommManager.cpp`
- `ESP32/include/CommManager.h`
Open items:
- None.

## 2026-01-02 21:40:09 +01:00 (DESKTOP-6KO022D)
Request: grounded pins enhanced using a ground-symbol.
Summary:
- Consolidated grounded-pin marker iterations into a single update for a mechanical ground symbol style.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:32:37 +01:00 (DESKTOP-6KO022D)
Request: make rail end caps more visible and add clearer grounded pin cues.
Summary:
- Rail end caps are now longer with a soft halo line behind them for visibility through pin overlays.
- Grounded pins now draw a diamond outline marker behind the pin dot for quick identification.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:27:40 +01:00 (DESKTOP-6KO022D)
Request: improve rail end caps visibility.
Summary:
- Replaced rail end dots with short vertical end cap lines for better contrast.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:22:55 +01:00 (DESKTOP-6KO022D)
Request: drop Config pose mode and improve the rail travel visualization.
Summary:
- Removed Config from pose mode selector; only Live and Test remain.
- Rail travel line now renders as a thicker track with a highlighted guide line and end caps.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:06:45 +01:00 (DESKTOP-6KO022D)
Request: apply 0.1mm resolution to all kinematic inputs.
Summary:
- Rail travel and test position inputs now round to 0.1mm; rail values normalize to non-negative.
- Test position wheel now supports Shift for 0.1mm steps; pin rounding already enforces 0.1mm.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 20:01:36 +01:00 (DESKTOP-6KO022D)
Request: limit pin X/Y resolution to 0.1mm.
Summary:
- Pin X/Y values now round to 0.1mm on update to keep stored coordinates at the desired resolution.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:56:01 +01:00 (DESKTOP-6KO022D)
Request: mouse wheel stepping for pin X/Y and rail inputs, with 0.1mm steps on shift.
Summary:
- Added mouse wheel adjustments on PinGrid X/Y cells and Rail +/- inputs (1mm steps, 0.1mm with Shift).
- Rail inputs clamp to >= 0; pin inputs apply deltas directly.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:30:58 +01:00 (DESKTOP-6KO022D)
Request: clamp Test pose position to the valid range and add mouse wheel 1mm/1N steps.
Summary:
- Test position now clamps to the cached contact range; switching to Test or rebuilding clamps it too.
- Added mouse wheel adjustments for test position/force (1mm/1N steps) and kept live update in Test mode.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:24:02 +01:00 (DESKTOP-6KO022D)
Request: fix NullReferenceException during plugin settings load in PoseModeCombo_SelectionChanged.
Summary:
- Guarded the pose mode selection handler during XAML initialization to avoid nulls and set isLoading before InitializeComponent.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:18:33 +01:00 (DESKTOP-6KO022D)
Request: add a Test pose mode with force and position inputs.
Summary:
- Added a Test pose mode with position/force inputs; the pose interpolation and force labels now use the selected mode.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:02:47 +01:00 (DESKTOP-6KO022D)
Request: center the contact force label above the arrow shaft.
Summary:
- Contact force label now anchors to the arrow shaft midpoint so it stays centered above the arrow.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 19:00:43 +01:00 (DESKTOP-6KO022D)
Request: move force labels above the arrow and sensor icon.
Summary:
- Contact force label now sits above the arrow tip; metered force label is centered above the sensor icon.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 18:41:28 +01:00 (DESKTOP-6KO022D)
Request: set the metering sensor icon size to 35x25 mm.
Summary:
- Updated the metering sensor icon to use fixed world dimensions of 35mm by 25mm (scaled by the canvas zoom).
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 18:15:04 +01:00 (DESKTOP-6KO022D)
Request: show contact force and measured force values next to the arrow and metering sensor.
Summary:
- Added force labels: contact point force from AxisState and measured force computed via the kinematic force factor.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 16:07:47 +01:00 (DESKTOP-6KO022D)
Request: align the contact force arrow with the contact point path direction.
Summary:
- Updated the contact arrow to follow the path tangent derived from the pose cache (positive travel direction) instead of bar geometry.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 16:02:17 +01:00 (DESKTOP-6KO022D)
Request: add a contact force arrow and a metering bar sensor icon in the kinematics view.
Summary:
- Added a contact point force arrow that points toward the contact pin based on nearby bar geometry.
- Added a stylized force sensor icon centered on the metering bar with rotation matching bar direction; ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 13:15:36 +01:00 (DESKTOP-6KO022D)
Request: reverse zoom direction, remove scale controls, and rename Recalc to Fit.
Summary:
- Reversed mouse wheel zoom direction and removed the scale buttons/label from the kinematics UI; renamed the Recalc button to Fit.
- Ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
Open items:
- None.

## 2026-01-02 13:07:39 +01:00 (DESKTOP-6KO022D)
Request: add zoom/pan to the general kinematics canvas and repurpose Recalc to reset view.
Summary:
- Added mouse wheel zoom and left-drag pan; updated scale limits and recalc button now resets the view to the auto-fit.
- Enabled canvas clipping so graphics do not draw outside the visualization area; ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 12:36:36 +01:00 (DESKTOP-6KO022D)
Request: add a migration check for a legacy DIY pedal config example.
Summary:
- Added a C# test that parses `axis1_diy_pedal_config.json` and verifies the DIY→General migration produces a valid kinematic config.
- Added `ESP32/sim/README.md` documenting the legacy example; ran Python and C# tests (17/18 tests, 0 failures).
Key files:
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
- `ESP32/sim/README.md`
- `ESP32/sim/axis1_diy_pedal_config.json`
- `ESP32/sim/test_general_kinematics.py`
Open items:
- None.

## 2026-01-02 12:20:43 +01:00 (DESKTOP-6KO022D)
Request: assume the rail is to the right in the DIY->General migration.
Summary:
- Updated the migration helper to place the rail interface pin at the minimum rail position and set travel as 0..stroke (no centering).
- Ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 12:16:24 +01:00 (DESKTOP-6KO022D)
Request: delete legacy DiyPedal kinematics files and add a migration helper.
Summary:
- Removed `DiyPedalKinematics` XAML/control files from the plugin; existing configs now migrate via a DIYPedal -> GeneralKinematic conversion.
- Conversion centers the rail travel, computes the link mount by circle intersection, and builds pedal + metering bars; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `SimhubPlugin/DiyPedalKinematics.xaml`
- `SimhubPlugin/DiyPedalKinematics.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 12:05:54 +01:00 (DESKTOP-6KO022D)
Request: remove DIY pedal kinematics, move general kinematics into the main tab control, and rename it to kinematics.
Summary:
- Removed the DIY pedal kinematics selector/control and placed the general kinematics UI as the first tab ("Kinematics").
- Default axis configs now build a basic GeneralKinematicConfig; DiyPedalKinematics removed from the SimHub plugin build.
- Renamed the GeneralKinematicsControl header label; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/AxisConfigControl.xaml`
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/DiyFfbPlugin.csproj`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:41:50 +01:00 (DESKTOP-6KO022D)
Request: add per-bar pin rings and color the bar list entries.
Summary:
- Added per-pin color rings for every bar (stacked when a pin belongs to multiple bars) and use the same palette to color bar outlines.
- Colored the Bar list "Pins" column to match bar colors; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:31:29 +01:00 (DESKTOP-6KO022D)
Request: emphasize zero axes and color bar outlines/first pins.
Summary:
- Added emphasized grid lines at x=0/y=0 and drew bar outlines with per-bar colors; first pin of each bar now gets a matching highlight ring.
- Bars now draw all pins (polyline closed for 3+ pins) instead of only farthest pair; ran Python and C# tests (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:21:01 +01:00 (DESKTOP-6KO022D)
Request: auto-fit the kinematic visualization to all poses.
Summary:
- Added auto-fit logic to compute scale/offset from pose bounds (or config fallback) and update on cache rebuild and canvas resize.
- Ran Python and C# test suites (17 tests each, 0 failures).
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:09:35 +01:00 (DESKTOP-6KO022D)
Request: run Python and C# tests after kinematics changes.
Summary:
- Python: `ESP32/.venv/Scripts/python.exe ESP32/sim/test_general_kinematics.py` (17 tests, 0 failures).
- C#: built `SimhubPlugin/DiyFfbPlugin.Tests` with `BuildProjectReferences=false` and ran the exe (17 tests, 0 failures); full plugin build via `dotnet run` still fails due to missing XAML-generated code in this environment.
Key files:
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
Open items:
- None.

## 2026-01-02 11:03:24 +01:00 (DESKTOP-6KO022D)
Request: add edge-case topology tests and verify extra collinear pins do not change polynomials.
Summary:
- Added Python/C# tests for unknown pin references, zero-length bars, and a regression check that adding a collinear bar pin leaves the polynomials unchanged.
Key files:
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`

## 2026-01-02 10:54:38 +01:00 (DESKTOP-6KO022D)
Request: represent rigid bars as pose variables (x,y,theta) with fixed pin offsets.
Summary:
- Non-collinear multi-pin bars now use pose variables with per-pin local offsets instead of rigid distance constraints in the Python and C# solvers.
- Updated bar position, Jacobian, and force projection math to use local offsets; the plot animation path uses the new constraint outputs.
Key files:
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/plot_kinematic_polynomials.py`

## 2026-01-02 03:54:48 +01:00 (DESKTOP-6KO022D)
Request: derive bar type from pin geometry (collinear vs rigid).
Summary:
- 3+ pin bars are now treated as collinear only if the pins lie on a line; otherwise they become rigid bars via distance constraints.
- Updated Python/C# solvers and tests for the new behavior.
Key files:
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
- `ESP32/sim/test_general_kinematics.py`

## 2026-01-02 02:33:29 +01:00 (DESKTOP-6KO022D)
Request: preserve GeneralKinematicConfig pins/bars when the FW returns axis config.
Summary:
- Implemented raw round-trip of axis_config protobuf payloads so ignored GeneralKinematicConfig.pins/bars are preserved on return.
- Cached raw axis_config bytes on update/load and used them when replying to return-axis-config; raw buffer is now dynamically sized.
Key files:
- `ESP32/src/ConfigManager.cpp`
- `ESP32/include/ConfigManager.h`
- `ESP32/src/CommManager.cpp`

## 2026-01-01 21:08:21 +01:00 (DESKTOP-6KO022D)
Request: fix missing bar segment in Config mode.
Summary:
- Config-mode bars now render using the farthest pin pair so collinear multi-pin bars show their full span.
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`

## 2026-01-01 20:59:13 +01:00 (DESKTOP-6KO022D)
Request: add a Live/Config toggle for the general kinematics canvas.
Summary:
- Added pose mode selector and update gating so the canvas can show config positions or live axis state.
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`

## 2026-01-01 20:27:14 +01:00 (DESKTOP-6KO022D)
Request: add an active switch between DIY pedal and general kinematics.
Summary:
- Added a kinematic model selector to AxisConfigControl and wired switching to preserve per-mode configs.
Key files:
- `SimhubPlugin/AxisConfigControl.xaml`
- `SimhubPlugin/AxisConfigControl.xaml.cs`

## 2026-01-01 19:58:42 +01:00 (DESKTOP-6KO022D)
Request: SimHub general kinematics UI with pin picker and cached pose interpolation.
Summary:
- Added GeneralKinematicsControl with pin/bar lists, pin picker context menu, rail travel inputs, and cached pose visualization driven by axis state.
- Added GeneralKinematics.BuildPoseCache for sample pose caching and wired AxisConfigControl to show the new control.
Key files:
- `SimhubPlugin/GeneralKinematicsControl.xaml`
- `SimhubPlugin/GeneralKinematicsControl.xaml.cs`
- `SimhubPlugin/GeneralKinematics.cs`
- `SimhubPlugin/AxisConfigControl.xaml`
- `SimhubPlugin/AxisConfigControl.xaml.cs`
- `SimhubPlugin/DiyFfbPlugin.csproj`

## 2026-01-01 19:04:06 +01:00 (DESKTOP-6KO022D)
Request: add more general kinematics test cases.
Summary:
- Expanded Python and C# tests to cover invalid configs (negative travel, duplicate pins, bad metering bars, non-collinear bars, shared collinear pins) and coefficient sanity checks.
Key files:
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`

## 2026-01-01 18:50:38 +01:00 (DESKTOP-6KO022D)
Request: tighten the general kinematics solvers.
Summary:
- Added adaptive LM damping, per-variable step caps, and Jacobian condition checks to both Python and C# solvers.
Key files:
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/GeneralKinematics.cs`

## 2026-01-01 18:09:50 +01:00 (DESKTOP-6KO022D)
Request: skip physics simulation when no function is active.
Summary:
- Physics loop now skips `sim.update` and holds the last contact position when `FunctionID_FUNCTION_ID_UNDEFINED`; sled target stays derived from the held contact position.
Key files:
- `ESP32/src/Main.cpp`

## 2026-01-01 17:54:14 +01:00 (DESKTOP-6KO022D)
Request: set firmware default function ID to undefined and guard SimHub for undefined active function.
Summary:
- Cleared default function-specific config so the firmware starts with no active function when `FunctionID_FUNCTION_ID_UNDEFINED`.
- SimHub now ignores undefined ActiveFunction updates and ignores undefined function configs without crashing.
Key files:
- `ESP32/src/ConfigManager.cpp`
- `ESP32/src/Main.cpp`
- `SimhubPlugin/DiyFfbPluginUI.xaml.cs`

## 2026-01-01 13:50:58 +01:00 (DESKTOP-6KO022D)
Request: set firmware default FunctionConfig function_id to undefined.
Summary:
- Changed default function_id to `FunctionID_FUNCTION_ID_UNDEFINED` in the ESP32 ConfigManager defaults.
Key files:
- `ESP32/src/ConfigManager.cpp`

## 2026-01-01 13:43:30 +01:00 (DESKTOP-6KO022D)
Request: implement option 2 collinear-bar handling for general kinematics.
Summary:
- Parameterized 3+ pin bars as rigid lines with bar pose variables in both Python and C# solvers; added fixed constraints for grounded/rail pins on those bars and updated force/Jacobian mapping.
- Updated the plotting script to use the new constraint API and draw collinear bars; added collinear-bar tests in Python and C#.
Key files:
- `ESP32/sim/general_kinematics.py`
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/plot_kinematic_polynomials.py`
- `ESP32/sim/test_general_kinematics.py`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`

## 2026-01-01 13:08:41 +01:00 (DESKTOP-6KO022D)
Request: fix convergence after adding another pin to the sample JSON and align protobuf outputs with updated field indices.
Summary:
- Adjusted the sample pin to avoid a collinear 3-pin bar (degenerate triangle).
- Regenerated `SimhubPlugin/DiyFfbProtocol.cs` and `ESP32/sim/diy_ffb_protocol_pb2.py` after GeneralKinematicConfig field index changes.
Key files:
- `ESP32/sim/sample_general_kinematic.json`
- `SimhubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`

## 2026-01-01 13:00:28 +01:00 (DESKTOP-6KO022D)
Request: add a sample GeneralKinematicConfig JSON file.
Summary:
- Added `ESP32/sim/sample_general_kinematic.json` using the new rail_travel_negative/positive fields.

## 2026-01-01 12:57:20 +01:00 (DESKTOP-6KO022D)
Request: replace symmetric rail travel with separate negative/positive distances from zero.
Summary:
- Schema: removed `rail_travel`, added `rail_travel_negative`/`rail_travel_positive` in `proto/diy_ffb_protocol.proto`, regenerated protobuf outputs.
- Solvers/tests/plot: updated C# and Python kinematics, tests, and plotting/animation script to use the new fields.
Key files:
- `proto/diy_ffb_protocol.proto`
- `SimhubPlugin/GeneralKinematics.cs`
- `ESP32/sim/general_kinematics.py`
- `ESP32/sim/test_general_kinematics.py`
- `ESP32/sim/plot_kinematic_polynomials.py`
- `SimhubPlugin/DiyFfbProtocol.cs`
- `ESP32/sim/diy_ffb_protocol_pb2.py`

## 2026-01-01 12:45:36 +01:00 (DESKTOP-6KO022D)
Request: add an animation of the kinematic layout to the plotting script.
Summary:
- Added layout animation support to the polynomial plotting script (with frame/interval options).
Key files:
- `ESP32/sim/plot_kinematic_polynomials.py`

## 2026-01-01 12:37:53 +01:00 (DESKTOP-6KO022D)
Request: add a Python script to plot the kinematic conversion polynomials.
Summary:
- Added a plotting helper that computes/loads KinematicParameters and plots sled/force conversion polynomials.
Key files:
- `ESP32/sim/plot_kinematic_polynomials.py`

## 2026-01-01 12:32:09 +01:00 (DESKTOP-6KO022D)
Request: add test cases, with C# coverage preferred.
Summary:
- Added a simple console-based C# test project covering GeneralKinematics centering and invalid config handling.
- Added a small Python test script mirroring the same checks.
Key files:
- `SimhubPlugin/DiyFfbPlugin.Tests/DiyFfbPlugin.Tests.csproj`
- `SimhubPlugin/DiyFfbPlugin.Tests/Program.cs`
- `ESP32/sim/test_general_kinematics.py`

## 2026-01-01 12:13:33 +01:00 (DESKTOP-6KO022D)
Request: make contact position zero at the initial pose (rail centered).
Summary:
- Shifted contact position by the center-rail pose using linear interpolation so zero aligns with rail centered.
Key files:
- `SimHubPlugin/GeneralKinematics.cs`
- `ESP32/sim/general_kinematics.py`
Open items:
- None.

## 2026-01-01 12:06:13 +01:00 (DESKTOP-6KO022D)
Request: add a general-purpose kinematic_config (pins/bars, rail travel, metering bar) with SimHub + Python solvers; have nanopb ignore pins/bars.
Summary:
- Protocol: added `GeneralKinematicPin`/`GeneralKinematicBar`/`GeneralKinematicConfig` (with `rail_travel`) and `AxisConfig.general_kinematic` in `proto/diy_ffb_protocol.proto`; nanopb options now ignore `GeneralKinematicConfig.pins` and `.bars` in `proto/diy_ffb_protocol.options`.
- SimHub: added `SimHubPlugin/GeneralKinematics.cs` solver (constraint solve + polynomial fit), wired `SimHubPlugin/AxisConfigControl.xaml.cs`, updated `SimHubPlugin/DiyFfbPlugin.csproj`, regenerated `SimHubPlugin/DiyFfbProtocol.cs`.
- Python: added `ESP32/sim/general_kinematics.py`, regenerated `ESP32/sim/diy_ffb_protocol_pb2.py`.
Notes:
- No tests run.

## 2025-12-31 15:30:33 +01:00 (DESKTOP-6KO022D)
Request: add dedicated FlightStickPitch/FlightStickRoll configs and matching sim scripts.
Summary:
- Protocol: added FunctionID FLIGHT_STICK_PITCH/ROLL and new FlightStickPitchConfig/FlightStickRollConfig in `proto/diy_ffb_protocol.proto`, with sizing hints in `proto/diy_ffb_protocol.options`.
- Firmware: added `ESP32/include/FlightStickFunction.h` and `ESP32/src/FlightStickFunction.cpp`, wired in `ESP32/src/Main.cpp`.
- SimHub: added `SimHubPlugin/FlightStickConfigControl.xaml` and `.xaml.cs`, wired in `SimHubPlugin/FunctionConfigControl.xaml` and `.xaml.cs`, extended function list in `SimHubPlugin/DiyFfbPluginUI.xaml.cs`, regenerated `SimHubPlugin/DiyFfbProtocol.cs`.
- Sim scripts: added `ESP32/sim/flight_stick_pitch.py` and `ESP32/sim/flight_stick_roll.py`; updated brake/accelerator sim scripts to *_PEDAL enum names; regenerated `ESP32/sim/diy_ffb_protocol_pb2.py` (untracked).
Defaults:
- Stick range -50..50, damping 0.5, centering 1.5, output mode TRAVEL.
- Pitch controller axis Y, roll controller axis X.
Axis IDs:
- `ESP32/sim/flight_stick_pitch.py` uses AXIS_ID_4.
- `ESP32/sim/flight_stick_roll.py` uses AXIS_ID_5.
Notes:
- No tests run.
- Unrelated modified/untracked files existed (DLL/PDB and various `DIY-FFB.srctrl*`/`ESP32/*` files), intentionally ignored.
