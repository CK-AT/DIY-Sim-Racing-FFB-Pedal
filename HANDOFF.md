# Handoff — Tiered Config Overrides

Branch: `ck_tiered_config`

## Current State

All 35 plans analyzed and verified against codebase (2026-02-10). 33 of 35 plans
complete. Build passes, 252/252 tests pass.

## Last Completed: Plan 33 — FlightStick Config Consolidation

**Plan doc:** `docs/plans/02_FlightStick_Config_Consolidation.md`

Replaced 3 identical protobuf messages (`FlightStickPitchConfig`, `FlightStickRollConfig`,
`FlightStickCollectiveConfig`) with single `FlightStickConfig`. Differentiate by
`FunctionID` alone. FunctionID values (Pitch=5, Roll=6, Collective=8) unchanged.

**Status:** All 6 phases complete, build passes, 252/252 tests pass.

---

## Open Points

### Deferred / Not Implemented

1. **Plan 07 — Grid auto-centering** (`SimHubPlugin/Docs/plans/07_FFB_Graph_Grid_Centering.md`)
   - Status: DEFERRED. Auto-centering caused pan/zoom regressions when implemented.
   - Zoom-to-fit exists as manual context-menu action. Grid works as background brush.
   - Open questions: padding value (200px?), whether zoom-to-fit should be a toolbar button.

2. **Progressive Spring** (`docs/plans/01_progressive-spring.md`)
   - Status: NOT IMPLEMENTED. Complete plan exists but zero code written.
   - Spans all layers: proto (`FlightFfbAction.spring_exponent`), plugin
     (`SpringExponent` graph outputs), ESP32 physics (`F = -k * sign(x) * |x|^n`),
     CAN transport (new payload field).
   - No dependencies on other plans.

### Follow-up / Technical Debt

3. ~~**Plan 27 follow-up — `AxisParameterOverrides` protobuf-in-JSON.NET risk**~~
   **RESOLVED.** Fix was already in place (`[JsonIgnore]` + `*Json` companion properties).
   Added 7 round-trip serialization tests in `ProtobufJsonSerializationTests.cs` covering
   `AxisParameterOverrides` (Kinematics, StaticBalance) and `FunctionConfigOverrides`
   (ForceCurve, ShifterConfig, ShifterDetectConfig).

4. **Plan 33 — Manual verification still needed**
   - Launch SimHub, verify flight stick config tab loads for Pitch/Roll/Collective
   - Verify existing baselines load after migration
   - ESP32 `pio build` (requires PlatformIO)

### Minor Polish (low priority)

5. **Plan 06 — Graph Editor Tabs polish**
   - Ctrl+W keyboard shortcut for close tab (not wired)
   - Tab overflow: scroll or dropdown when many tabs open
   - Drag-to-reorder tabs
   - Tab context menu: Close, Close Others, Close All

6. **Plan 12 — Inspector signal picker**
   - Uses flat ComboBox list; hierarchical signal picker deferred.

7. **Plan 18 — Profile Browser enhancements**
   - Replace `PromptForGraphTemplate()` with browser in NewVehicle mode
   - Add search/filter for large profile lists
   - Add `LastUsed` timestamp to `AircraftFfbProfile` for sorting

8. **Plan 17 #5 — Game-specific fields**
   - Only `XPlaneRotorIndex` exists in `AircraftFfbProfile`. Monitor for more
     game-specific settings. Current approach (add fields directly) is fine until
     there are 3+ such fields.

---

## What's In Place

### Tiered Config System (Plans 24-33)

- **Baseline → Profile → User override** layers for function configs
- **Derived field reconciliation** — `ConfigMerger.ReconcileDerivedFields` delegates to per-type processors
- **ESP32 config authority** — when stored baselines exist, incoming ESP32 configs are ignored and merged config is pushed back
- **Batched clear+apply** — silent clear + batched flush avoids 2N upload race condition
- **Axis parameter overrides** — per-function kinematics/static-balance overrides, plugin-side merge
- **Baseline persistence** — both function and axis baselines survive ESP32 reconnects
- **Clear Baseline buttons** — remove stored baseline; overrides preserved, re-apply on next baseline
- **Override badges** — `[U]`/`[P]` markers with context menu (Move to layer, Save to Baseline, Clear)
- **Override Review dialog** — standalone dialog showing all active overrides with Move/Bake/Clear actions
- **User profile header** — always-visible user profile ComboBox above all tabs
- **Auto-assign template** — single-match games get template assigned automatically
- **All event handlers migrated** to `UpdateFunctionOverrideField()` pattern
- **TieredConfigOrchestrator** — extracted from DiyFfbPlugin, owns merge-and-apply lifecycle
- **4 function processors** — AutomotivePedal, FlightPedals, FlightStick, Shifter
- **Single FlightStickConfig** proto type — no more IFlightStickSubConfig or per-axis dispatch
- **AircraftFfbProfiles resilience** — backup/restore safety net, [JsonIgnore] on protobuf fields
- **252 unit tests** — processors, ConfigMerger, ConfigComparer, ConflictDetector, OverrideFieldRegistry, FunctionConfigManager, ChangeTracker, FieldRouter, OrchestratorReroute, migration

### Graph Editor System (Plans 1-15, 19-21)

- **FFB graph templates** — plane (3 axes) and heli (4 axes) with Include composition
- **Graph editor tabs** — multi-tab editing with pinned active graph, dirty tracking, undo/redo
- **Copy/paste** — Ctrl+C/V/X with Param and Include node handling
- **Include context preview** — live preview using parent graph inputs, auto-select on double-click
- **Nested include resolution** — correct path resolution for deeply nested graphs
- **Inspector per-node templates** — dedicated views for each node type
- **Op input negate flags** — per-input negation for Add/Mul ops
- **Shared graph save protection** — warns when saving graphs used by multiple vehicles
- **Themed MessageBox** — dark-themed replacement for all MessageBox.Show calls
- **Legacy XPlane FFB removed** — graph runtime is the only FFB source

### Vehicle Profile System (Plans 16-18, 20, 23)

- **Profile Browser dialog** — unified template/profile/import UI
- **Staged imports** — imported profiles held in staging area with Save to Library
- **3-tier param resolution** — param default → graph ParamValues → profile overrides
- **Graph change detection** — hash-based, with param review window
- **Vehicle tab** — central UI for all non-System params, graph-layout ordering
- **FFB Graph sub-tab removed** — consolidated into Vehicle tab

## Key Architecture Notes

- `function.Config` is the UI's working copy; `FunctionConfigManager._currentConfigs` is the authoritative merged config
- `_lastSentConfigs` is ONLY updated via `MarkAsSent()` in UI event handlers, never in FunctionConfigManager event-firing methods
- Processors are called by both ConfigMerger (backend) and UI controls, eliminating duplicated derived-field logic
- Batched clear+apply avoids 2N upload race condition
- Axis overrides use full replacement (not field-level merge)
- Single `FlightStickConfig` type used everywhere — no more `IFlightStickSubConfig` or per-axis dispatch

## Plan Index

All plans with current status:

| # | Plan | Location | Status |
|---|------|----------|--------|
| 01 | Vehicle Tab | `SimHubPlugin/Docs/plans/01_Vehicle_Tab_Plan.md` | Complete |
| 02 | FFB Graph Template Rework | `SimHubPlugin/Docs/plans/02_FFB_Graph_Template_Rework_Plan.md` | Complete |
| 03 | Nested Include Fix | `SimHubPlugin/Docs/plans/03_Nested_Include_Fix_Plan.md` | Complete |
| 04 | Include Preview Debug | `SimHubPlugin/Docs/plans/04_Include_Preview_Debug_Plan.md` | Resolved |
| 05 | Include Context Preview | `SimHubPlugin/Docs/plans/05_Include_Context_Preview_Plan.md` | Complete |
| 06 | FFB Graph Editor Tabs | `SimHubPlugin/Docs/plans/06_FFB_Graph_Editor_Tabs.md` | Complete (polish remaining) |
| 07 | FFB Graph Grid Centering | `SimHubPlugin/Docs/plans/07_FFB_Graph_Grid_Centering.md` | **DEFERRED** |
| 08 | Include Context Auto-Select | `SimHubPlugin/Docs/plans/08_Include_Context_Auto_Select_Plan.md` | Complete |
| 09 | FFB Graph Copy/Paste | `SimHubPlugin/Docs/plans/09_FFB_Graph_CopyPaste_Plan.md` | Complete |
| 10 | Graph Editor Undo/Redo | `SimHubPlugin/Docs/plans/10_Graph_Editor_Undo_Redo_Plan.md` | Complete |
| 11 | Param Control Layout | `SimHubPlugin/Docs/plans/11_Param_Control_Layout_Restructure_Plan.md` | Complete |
| 12 | Inspector Panel Restructure | `SimHubPlugin/Docs/plans/12_Inspector_Panel_Restructure_Plan.md` | Complete |
| 13 | Op Input Negate Flags | `SimHubPlugin/Docs/plans/13_Op_Input_Negate_Flags_Plan.md` | Complete |
| 14 | Legacy XPlane FFB Removal | `SimHubPlugin/Docs/plans/14_Legacy_XPlane_FFB_Removal_Plan.md` | Complete |
| 15 | Graph Param Override Migration | `SimHubPlugin/Docs/plans/15_Graph_Param_Override_Migration_Plan.md` | Complete |
| 16 | Vehicle Profile Lifecycle | `SimHubPlugin/Docs/plans/16_Vehicle_Profile_Lifecycle.md` | Reference doc |
| 17 | Profile System Improvements | `SimHubPlugin/Docs/plans/17_Profile_System_Improvements.md` | Complete |
| 18 | Unified Profile Browser | `SimHubPlugin/Docs/plans/18_Unified_Profile_Browser.md` | Complete |
| 19 | Shared Graph Save Protection | `SimHubPlugin/Docs/plans/19_Shared_Graph_Save_Protection.md` | Complete |
| 20 | FFB Graph Tab Removal | `SimHubPlugin/Docs/plans/20_FFB_Graph_Tab_Removal.md` | Complete |
| 21 | Themed MessageBox | `SimHubPlugin/Docs/plans/21_Themed_MessageBox_Plan.md` | Complete |
| 22 | Unit Test Expansion | `SimHubPlugin/Docs/plans/22_Unit_Test_Expansion_Plan.md` | Complete (245 tests) |
| 23 | Staged Imports | `SimHubPlugin/Docs/plans/23_Staged_Imports_Plan.md` | Complete |
| 24 | Tiered Config Overrides | `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md` | Complete |
| 25 | Override Field Registry | `SimHubPlugin/Docs/plans/25_Override_Field_Registry_Plan.md` | Complete (all 15 phases) |
| 26 | Motion Range Migration HOWTO | `SimHubPlugin/Docs/plans/26_Motion_Range_Migration_HOWTO.md` | Reference doc |
| 27 | AircraftFfbProfiles Resilience | `SimHubPlugin/Docs/plans/27_AircraftFfbProfiles_Resilience.md` | Complete (follow-up pending) |
| 28 | Clear Baseline | `SimHubPlugin/Docs/plans/28_Clear_Baseline.md` | Complete |
| 29 | Function Processors | `SimHubPlugin/Docs/plans/29_Function_Processors.md` | Complete |
| 30 | Duplication Cleanup | `SimHubPlugin/Docs/plans/30_Duplication_Cleanup.md` | Complete |
| 31 | Orchestrator Extraction | `SimHubPlugin/Docs/plans/31_TieredConfig_Orchestrator_Extraction.md` | Complete |
| 32 | Badge Menu & Review Dialog | `SimHubPlugin/Docs/plans/32_Badge_Menu_And_Review_Dialog.md` | Complete |
| 33 | FlightStick Consolidation | `docs/plans/02_FlightStick_Config_Consolidation.md` | Complete |
| ESP32-01 | A6 Servo Error Logging | `ESP32/docs/plans/01_a6-servo-error-logging.md` | Complete |
| docs-01 | Progressive Spring | `docs/plans/01_progressive-spring.md` | **NOT IMPLEMENTED** |
