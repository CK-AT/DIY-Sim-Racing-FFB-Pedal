# Handoff — Tiered Config Overrides

Branch: `ck_tiered_config`

## Current State

All planned work is complete. The branch implements a full tiered config system
and two major refactoring plans on top of it.

## What's In Place

### Tiered Config System

- **Baseline → Profile → User override** layers for function configs
- **Derived field reconciliation** — `ConfigMerger.ReconcileDerivedFields` delegates to per-type processors
- **ESP32 config authority** — when stored baselines exist, incoming ESP32 configs are ignored and merged config is pushed back
- **Batched clear+apply** — silent clear + batched flush avoids 2N upload race condition
- **Axis parameter overrides** — per-function kinematics/static-balance overrides, plugin-side merge
- **Baseline persistence** — both function and axis baselines survive ESP32 reconnects
- **Clear Baseline buttons** — remove stored baseline; overrides preserved, re-apply on next baseline
- **Override badges** — `[F]` markers on function selector items with existing overrides
- **User profile header** — always-visible user profile ComboBox above all tabs
- **Auto-assign template** — single-match games get template assigned automatically
- **230 unit tests** — all processors, ConfigMerger, ConfigComparer, ConflictDetector, OverrideFieldRegistry, FunctionConfigManager, ChangeTracker, FieldRouter

### Plan 29: Function Processor Extractions — COMPLETE

Four per-type processors extracted from `ConfigMerger`:
AutomotivePedalProcessor, FlightPedalsProcessor, FlightStickProcessor, ShifterProcessor.

### Plan 30: Duplication Cleanup — COMPLETE

~810 lines eliminated across 5 phases: registry consolidation, BadgeHelper,
GraphParamHelper, KinematicBoundsHelper/TravelDisplayHelper, and FlightStick
interface dispatch (`IFlightStickSubConfig`).

### Plan 31: TieredConfigOrchestrator Extraction — COMPLETE

All 38 config orchestration methods extracted from `DiyFfbPlugin.cs` into
`TieredConfigOrchestrator` (1,185 lines). UI accesses via
`plugin.ConfigOrchestrator.X()`. DiyFfbPlugin.cs reduced from 4,685 to 3,556
lines (-1,129).

## Commit History

| Commit | Description |
|--------|-------------|
| `0ca56e41` | Replace forwarding methods with direct orchestrator access (Plan 31, Phase 6) |
| `4de3ec1c` | Move ESP32 authority logic into orchestrator (Plan 31, Phase 5) |
| `27039a51` | Move axis parameter override API to orchestrator (Plan 31, Phase 4) |
| `6e9af144` | Move override field ops, user prefs, and events to orchestrator (Plan 31, Phase 3) |
| `f41320b1` | Extract TieredConfigOrchestrator (Plan 31, Phases 1-2) |
| `1e3e3270` | Collapse FlightStick 8 mode-dispatch accessors to interface |
| `dee6284c` | Extract KinematicBoundsHelper and TravelDisplayHelper |
| `ac02c520` | Extract GraphParamHelper from 2 controls |
| `5eaaac83` | Extract BadgeHelper from 5 controls |
| `2a0f0994` | Consolidate registry, extract ReapplyMergedOverrides |
| `997cae87` | Extract ShifterProcessor from ConfigMerger |
| `0dfce9ed` | Extract FlightStickProcessor from ConfigMerger |
| `f6d42b8d` | Extract FlightPedalsProcessor from ConfigMerger |
| `e12f881a` | Extract AutomotivePedalProcessor from ConfigMerger |
| `19eecaea` | Auto-assign graph template for single-match games |
| `c9a62601` | Fix batched config uploads, derived fields, and profile dirty detection |
| `18ebc686` | Gate automatic config uploads on active function status |
| `37e77561` | Ignore ESP32 configs when stored baselines exist |
| `b1c12b58` | Fix user overrides lost on switch, vehicle label, and stale labels |
| `0bb69b4a` | Add Clear Baseline plan doc (completed) |
| `666e1faa` | Layout tweaks for user profile header and updated docs |
| `56914553` | Promote user profile selector to always-visible header |
| `15aefff8` | Add Clear Baseline buttons for function and axis configs |
| `fb34a0cf` | Remove obsolete override editor from Active Functions panel |
| `008d31aa` | Fix user switch not updating force curves in UI |
| `78164ac1` | Fix axis baseline init, import-to-override, and selector memory |
| `27873372` | Fix config import, vehicle label, and user-switch bugs |
| `ae699b2c` | Fix protobuf/JSON.NET data loss and profile save gaps |
| `08ae9199` | Move axis function selector to parent UI |
| `1dd30987` | Persist axis baselines across ESP32 reconnects |
| `45e3812d` | Fix config corruption during function view switch |
| `a82dbb94` | Snapshot baseline geometry before applying axis overrides |
| `8bd2f9a0` | Rename ConfigLayer.Hardware to Baseline everywhere |
| `0b8bbd75` | Wire force curve override: store, badge, and clear |
| `3857117e` | Force ownerless modal dialogs to foreground, add resilience plan |
| `58a99583` | Wire shifter overrides, fix label init, restructure panels |

## Key Architecture Notes

- `function.Config` is the UI's working copy; `FunctionConfigManager._currentConfigs` is the authoritative merged config
- `_lastSentConfigs` is ONLY updated via `MarkAsSent()` in UI event handlers, never in FunctionConfigManager event-firing methods
- Processors are called by both ConfigMerger (backend) and UI controls, eliminating duplicated derived-field logic
- Batched clear+apply avoids 2N upload race condition
- Axis overrides use full replacement (not field-level merge)
- `IFlightStickSubConfig` — interface over 3 protobuf types via partial classes; `GetActiveSubConfig()` dispatches once on mode

## References

- **Orchestrator extraction plan**: `SimHubPlugin/Docs/plans/31_TieredConfig_Orchestrator_Extraction.md`
- **Duplication cleanup plan**: `SimHubPlugin/Docs/plans/30_Duplication_Cleanup.md`
- **Function Processors plan**: `SimHubPlugin/Docs/plans/29_Function_Processors.md`
- **Clear Baseline plan**: `SimHubPlugin/Docs/plans/28_Clear_Baseline.md`
- **Tiered config design**: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
- **Plugin design**: `SimHubPlugin/Docs/Plugin_Design.md`
