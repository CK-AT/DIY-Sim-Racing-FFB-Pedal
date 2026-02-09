# Handoff — Tiered Config Overrides

Branch: `ck_tiered_config`

## Uncommitted Work

Plan 30 Phase 1 (registry consolidation) — ready to commit:
- `DiyFfbPlugin.cs`: Replaced `ClearOverrideFieldValue` 70-line switch with `OverrideFieldRegistry.ClearValue` delegation
- `DiyFfbPlugin.cs`: Replaced `NormalizeFunctionOverrideFieldPath` 13-line switch with `OverrideFieldRegistry.NormalizeFieldPath`
- `DiyFfbPlugin.cs`: Extracted `ReapplyMergedOverrides(functionId, diffCheck)` method, replaced 5 duplicated gather-deltas-then-apply patterns
- `DiyFfbPlugin.cs`: Replaced 6 `new JsonParser`/`new JsonFormatter` with shared `ProtobufJsonHelper` instances
- `ConfigLayerProvider.cs`: Replaced `HasFieldValue` 70-line switch with `OverrideFieldRegistry.HasValue` delegation

Planning docs (not committed, not code changes):
- `SimHubPlugin/Docs/plans/30_Duplication_Cleanup.md` — 5-phase plan to eliminate ~810 lines of duplicated code
- `SimHubPlugin/Docs/plans/31_TieredConfig_Orchestrator_Extraction.md` — 6-phase plan to extract TieredConfigOrchestrator from the 4,685-line DiyFfbPlugin god class

## Next Steps

### Plan 30: Duplication Cleanup (phases 2-5 remaining)

| Phase | What | Effort | Lines saved | Status |
|-------|------|--------|-------------|--------|
| 1 | Registry consolidation | ~2h | ~180 | **DONE** |
| 2 | Badge infrastructure — extract identical code from 5 controls into `BadgeHelper` | ~3h | ~200 | |
| 3 | Graph parameter UI — extract from 2 controls into `GraphParamHelper` | ~3h | ~200 | |
| 4 | Kinematic bounds + travel display — extract from 3 controls into helpers | ~3h | ~150 | |
| 5 | FlightStick mode-dispatch — collapse 8 accessor methods to 1 | ~1h | ~80 | |

Phase 1 complete — **Plan 31 is now unblocked**.

### Plan 31: TieredConfigOrchestrator Extraction (do after Plan 30 phase 1)

| Phase | What |
|-------|------|
| 1 | Create orchestrator shell, move 12 baseline + initialization methods |
| 2 | Move 10 override application + activity query methods |
| 3 | Move 15 override field operations + user preference methods + events |
| 4 | Move 9 axis parameter override API methods |
| 5 | Move ESP32 authority logic from DiyFfbPluginUI into orchestrator |
| 6 | Replace forwarding methods with direct `plugin.ConfigOrchestrator.X()` calls |

## Completed Work

All core tiered config phases are implemented and committed:

| Commit | Description |
|--------|-------------|
| `997cae87` | Extract ShifterProcessor from ConfigMerger |
| `0dfce9ed` | Extract FlightStickProcessor from ConfigMerger |
| `f6d42b8d` | Extract FlightPedalsProcessor from ConfigMerger |
| `e12f881a` | Extract AutomotivePedalProcessor from ConfigMerger |
| `19eecaea` | Auto-assign graph template for single-match games |
| `c9a62601` | Fix batched config uploads, derived fields, and profile dirty detection |
| `18ebc686` | Gate automatic config uploads on active function status |
| `37e77561` | Ignore ESP32 configs when stored baselines exist; plugin pushes its merged config back |
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

### Summary of what's in place

- **Tiered config system** — Baseline -> Profile -> User override layers for function configs
- **Derived field reconciliation** — `ConfigMerger.ReconcileDerivedFields` delegates to per-type processors
- **All 4 function processor extractions complete** — AutomotivePedal, FlightPedals, FlightStick, Shifter
- **Axis parameter overrides** — Per-function kinematics/static-balance overrides, plugin-side merge
- **Baseline persistence** — Both function and axis baselines survive ESP32 reconnects and restarts
- **Clear Baseline** — Buttons to remove stored baseline; overrides preserved, re-apply on next baseline
- **Override badges** — `[F]` markers on function selector items with existing overrides
- **User profile header** — Always-visible user profile ComboBox above all tabs
- **Auto-assign template** — Single-match games get template assigned automatically
- **230 unit tests** — All processors, ConfigMerger, ConfigComparer, ConflictDetector, OverrideFieldRegistry, FunctionConfigManager, ChangeTracker, FieldRouter

## Key Architecture Notes

- `function.Config` is the UI's working copy; `FunctionConfigManager._currentConfigs` is the authoritative merged config
- `_lastSentConfigs` is ONLY updated via `MarkAsSent()` in UI event handlers, never in FunctionConfigManager event-firing methods
- Processors are called by both ConfigMerger (backend) and UI controls, eliminating duplicated derived-field logic
- ESP32 config authority: when stored baselines exist, incoming ESP32 configs are ignored and merged config is pushed back
- Batched clear+apply avoids 2N upload race condition
- Axis overrides use full replacement (not field-level merge)

## References

- **Duplication cleanup plan**: `SimHubPlugin/Docs/plans/30_Duplication_Cleanup.md`
- **Orchestrator extraction plan**: `SimHubPlugin/Docs/plans/31_TieredConfig_Orchestrator_Extraction.md`
- **Function Processors plan**: `SimHubPlugin/Docs/plans/29_Function_Processors.md`
- **Clear Baseline plan**: `SimHubPlugin/Docs/plans/28_Clear_Baseline.md`
- **Resilience plan**: `SimHubPlugin/Docs/plans/27_AircraftFfbProfiles_Resilience.md`
- **Tiered config design**: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
- **Memory**: `C:\Users\Christian\.claude\projects\d--Projects-DIY-Sim-Racing-FFB-Pedal\memory\MEMORY.md`
