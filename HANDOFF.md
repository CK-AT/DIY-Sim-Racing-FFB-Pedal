# Handoff — Tiered Config Overrides

Branch: `ck_tiered_config`

## Uncommitted Work

**AutomotivePedalProcessor extraction (Phase 1)** — ready to commit:

- `SimHubPlugin/TieredConfig/AutomotivePedalProcessor.cs` — new processor class
- `SimHubPlugin/TieredConfig/ConfigMerger.cs` — delegates to processor, 2 private methods deleted
- `SimHubPlugin/AutomotivePedalConfigControl.xaml.cs` — uses processor for derived fields
- `SimHubPlugin/TieredConfigTests/AutomotivePedalProcessorTests.cs` — 12 new tests
- Both `.csproj` files and `Program.cs` updated

All 184 tests pass (12 new + 172 existing).

Also uncommitted:
- `SimHubPlugin/Docs/plans/29_Function_Processors.md` — plan doc

## Completed Work

All core tiered config phases are implemented and committed:

| Commit | Description |
|--------|-------------|
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
- **AutomotivePedalProcessor** — single source of truth for pedal derived fields and overrides
- **Axis parameter overrides** — Per-function kinematics/static-balance overrides, plugin-side merge
- **Baseline persistence** — Both function and axis baselines survive ESP32 reconnects and restarts
- **Clear Baseline** — Buttons to remove stored baseline; overrides preserved, re-apply on next baseline
- **Axis function selector** — Dropdown in parent UI to switch between baseline and function override editing
- **Override badges** — `[F]` markers on function selector items with existing overrides
- **User profile header** — Always-visible user profile ComboBox above all tabs for quick switching
- **Auto-assign template** — Single-match games (automotive) get template assigned automatically; seeds ActiveFunctionIds from category defaults; refreshes active-function checkboxes
- **184 unit tests** — AutomotivePedalProcessor, ConfigMerger, ConfigComparer, ConflictDetector, OverrideFieldRegistry, FunctionConfigManager, ChangeTracker, FieldRouter

## Key Architecture Notes

- `function.Config` is the UI's working copy; `FunctionConfigManager._currentConfigs` is the authoritative merged config
- `_lastSentConfigs` is ONLY updated via `MarkAsSent()` in UI event handlers (after actual enqueue), never in FunctionConfigManager event-firing methods
- `ConfigMerger.ReconcileDerivedFields` runs at the end of every `MergeFunctionConfig` call — both profile and user override layers get reconciled
- **AutomotivePedalProcessor**: `ReconcileDerivedFields` and `ApplyOverrides` are called by both ConfigMerger (backend) and AutomotivePedalConfigControl (UI), eliminating duplicated logic
- **ESP32 config authority**: When stored baselines exist, `OnFunctionConfigUpdate` and `OnAxisConfigUpdate` ignore incoming ESP32 configs and push the plugin's merged config back
- **Active-function upload gate**: `OnMergedFunctionConfigChanged` only uploads to ESP32 if `IsFunctionActive()` returns true. `InvalidateLastSent()` clears diff-check tracking so next activation re-sends.
- **Batched clear+apply**: `ClearAllProfileOverrides(fireEvents: false)` silently resets, then applies fire events per function, then `SendAllPendingChanges()` flushes cleared-but-not-reapplied functions
- **Auto-assign flow**: `ResolveActiveGraph` checks `GraphTemplateRegistry.GetTemplates` — if exactly 1 match, stores GraphPath and seeds `ActiveFunctionIds` via `SeedDefaultActiveFunctionIds` after graph load. Multiple matches (flight sims) still prompt.
- Axis overrides use full replacement (not field-level merge) for kinematics and static balance
- User profile header ComboBox syncs with SYSTEM > User tab ComboBox via shared `RefreshUserProfileUi()` and `suppressUserProfileSelectionChange` flag

## Next Up

**Function Processor extraction — remaining phases** (plan doc: `SimHubPlugin/Docs/plans/29_Function_Processors.md`)

| Phase | Processor | Status |
|-------|-----------|--------|
| 1 | `AutomotivePedalProcessor` | Done (uncommitted) |
| 2 | `FlightPedalsProcessor` | Pending |
| 3 | `FlightStickProcessor` | Pending |
| 4 | `ShifterProcessor` | Pending |

## References

- **Clear Baseline plan**: `SimHubPlugin/Docs/plans/28_Clear_Baseline.md`
- **Function Processors plan**: `SimHubPlugin/Docs/plans/29_Function_Processors.md`
- **Resilience plan**: `SimHubPlugin/Docs/plans/27_AircraftFfbProfiles_Resilience.md`
- **Tiered config design**: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
- **Memory**: `C:\Users\Christian\.claude\projects\d--Projects-DIY-Sim-Racing-FFB-Pedal\memory\MEMORY.md`
