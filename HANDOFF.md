# Handoff — Tiered Config Overrides

Branch: `ck_tiered_config`

## Uncommitted Work

None.

## Completed Work

All core tiered config phases are implemented and committed:

| Commit | Description |
|--------|-------------|
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
- **Axis parameter overrides** — Per-function kinematics/static-balance overrides, plugin-side merge
- **Baseline persistence** — Both function and axis baselines survive ESP32 reconnects and restarts
- **Clear Baseline** — Buttons to remove stored baseline; overrides preserved, re-apply on next baseline
- **Axis function selector** — Dropdown in parent UI to switch between baseline and function override editing
- **Override badges** — `[F]` markers on function selector items with existing overrides
- **User profile header** — Always-visible user profile ComboBox above all tabs for quick switching
- **172 unit tests** — ConfigMerger, ConfigComparer, ConflictDetector, OverrideFieldRegistry, FunctionConfigManager, ChangeTracker, FieldRouter

## Key Architecture Notes

- `config` in `AxisConfigControl` IS `axis.Config` (same reference, set at `LoadConfigIntoUi`)
- `AxisConfigManager.GetBaseConfig()` is the authoritative baseline; `_baselineXxx` fields are fallback before first ESP32 config
- `OnAxisConfigUpdate` routes through `AxisConfigManager.SetBaseConfig()` and re-applies active function overrides
- Axis overrides use full replacement (not field-level merge) for kinematics and static balance
- `AxisConfigControl` exposes `SwitchToBaseline()`, `SwitchToFunction()`, `ClearCurrentOverride()`, `OverrideChanged` event — parent UI owns the selector
- `UpdateAxisSelection` saves/restores per-axis function selection via `_lastSelectedFunctionPerAxis`
- User profile header ComboBox (`ComboBox_UserProfileHeader`) syncs with SYSTEM > User tab ComboBox via shared `RefreshUserProfileUi()` and `suppressUserProfileSelectionChange` flag

## References

- **Clear Baseline plan**: `SimHubPlugin/Docs/plans/28_Clear_Baseline.md`
- **Resilience plan**: `SimHubPlugin/Docs/plans/27_AircraftFfbProfiles_Resilience.md`
- **Tiered config design**: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
- **Memory**: `C:\Users\Christian\.claude\projects\d--Projects-DIY-Sim-Racing-FFB-Pedal\memory\MEMORY.md`
