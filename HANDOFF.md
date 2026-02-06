# Override Field Registry — Shifter Bugfixes

Branch: `ck_tiered_config`
Last commit: `481fb85c` — Migrate FlightStick & FlightPedals overrides, fix badge clipping

## Session Summary (2026-02-06 cont.) — Shifter Label Fix

### Bug Fixed

#### Shifter tab labels wrong on startup

**ShifterConfigControl.xaml.cs** `SwitchFunction()`: Sliders for friction, simulated_mass, and damping were set while `isUpdating = true`, so their `ValueChanged` handlers returned early without updating labels. Added explicit label updates after slider assignments (same pattern FlightPedalsConfigControl uses).

### Verified Working

- **shifter_config badge + persistence**: Works correctly on Profile layer when a sim is running (requires `activeCarId`). Previous false alarm was caused by testing with a stale User-layer build.

### Files Changed (NOT committed)

- **ShifterConfigControl.xaml.cs** — label updates in SwitchFunction + previous session changes
- **FunctionConfigControl.xaml** — badge placement fix (prev session)
- **ShifterConfigControl.xaml** — panel restructure (prev session)
- **AutomotivePedalConfigControl.xaml.cs** — override handlers (prev session)
- **FunctionConfigControl.xaml.cs** — override handlers (prev session)
- **TieredConfig/TieredConfigTypes.cs** — ShifterDetectConfig property (prev session)
- **TieredConfig/OverrideFieldRegistry.cs** — HasValue/ClearValue updated (prev session)
- **TieredConfig/ConfigMerger.cs** — ShifterDetectConfig merge (prev session)
- **TieredConfig/ConfigLayerProvider.cs** — HasFieldValue updated (prev session)
- **DiyFfbPlugin.cs** — ClearOverrideFieldValue updated (prev session)

### Build

Build: **Success** (warnings only, no errors)

### Next Steps

1. **Commit** — all changed files
2. **Initial slider bounds bug** — still open (range slider bounds wrong on initial load)

## References

- **Migration HOWTO**: `SimHubPlugin/Docs/plans/26_Motion_Range_Migration_HOWTO.md`
- **Full plan**: `SimHubPlugin/Docs/plans/25_Override_Field_Registry_Plan.md`
- **Tiered config design**: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
