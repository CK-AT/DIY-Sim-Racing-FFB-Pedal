# Override Field Registry — Complete Infrastructure Ready

Branch: `ck_tiered_config`
Last commit: `4f77e4d3` — Add badge wrappers to all function editor controls (Phase 8)

## Session Summary (2026-02-05)

Fixed badge system initialization, corrected registry field structure per plan, implemented complete merge logic for all function types. Badge infrastructure 100% complete and tested.

**Phases completed:** 1-6, 8-10 (All infrastructure complete)
**Phase deferred:** 7 (Baseline integration - needs explicit user control UI)
**Status:** Infrastructure complete. Ready for UI handler migration to override system.

**New commits:** 4 commits
- `075e58f0` — Badge initialization timing fix
- `bf451542` — Registry structure fix (individual fields per plan)
- `f253dc7f` — ConfigMerger complete merge logic
- `4f77e4d3` — Phase 8 badge wrapping complete

**Files modified:** 13 files
**Tests:** 172/172 passing (40 registry tests)
**Build:** Success

## Phase 9 Complete ✓ (Badge Initialization Fix)

Fixed badge initialization timing issue discovered during testing.

**Problem:** `InitializeBadges()` called in `SwitchFunction()` before visual tree loaded, found 0 badges. Subsequent function switches worked, but initial function failed.

**Solution:**
- Added deferred `InitializeBadges()` call in `OnLoaded` event (ContextIdle priority)
- Kept deferred call in `SwitchFunction` for function switches
- Removed debug logging after verification

**Files modified:**
- `Controls/LayerBadgeWrapper.xaml.cs` — Fixed template loading, added static constructor
- `FunctionConfigControl.xaml.cs` — Added OnLoaded badge initialization

**Result:** Badges now initialize correctly for both initial load and function switches.

## Phase 10 Complete ✓ (Registry Structure Correction)

Corrected registry structure to match plan specification. Phase 5 was implemented incorrectly with whole-config overrides instead of individual fields.

**Registry changes:**

**Removed (wrong):**
- `flight_pedals` → entire FlightPedalsConfig
- `flight_stick` → entire FlightStickPitchConfig

**Added (correct):**
- `flight_pedals.motion_range` (Complex - RangeSlider, FormatValue)
- `flight_pedals.damping` (Float)
- `flight_pedals.centering_spring_const` (Float)
- `flight_stick.motion_range` (Complex - RangeSlider, FormatValue)
- `flight_stick.damping` (Float)
- `flight_stick.centering_spring_const` (Float)
- `aux_function.rudder_brake.force_range` (Complex - RangeSlider, FormatValue)

**Kept (correct):**
- `shifter_config` (Complex - entire config, interdependent geometry)
- `force_curve` (Complex - entire spline curve)

**Total registry fields:** 17 fields (was 13, removed 2 wrong, added 6 correct)

**Files modified:**
- `TieredConfig/TieredConfigTypes.cs` — Added MotionRangeOverrides, ForceRangeOverrides types; replaced whole configs with individual fields
- `TieredConfig/OverrideFieldRegistry.cs` — Replaced whole-config entries with per-field registrations
- `TieredConfig/FieldRouter.cs` — Removed explicit whole-config paths, added aux_function prefix routing
- `FlightPedalsConfigControl.xaml` — Fixed FieldPath from `flight_pedals` to `flight_pedals.motion_range`, wrapped damping/spring sliders
- `FlightStickConfigControl.xaml` — Fixed FieldPath from `flight_stick` to `flight_stick.motion_range`, wrapped damping/spring sliders

## Phase 11 Complete ✓ (ConfigMerger Complete)

Implemented complete merge logic for all function-specific override fields based on control analysis.

**Key principle:** Merge SOURCE fields only, let controls derive Base.OutputMin/OutputMax.

**Control relationships discovered:**
- **AutomotivePedals**: `Base.Output{Min,Max} = ForceCurveConfig.{FMin,FMax}` OR `{PosMin,PosMax}` (depends on OutputMode)
- **FlightPedals**: `Base.Output{Min,Max} = {PosNearLim, PosFarLim}` (1:1 sync)
- **FlightStick**: `Base.Output{Min,Max} = {PosMin, PosMax}` (1:1 sync)
- **Shifter**: `Base.Output{Min,Max} = {PosXMin/Max}` OR `{PosYMin/Max}` (depends on Sequential flag)

**Merge methods added:**
- `ApplyAutomotivePedalOverrides()` — Merges DamperConfig, ForceCurve
- `ApplyFlightPedalsOverrides()` — Merges motion_range, damping, centering_spring_const, rudder_brake force_range
- `ApplyFlightStickOverrides()` — Mode-specific merge (Pitch/Roll/Collective), uses dynamic for type flexibility

**Files modified:**
- `TieredConfig/ConfigMerger.cs` — Added 3 merge methods, integrated into MergeFunctionConfig

## Phase 8 Complete ✓ (UI Integration)

Wrapped all function editor controls with LayerBadgeWrapper to display [U]/[P] layer badges.

**Files modified (XAML):**
- `FunctionConfigControl.xaml` — Added `badge:` namespace, wrapped Static Balance controls
- `AutomotivePedalConfigControl.xaml` — Wrapped damping, friction, simulated_mass, force_curve
- `FlightPedalsConfigControl.xaml` — Wrapped motion_range, damping, centering_spring_const, friction, simulated_mass
- `FlightStickConfigControl.xaml` — Wrapped motion_range, damping, centering_spring_const, friction, simulated_mass
- `ShifterConfigControl.xaml` — Wrapped friction, simulated_mass

**Files modified (code-behind):**
- `FunctionConfigControl.xaml.cs` — Badge infrastructure (InitializeBadges, event subscriptions, visual tree traversal, OnLoaded initialization)
- `AutomotivePedalConfigControl.xaml.cs` — Badge event wiring (Loaded/Unloaded handlers)
- `FlightPedalsConfigControl.xaml.cs` — Badge event wiring
- `FlightStickConfigControl.xaml.cs` — Badge event wiring
- `ShifterConfigControl.xaml.cs` — Badge event wiring

**Namespace change:** Changed `controls:RangeSlider` to `metro:RangeSlider` across all function editor XAMLs.

## Previous Phases (Commit e7f13625)

**Phase 1-6 complete** from previous session:
- Phase 1: OverrideFieldRegistry infrastructure
- Phase 2: LayerBadgeWrapper WPF control
- Phase 3: Event system (ContextChanged, OverrideFieldChanged)
- Phase 4: Baseline storage (Hardware layer persistence)
- Phase 5: Field expansion (originally wrong, corrected in Phase 10)
- Phase 6: Event wiring (context changes, field edits)

## Next Steps

Badge system 100% complete. Next phase: Update UI event handlers to create overrides.

**Completed phases:**
- ✓ Phase 1: OverrideFieldRegistry (field definitions + accessors)
- ✓ Phase 2: LayerBadgeWrapper (WPF control)
- ✓ Phase 3: Event system (ContextChanged + OverrideFieldChanged)
- ✓ Phase 4: Baseline storage (Hardware layer persistence)
- ✓ Phase 5: Field expansion (corrected in Phase 10)
- ✓ Phase 6: Event wiring (context changes + field edits)
- ✓ Phase 7: Baseline integration (manager initialization on startup)
- ✓ Phase 8: UI integration (badges on all function editors)
- ✓ Phase 9: Badge initialization fix (OnLoaded + SwitchFunction)
- ✓ Phase 10: Registry structure correction (per-field vs whole-config)
- ✓ Phase 11: ConfigMerger complete (all function types)
- ✓ Phase 12: Proof-of-concept (simulated_mass override with full persistence)

**Infrastructure complete:**
- ✅ Badge system (display, initialization, refresh)
- ✅ Override registry (17 fields, correct structure)
- ✅ Merge logic (all function types, respects control derivations)
- ✅ Event system (ContextChanged, OverrideFieldChanged)
- ✅ Storage (FunctionBaselines, FunctionOverrides, UserFunctionOverrides)

**To make badges visible to users:**

Update UI event handlers to call `plugin.UpdateFunctionOverrideField()` instead of directly setting `function_config.Field`.

**Example conversion (FlightStickConfigControl.xaml.cs:514-515):**

```csharp
// OLD: Direct edit (bypasses override system)
SetPosMin(Convert.ToInt16(e.NewValue));
function_config.Base.OutputMin = Convert.ToInt16(e.NewValue);

// NEW: Use override system
if (plugin != null && function != null)
{
    var newValue = Convert.ToInt16(e.NewValue);
    plugin.UpdateFunctionOverrideField((int)function.ID, "flight_stick.motion_range",
        overrides => {
            if (overrides.FlightStickMotionRange == null)
                overrides.FlightStickMotionRange = new MotionRangeOverrides();
            overrides.FlightStickMotionRange.Min = newValue;
        });
}
```

**Note:** Controls will continue to derive Base.OutputMin/OutputMax from source fields. Merge happens in ConfigMerger, controls display merged result.

## Key Design Decisions

| Decision | Choice |
|----------|--------|
| ESP32 sends | Manual "Upload Changes" button only — no auto-send during edits |
| Event system | Two-tier: `ContextChanged` (full refresh + ESP32), `OverrideFieldChanged` (local badge only) |
| LayerBadgeWrapper | Wraps any editor (TextBox, Slider, RangeSlider, complex), displays `[U]`/`[P]` badge |
| Subscriber lifecycle | Subscribe in `Loaded`, unsubscribe in `Unloaded` |
| Badge initialization | Deferred to ContextIdle priority in both OnLoaded and SwitchFunction |
| Range fields | Complex type with FormatValue (motion_range, force_range) wraps two protobuf fields |
| Base.OutputMin/Max | DERIVED fields - merged by ConfigMerger into source fields, controls calculate Base.Output |
| Merge strategy | Source fields only - controls handle all derivations and synchronizations |

## ESP32 Send Policy

| Action | Badge update | ESP32 send |
|--------|--------------|------------|
| Edit override field | ✓ Immediate | ✗ No |
| Clear override | ✓ Immediate | ✗ No |
| Click "Upload Changes" | — | ✓ Yes |
| Profile/vehicle switch | ✓ Full refresh | ✓ Yes |

## Build

```bash
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Debug /v:minimal /nologo
```

## References

- **Full plan**: `SimHubPlugin/Docs/plans/25_Override_Field_Registry_Plan.md`
- **Tiered config design**: `SimHubPlugin/Docs/plans/24_Tiered_Config_Overrides.md`
- **Control analysis**: Task agent a5a2995 (field mappings, derived fields, transformations)

## Phase 7 - Baseline Integration (Complete) ✓

Initialize FunctionConfigManager with stored baselines and overrides on plugin startup. Add "Save as Baseline" button for explicit baseline creation.

**Implementation:**

1. **InitializeManagerFromSettings()** (DiyFfbPlugin.cs:2415-2447):
   - Loads all stored baselines from `Settings.FunctionBaselines`
   - Calls `_functionConfigManager.SetBaseConfig()` for each function
   - Retrieves profile and user overrides from settings
   - Applies overrides via `ApplyProfileOverrides()` to populate merged configs
   - Called in `Init()` after settings load (DiyFfbPlugin.cs:3728)

2. **"Save as Baseline" button**:
   - Added to main UI below Upload/Download/Load/Store buttons (DiyFfbPluginUI.xaml:520-524)
   - Click handler `OnSaveFunctionBaselineClicked()` (DiyFfbPluginUI.xaml.cs:2835-2871)
   - Gets current merged config from manager (baseline + all overrides)
   - Saves merged config as new baseline via `SetFunctionBaseline()`
   - **Clears all overrides** via `ClearAllFunctionOverrides()` (bakes them into baseline)
   - Re-applies to manager and refreshes UI
   - Shows confirmation dialog

3. **ClearAllFunctionOverrides()** (DiyFfbPlugin.cs:2582-2606):
   - Clears profile overrides for function
   - Clears user overrides for function
   - Re-applies to manager to update merged config
   - Used when "baking" overrides into baseline

**Architecture:**

Baselines are **explicitly saved** by user action (not auto-saved from ESP32):
- User clicks "Save as Baseline" to capture current effective config
- Baseline stored in `Settings.FunctionBaselines` (persisted to disk)
- On startup, baseline loaded as base layer for merge operations
- Overrides (Profile + User) merge on top of baseline

**Result:**

Manager is now populated with merged configs (baseline + profile + user overrides) before UI loads. Function switching displays correct values immediately, even before ESP32 connection. Stored overrides persist across plugin restarts.

**Testing workflow:**
1. Edit simulated_mass on Flight Stick Pitch → override created, [U] badge appears
2. Click "Save as Baseline" → baseline saved, overrides cleared (badge disappears)
3. Edit simulated_mass again → new override created, [U] badge appears
4. Restart plugin → UI shows baseline + new override (persisted correctly)

## Phase 12 - Proof-of-Concept (Complete) ✓

Implemented end-to-end test with FlightStickPitch.simulated_mass to validate badge system. Fixed all UI refresh issues.

**What works:**
- ✅ Override creation (UpdateFunctionOverrideField)
- ✅ Override storage (User layer)
- ✅ Badge appearance ([U] blue badge displays)
- ✅ Badge refresh (OnOverrideFieldChanged event fires)
- ✅ Merge logic (ConfigMerger applies overrides correctly)
- ✅ Manager re-merge (FunctionConfigManager.ApplyProfileOverrides called after override update)
- ✅ UI shows merged values after override creation
- ✅ Initial load displays merged config (base + profile + user overrides)
- ✅ Function switching preserves and displays overrides

**Fixes applied:**

1. **OnSimulatedMassChanged** (FlightStickConfigControl.xaml.cs:555-591):
   - Create override first via UpdateFunctionOverrideField
   - Get merged config from FunctionConfigManager.GetCurrentConfig
   - Update function_config.SimulatedMass with merged value
   - Update label with merged value
   - Fallback to direct edit if manager unavailable

2. **SwitchFunction** (FunctionConfigControl.xaml.cs:155-178):
   - Check if manager has base config for function
   - Get merged config from FunctionConfigManager.GetCurrentConfig
   - Set config = mergedConfig to display merged values
   - Update function.Config so child controls see merged config
   - Fallback to function.Config if manager unavailable

3. **SwitchFunction** (FlightStickConfigControl.xaml.cs:441-503):
   - Check if manager has base config for function
   - Get merged config from FunctionConfigManager.GetCurrentConfig
   - Set function_config = mergedConfig to display merged values
   - Explicitly update all labels after is_updating=false (simulated_mass, friction, centering_spring_const, damping)
   - Fallback to function.Config if manager unavailable

**Architecture:**

The POC uses a **merged config display pattern**:
- `FunctionConfigManager` maintains merged configs in `_currentConfigs`
- UI controls read merged config via `GetCurrentConfig()` on load
- When editing, create override AND fetch merged config from manager
- Manager automatically re-merges when override is updated

**Files modified:**
- `Controls/LayerBadgeWrapper.xaml` — Fixed layout (Visibility.Hidden, negative margins)
- `Controls/LayerBadgeWrapper.xaml.cs` — Use Hidden instead of Collapsed
- `FunctionConfigControl.xaml.cs` — Load merged config in SwitchFunction
- `FlightStickConfigControl.xaml.cs` — Load merged config in SwitchFunction, fetch merged value after override
- `DiyFfbPlugin.cs` — UpdateFunctionOverrideField re-merges via manager

**Result:**

POC fully working. Badge system displays override state, UI shows merged values at all times (initial load, after edits, during function switches). Ready to migrate remaining event handlers using this pattern.
