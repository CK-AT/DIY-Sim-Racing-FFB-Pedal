# Override Field Registry — Badge System Complete

Branch: `ck_tiered_config`
Last commit: `e7f13625` — Add override field registry infrastructure (Phases 1-6)

## Session Summary (2026-02-05)

Fixed badge display system - badges now initialize correctly on load and function switches. All infrastructure complete and functional.

**Phases completed:** 1-6 (Registry, Control, Events, Storage, Fields, Wiring), 8 (UI Integration), 9 (Badge Initialization Fix)
**Phase deferred:** 7 (Baseline integration - needs explicit user control UI)
**Status:** Badge system complete and tested. Ready for UI handler updates to create overrides.

**Files created:** 4 new files
**Files modified:** 9 existing files (7 original + 2 badge fixes)
**Tests:** 172/172 passing (40 new registry tests)
**Build:** All code compiles successfully

## Phase 9 Complete ✓ (Badge Initialization Fix)

Fixed badge initialization timing issue. Badges now load correctly for both initial function and function switches.

**Files modified:**

- `Controls/LayerBadgeWrapper.xaml.cs` — Fixed template loading, removed debug logging
- `FunctionConfigControl.xaml.cs` — Added deferred InitializeBadges in OnLoaded, removed debug logging

**Issue fixed:**

Initial function load: Badges weren't found because SwitchFunction() called InitializeBadges before child controls were in visual tree.

**Solution:**

- Added deferred InitializeBadges call in OnLoaded event (ContextIdle priority)
- Kept deferred call in SwitchFunction for function switches
- Both paths now successfully initialize badges

**Test results:**

- Initial function load: ✅ 4 badges found after OnLoaded
- Function switches: ✅ 4 badges found immediately
- Badge visibility: ✅ Hidden when using Hardware defaults (correct behavior)
- Badge updates: ✅ Properties set correctly (Plugin, FunctionId)

## Phase 1 Complete ✓

Created unified field registry for function-level override fields with comprehensive unit tests.

**Files created:**

- `TieredConfig/OverrideFieldRegistry.cs` — Unified field definitions, metadata, and accessors
- `TieredConfigTests/OverrideFieldRegistryTests.cs` — 40 comprehensive unit tests

**Test results:** 172/172 tests passing (132 existing + 40 new registry tests)

## Phase 2 Complete ✓

Created `LayerBadgeWrapper` WPF control for displaying layer badges on override fields.

**Files created:**

- `Controls/LayerBadgeWrapper.xaml.cs` — Control code-behind with badge display logic
- `Controls/LayerBadgeWrapper.xaml` — Control template with badge overlay

**Features:**

- Wraps any content control (TextBox, Slider, etc.)
- Displays [U]/[P] badges in upper-right corner
- Blue badge for User layer, Green for Profile layer
- Hides badge when using Hardware defaults
- Simple tooltip showing field name and layer source
- Queries OverrideFieldRegistry for field metadata
- Creates ConfigLayerProvider on demand to determine source layer

**Build status:** Compiles successfully with no errors

## Phase 3 Complete ✓ (Infrastructure)

Added event system for badge refresh coordination.

**Changes to DiyFfbPlugin.cs:**

- Added `ContextChanged` event — fired on profile/vehicle/user switches
- Added `OverrideFieldChanged` event — fired on single field edits (no ESP32 send)
- Added `OverrideFieldChangedEventArgs` class
- Added `OnContextChanged()` helper method
- Added `OnOverrideFieldChanged(int functionId, string fieldPath)` helper method

**Event firing locations (to be wired up):**

- `OnContextChanged()` should be called:
  - When profile switches (game/aircraft change)
  - When vehicle changes
  - After "Upload Changes" button sends to ESP32
- `OnOverrideFieldChanged(functionId, fieldPath)` should be called:
  - In `UpdateFunctionOverrideField()` after update
  - In `ClearFunctionOverrideField()` after clear
  - In any UI code that directly modifies overrides

**Build status:** Compiles successfully with no errors

## Phase 4 Complete ✓ (Infrastructure)

Added function baseline storage system for Hardware layer.

**Changes to DiyFfbPluginSettings.cs:**

- Added `FunctionBaselines` dictionary (int → FunctionConfig)
- Stores complete FunctionConfig snapshots as Hardware layer
- Automatically persisted with plugin settings (JSON serialization)

**Changes to DiyFfbPlugin.cs:**

- Added `GetFunctionBaseline(int functionId)` — retrieves Hardware layer config
- Added `SetFunctionBaseline(int functionId, FunctionConfig config)` — updates Hardware layer
- Added `HasFunctionBaseline(int functionId)` — checks if baseline exists

**Usage pattern:**

```csharp
// On ESP32 connect or compound config import:
SetFunctionBaseline(functionId, configFromEsp32);

// For merge operations:
var hardware = GetFunctionBaseline(functionId);
var profile = GetFunctionOverrides(functionId);
var user = GetUserFunctionOverrides(functionId);
var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);
```

**Integration points (to be wired up):**

- Call `SetFunctionBaseline()` when receiving configs from ESP32
- Call `SetFunctionBaseline()` when importing compound configs
- Use baselines in merge operations instead of ESP32-only configs

**Build status:** Compiles successfully with no errors

## Phase 5 Complete ✓ (Field Expansion)

Added all remaining override fields to registry for all function types.

**Changes to TieredConfigTypes.cs:**

- Added `DamperConfigOverrides` class (PositiveFactor, NegativeFactor)
- Added to `FunctionConfigOverrides`:
  - `ForceCurve` (SplineForceCurveConfig) - AutomotivePedals
  - `DamperConfig` (DamperConfigOverrides) - AutomotivePedals
  - `FlightPedalsConfig` (FlightPedalsConfig) - FlightPedals
  - `FlightStickConfig` (FlightStickPitchConfig) - FlightStick
  - `ShifterConfig` (ShifterConfig) - Shifter
- Updated `IsEmpty` to check all new fields

**Changes to OverrideFieldRegistry.cs:**

- Added OverrideFieldGroup enums: Damper, AutomotivePedals, FlightPedals, FlightStick
- Added `FormatValue` property to OverrideFieldDefinition for complex type tooltips
- Registered 7 new fields:
  - DamperPositiveFactor, DamperNegativeFactor (User, Float)
  - ForceCurve (Profile, Complex with FormatValue)
  - FlightPedalsConfig (User, Complex with FormatValue)
  - FlightStickConfig (User, Complex with FormatValue)
  - ShifterConfig (Profile, Complex with FormatValue)

**Changes to FieldRouter.cs:**

- Added new User-level field paths (damper_config, flight_pedals, flight_stick)
- Added prefix routing for nested fields
- Added explicit Profile routing for force_curve and shifter_config

**Total fields in registry:** 13 fields (6 original + 7 new)

- 8 scalar (Float/Bool)
- 5 complex (with FormatValue delegates)

**Build status:** Compiles successfully with no errors

## Phase 6 Complete ✓ (Event Wiring)

Wired up event firing at all context change and field edit locations.

**Changes to DiyFfbPlugin.cs:**

- Added `OnContextChanged()` call in `HandleAircraftChange()` (line ~2100)
  - Fires when profile/vehicle switches (game/aircraft change)
  - Triggers full badge refresh + UI update
- Added `OnOverrideFieldChanged()` call in `UpdateFunctionOverrideField()` (line ~2468)
  - Fires after field updates (User or Profile layer)
  - Triggers targeted badge refresh only, NO ESP32 send
- Added `OnOverrideFieldChanged()` call in `ClearFunctionOverrideField()` (line ~2483)
  - Fires after field clears (User or Profile layer)
  - Triggers targeted badge refresh only, NO ESP32 send

**Event flow:**

```text
Profile/Vehicle Switch → OnContextChanged() → Subscribers refresh all badges
Field Edit/Clear       → OnOverrideFieldChanged() → Subscribers refresh single badge
```

**Build status:** Compiles successfully with no errors

## Phase 7 - Deferred (Design Clarification Needed)

**Baseline Integration:** Infrastructure exists (GetFunctionBaseline/SetFunctionBaseline methods, FunctionBaselines storage) but integration deferred pending design clarification.

**Key insight:** ESP32 configs should NOT auto-populate baselines because:

- ESP32 RAM config may already have overrides applied from previous session
- Auto-baseline on every connect/reconnect would corrupt baseline with overridden values
- Defeats purpose of persistent baseline

**Correct baseline approach (TBD):**

- Baselines set explicitly by user (e.g., "Save as Baseline" button in UI)
- Baselines loaded from known-good template configs
- Baselines used as fallback when ESP32 not connected
- **NOT** auto-set from ESP32 arrival or file imports

**Integration deferred** until baseline semantics are fully defined.

## Phase 8 Complete ✓ (UI Integration)

Wrapped function editor controls with LayerBadgeWrapper to display [U]/[P] layer badges.

**Files modified (XAML):**

- `FunctionConfigControl.xaml` — Added `badge:` namespace, wrapped Static Balance controls
- `AutomotivePedalConfigControl.xaml` — Wrapped damping, friction, simulated_mass, force_curve
- `FlightPedalsConfigControl.xaml` — Wrapped friction, simulated_mass
- `FlightStickConfigControl.xaml` — Wrapped friction, simulated_mass
- `ShifterConfigControl.xaml` — Wrapped friction, simulated_mass

**Files modified (code-behind):**

- `FunctionConfigControl.xaml.cs` — Badge infrastructure (InitializeBadges, event subscriptions, visual tree traversal)
- `AutomotivePedalConfigControl.xaml.cs` — Badge event wiring (Loaded/Unloaded handlers)
- `FlightPedalsConfigControl.xaml.cs` — Badge event wiring
- `FlightStickConfigControl.xaml.cs` — Badge event wiring
- `ShifterConfigControl.xaml.cs` — Badge event wiring

**Badge pattern:**

```xml
<badge:LayerBadgeWrapper FieldPath="friction">
    <Slider x:Name="Slider_friction" .../>
</badge:LayerBadgeWrapper>
```

**Code-behind pattern:**

```csharp
private void InitializeBadges()
{
    if (plugin == null || function == null) return;
    int functionId = (int)function.ID;
    foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
    {
        wrapper.Plugin = plugin;
        wrapper.FunctionId = functionId;
    }
}
```

**Namespace change:** Changed `controls:RangeSlider` to `metro:RangeSlider` across all function editor XAMLs (fixed undeclared prefix error).

**Build status:** Compiles successfully with no errors

## Next Steps

Badge system fully functional. Next phase: Update UI event handlers to use override system.

**Completed phases:**

- ✓ Phase 1: OverrideFieldRegistry (field definitions + accessors)
- ✓ Phase 2: LayerBadgeWrapper (WPF control)
- ✓ Phase 3: Event system (ContextChanged + OverrideFieldChanged)
- ✓ Phase 4: Baseline storage (Hardware layer persistence)
- ✓ Phase 5: Field expansion (all 13 fields registered)
- ✓ Phase 6: Event wiring (context changes + field edits)
- ⏸️ Phase 7: Baseline integration (deferred - infrastructure ready)
- ✓ Phase 8: UI integration (badges on all function editors)
- ✓ Phase 9: Badge initialization fix (OnLoaded + SwitchFunction)

**Current state:**

- ✅ Badge system works correctly (displays, hides, updates)
- ✅ Badges initialize on load and function switches
- ✅ Override infrastructure complete (UpdateFunctionOverrideField, events, storage)
- ❌ UI controls still use direct-edit approach (bypass override system)

**To make badges appear with user edits:**

Update UI event handlers to call `plugin.UpdateFunctionOverrideField()` instead of directly setting `function_config.Field`. This will:

1. Create User/Profile overrides
2. Fire `OnOverrideFieldChanged` event
3. Badges will display [U] or [P]

**Example conversion (FlightStickConfigControl.xaml.cs):**

```csharp
// OLD: Direct edit (bypasses override system)
private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
{
    label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", e.NewValue);
    function_config.SimulatedMass = (float)e.NewValue;
}

// NEW: Use override system
private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
{
    label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", e.NewValue);
    if (plugin != null && function != null)
    {
        plugin.UpdateFunctionOverrideField((int)function.ID, "simulated_mass",
            overrides => overrides.SimulatedMass = (float)e.NewValue);
    }
}
```

**Future enhancements:**

- Add "Clear Override" context menu to badges
- Add "Upload Changes" button for explicit ESP32 sync
- Add badges to additional fields (ABS, RPM effects, etc.)

See plan: `SimHubPlugin/Docs/plans/25_Override_Field_Registry_Plan.md`

## Key Design Decisions

| Decision | Choice |
|----------|--------|
| ESP32 sends | Manual "Upload Changes" button only — no auto-send during edits |
| Event system | Two-tier: `ContextChanged` (full refresh + ESP32), `OverrideFieldChanged` (local badge only) |
| LayerBadgeWrapper | Wraps any editor (TextBox, Slider, complex), displays `[U]`/`[P]` badge |
| Subscriber lifecycle | Subscribe in `Loaded`, unsubscribe in `Unloaded` |
| Badge initialization | Deferred to ContextIdle priority in both OnLoaded and SwitchFunction |
| Debouncing | Not needed for ESP32 (no sends); optional for UI if slider lag observed |

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
