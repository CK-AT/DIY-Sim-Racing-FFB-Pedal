# Override Field Registry — Infrastructure Complete

Branch: `ck_tiered_config`
Last commit: `9ef65069` — Add user profile UI and fix function badge refresh

## Session Summary (2026-02-04)

Implemented complete infrastructure for unified override system with layer badges. All core components built and tested, ready for UI integration.

**Phases completed:** 1-6 (Registry, Control, Events, Storage, Fields, Wiring)
**Phase deferred:** 7 (Baseline integration - needs explicit user control UI)
**Next:** Phase 8 - UI integration (wrap function editor controls with badges)

**Files created:** 4 new files
**Files modified:** 7 existing files
**Tests:** 172/172 passing (40 new registry tests)
**Build:** All code compiles successfully

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

## Next Steps

Infrastructure complete and ready for UI integration:

- ✓ Phase 1: OverrideFieldRegistry (field definitions + accessors)
- ✓ Phase 2: LayerBadgeWrapper (WPF control)
- ✓ Phase 3: Event system (ContextChanged + OverrideFieldChanged)
- ✓ Phase 4: Baseline storage (Hardware layer persistence)
- ✓ Phase 5: Field expansion (all 13 fields registered)
- ✓ Phase 6: Event wiring (context changes + field edits)
- ⏸️ Phase 7: Baseline integration (deferred - infrastructure ready)

**Ready for Phase 8:** UI integration - wrap function editor controls with LayerBadgeWrapper.

See plan: `SimHubPlugin/Docs/plans/25_Override_Field_Registry_Plan.md`

## Key Design Decisions

| Decision | Choice |
|----------|--------|
| ESP32 sends | Manual "Upload Changes" button only — no auto-send during edits |
| Event system | Two-tier: `ContextChanged` (full refresh + ESP32), `OverrideFieldChanged` (local badge only) |
| LayerBadgeWrapper | Wraps any editor (TextBox, Slider, complex), displays `[U]`/`[P]` badge |
| Subscriber lifecycle | Subscribe in `Loaded`, unsubscribe in `Unloaded` |
| Debouncing | Not needed for ESP32 (no sends); optional for UI if slider lag observed |

## ESP32 Send Policy

| Action | Badge update | ESP32 send |
|--------|--------------|------------|
| Edit override field | ✓ Immediate | ✗ No |
| Clear override | ✓ Immediate | ✗ No |
| Click "Upload Changes" | — | ✓ Yes |
| Profile/vehicle switch | ✓ Full refresh | ✓ Yes |

## Implementation Phases

1. **Phase 1**: Create `OverrideFieldRegistry.cs` + unit tests ← **START HERE**
2. **Phase 2**: Create `LayerBadgeWrapper` control
3. **Phase 3**: Wrap scalar fields in Vehicle Profile tab
4. **Phase 4**: Wrap function-level fields in Functions tab
5. **Phase 5**: Add "Linked Axes" summary panel + navigation
6. **Phase 6**: Add activation toggle to Functions tab
7. **Phase 7**: Add "Upload Changes" button

## Files to Create (Phase 1)

| File | Purpose |
|------|---------|
| `TieredConfig/OverrideFieldRegistry.cs` | Field definitions, accessors, layer routing |
| `TieredConfigTests/OverrideFieldRegistryTests.cs` | Unit tests |

## Existing Infrastructure

**TieredConfig classes** (`SimHubPlugin/TieredConfig/`):
- `TieredConfigTypes.cs` — `ConfigLayer` enum, `FunctionConfigOverrides`, `AxisParameterOverrides`
- `ConfigMerger.cs` — Pure merge functions
- `ConfigLayerProvider.cs` — Determines field source layer
- `FieldRouter.cs` — Routes changes to correct layer
- `FunctionConfigManager.cs` — Manages function config lifecycle, fires `FunctionConfigChanged`
- `AxisConfigManager.cs` — Manages axis config lifecycle, fires `AxisConfigChanged`

**Unit tests**: 132 tests passing
```bash
cd SimHubPlugin/TieredConfigTests/bin/Debug && ./TieredConfigTests.exe
```

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
