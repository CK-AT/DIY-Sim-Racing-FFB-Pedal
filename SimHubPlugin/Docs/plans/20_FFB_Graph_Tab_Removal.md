# FFB Graph Tab Removal

Date: 2026-01-31
Status: Implemented

## Goal

Remove the redundant "FFB Graph" sub-tab from Settings and consolidate vehicle-related info into the Vehicle tab.

## Background

The Profile Browser now handles graph selection for vehicles, making several controls in Settings → FFB Graph redundant. The remaining useful elements belong in the Vehicle tab.

## Current State (Settings → FFB Graph)

| Element | Purpose | Disposition |
|---------|---------|-------------|
| Aircraft display | Shows current vehicle ID | Move to Vehicle tab header |
| Vehicle graph (Browse/Clear) | Set graph for current vehicle | Remove (Profile Browser) |
| Game graph (Browse/Clear) | Default graph for game | Remove (Profile Browser handles new vehicles) |
| Active graph display | Shows resolved graph path | Move to Vehicle tab |
| System Parameters | Params with Group="System" | Keep in Vehicle tab (as a group) |
| Save/Load FFB Map | Bulk export/import all profiles | Keep in Settings (different location) |

## Implementation

### Phase 1: Add info to Vehicle tab

1. Add vehicle ID line to Vehicle tab header area (e.g., "XPlane::Cessna_172")
2. Add active graph display below header
3. System params already work — they'll appear as a "System" group if any params have that group

### Phase 2: Move bulk export/import

Move "Save FFB Map" / "Load FFB Map" buttons to Settings tab (outside the FFB Graph sub-tab).

Suggested location: Near other import/export functionality, or in a "Backup" section.

### Phase 3: Remove FFB Graph sub-tab

1. Remove the entire FFB Graph TabItem from Settings
2. Remove associated UI elements and event handlers:
   - `TextBlock_ActiveAircraft`
   - `TextBox_VehicleGraphPath`, `btn_select_vehicle_graph`, `btn_clear_vehicle_graph`
   - `TextBox_GameGraphPath`, `btn_select_game_graph`, `btn_clear_game_graph`
   - `TextBlock_ActiveGraph`
   - `SystemGraphParamsPanel`
3. Remove handler methods:
   - `btn_select_vehicle_graph_Click`
   - `btn_clear_vehicle_graph_Click`
   - `btn_select_game_graph_Click`
   - `btn_clear_game_graph_Click`
   - `RefreshSystemGraphParams`
   - `RefreshGraphSelectionUI` (if no longer needed)
4. Remove `systemGraphParamControls` dictionary

### Phase 4: Clean up backend

1. Remove `GameGraphPaths` from settings (no longer used)
2. Keep `SetGameGraphPath` / `GetGameGraphPath` temporarily for migration, or remove if safe
3. Update `ResolveActiveGraph` to not fall back to game-level paths

## Files to Modify

| File | Changes |
|------|---------|
| `DiyFfbPluginUI.xaml` | Add Vehicle tab header info, remove FFB Graph sub-tab, relocate bulk export buttons |
| `DiyFfbPluginUI.xaml.cs` | Remove handlers, add new refresh methods for Vehicle tab info |
| `DiyFfbPluginSettings.cs` | Remove `GameGraphPaths` (or mark obsolete) |
| `DiyFfbPlugin.cs` | Update `ResolveActiveGraph`, remove game graph methods if unused |

## Testing

- Build succeeds
- Vehicle tab shows vehicle ID and active graph
- System params appear in Vehicle tab if defined in graph
- Bulk export/import still works from new location
- New vehicle detection still triggers Profile Browser
- No references to removed UI elements

## Risks

| Risk | Mitigation |
|------|------------|
| Users with GameGraphPaths set lose fallback | Profile Browser handles new vehicles; existing profiles unaffected |
| Breaking change for users | Low impact — game graph was a minor feature |

## Done Definition

- FFB Graph sub-tab removed from Settings
- Vehicle ID and active graph shown in Vehicle tab
- Bulk export/import accessible from Settings
- Build passes, tests pass
- No dead code remains
