# Session Handoff

Date: 2026-01-31
Last commit: `5ba1989e` — Fix param changes persisting when discarded on vehicle switch

## What Was Done This Session

### Committed Changes

#### 1. Move profile buttons from Settings to Vehicle tab (`f7bf66b1`)

- Added button bar to Vehicle tab with: Manage Profiles, Review Params, Store Profile, Export Profile, Import Profile, Graph Editor
- Removed those buttons from Settings → FFB Graph
- Removed Graph Editor button from Function Config area
- Removed obsolete `btn_reset_params_defaults_Click` handler

#### 2. Fix param changes persisting when discarded (`5ba1989e`)

Fixed bug where parameter changes were stored even when user clicked "No" (discard) in the vehicle-change or shutdown dialog.

**Root cause:** `SetGraphParamValue` wrote directly to the profile object in `Settings.AircraftFfbProfiles`. When the user discarded, the profile already had the modified values which got saved on shutdown.

**Fix:** Snapshot-based restore mechanism:

- Added `_graphParamValuesSnapshot` field to capture original `GraphParamValues` before first edit
- `MarkProfileDirty()` now takes snapshot on first dirty (before any modification)
- Added `DiscardProfileChanges()` method to restore from snapshot
- Added `ClearDirtyState()` helper to reset dirty flag and snapshot
- Fixed both code paths: `HandleAircraftChange` (vehicle switch) and `End` (SimHub shutdown)
- Param overrides no longer incorrectly mark graph tab as dirty (they're profile-level, not graph-level)

**Modified files:**

| File | Changes |
| ---- | ------- |
| `SimHubPlugin/DiyFfbPlugin.cs` | Snapshot/restore mechanism, discard on cancel |
| `SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs` | Remove incorrect dirty marking on param change |

## Build Status

Build compiles successfully. 77/77 tests pass.

## Build & Test Commands

```bash
# Build main plugin
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" /p:Configuration=Debug /v:minimal /nologo

# Build and run tests
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\GraphTest.csproj" /p:Configuration=Debug /v:minimal /nologo
"d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\bin\Debug\net48\GraphTest.exe"
```
