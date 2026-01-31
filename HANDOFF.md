# Session Handoff

Date: 2026-01-31
Last commit: `d633e76b` — Remove FFB Graph tab, show vehicle info in Vehicle tab

## What Was Done This Session

### Implemented Plan 20: FFB Graph Tab Removal

Removed the redundant "FFB Graph" sub-tab from Settings and consolidated vehicle info into the Vehicle tab.

**Changes made:**

1. **Vehicle tab info display** — Added vehicle ID (with game name in parenthesis) and active graph path to Vehicle tab header
2. **Bulk export/import relocated** — Moved "Export All Profiles" / "Import All Profiles" buttons to Settings left panel under "Profile Backup" section
3. **FFB Graph sub-tab removed** — Entire tab removed from Settings
4. **Code-behind cleanup** — Removed unused handlers and methods:
   - Vehicle/game graph selection handlers
   - `RefreshSystemGraphParams` and `systemGraphParamControls`
   - Updated `UpdateActiveAircraftLabel` to show game name
5. **Backend cleanup** — Removed game graph fallback logic and methods
6. **Layout fixes** — ScrollViewer set to always-visible scrollbar with margin centering

## Build Status

Build compiles successfully. 77/77 tests pass.

## Files Modified

- `DiyFfbPluginUI.xaml` — Vehicle tab info, bulk export buttons, removed FFB Graph tab, layout fixes
- `DiyFfbPluginUI.xaml.cs` — Cleaned up handlers and refresh methods
- `DiyFfbPlugin.cs` — Removed game graph methods and fallback logic
- `DiyFfbPluginSettings.cs` — Marked `GameGraphPaths` as obsolete
- `AxisConfigControl.xaml` — Minor margin tweak
- `Docs/plans/20_FFB_Graph_Tab_Removal.md` — Created and marked Implemented

## Build & Test Commands

```bash
# Build main plugin
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" /p:Configuration=Debug /v:minimal /nologo

# Build and run tests
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\GraphTest.csproj" /p:Configuration=Debug /v:minimal /nologo
"d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\GraphTest\bin\Debug\net48\GraphTest.exe"
```

## Related Documents

- [Plan 20: FFB Graph Tab Removal](SimHubPlugin/Docs/plans/20_FFB_Graph_Tab_Removal.md)
