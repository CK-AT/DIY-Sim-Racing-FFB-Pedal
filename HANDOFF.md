# Session Handoff

Date: 2026-01-30
Last commit: `53fcb8d7` — Remove cross-session pending graph params file

## What Was Done This Session

### 1. Fixed Profile Key Inconsistency (Issue #1)
Changed profile keying from `CarId` alone to `gameId::carId` format:

**Code changes in `DiyFfbPlugin.cs`:**
- Added `BuildProfileKey(gameId, carId)` helper method
- Added `MigrateProfileKeyIfNeeded(gameId, carId)` for automatic migration of old keys
- Added `GetActiveProfileKey()` public accessor for UI
- Updated all profile access methods to use composite key:
  - `GetCurrentAircraftProfile()`
  - `SaveCurrentAircraftProfile(gameId, carId)`
  - `ApplyAircraftProfile(gameId, carId)`
  - `HasUnsavedProfileChanges(gameId, carId)`
  - `ApplyAircraftFfbProfile()`
  - `ReplaceAircraftFfbProfiles()`
  - `HandleAircraftChange()`

**Code changes in `DiyFfbPluginUI.xaml.cs`:**
- Updated `btn_save_aircraft_ffb_Click` to use `GetActiveProfileKey()`

**Migration behavior:** When a profile is accessed, if the new key (`gameId::carId`) doesn't exist but the old key (`carId`) does, the profile is automatically migrated to the new key format.

## Open Items (Priority Order)

From [17_Profile_System_Improvements.md](SimHubPlugin/Docs/plans/17_Profile_System_Improvements.md):

| Priority | Issue | Notes |
|----------|-------|-------|
| Medium | #4 Tier 2 deprecation | Clarify graph.ParamValues vs profile.GraphParamValues |
| Medium | #7 Reset to defaults | Add button to clear vehicle profile |
| Low | #3 FunctionFfbSettings | Stub code—decide to implement or remove |
| Low | #6 Export/import | Nice-to-have |
| Low | #8 Profile deletion | Nice-to-have |

From [15_Graph_Param_Override_Migration_Plan.md](SimHubPlugin/Docs/plans/15_Graph_Param_Override_Migration_Plan.md):
- Implement hash tracking for graph + includes
- Implement Parameter Review Window
- Add notification on defaults change
- Add tests for override persistence/clamping/orphans

## Key Files

| File | Purpose |
|------|---------|
| `SimHubPlugin/DiyFfbPlugin.cs` | Main plugin—profile handling around lines 1469-1520, 1873-2070 |
| `SimHubPlugin/DiyFfbPluginSettings.cs` | AircraftFfbProfile class, settings storage |
| `SimHubPlugin/DiyFfbPluginUI.xaml.cs` | UI dialogs and controls |
| `SimHubPlugin/Docs/plans/15*.md` | Migration plan |
| `SimHubPlugin/Docs/plans/16*.md` | Profile lifecycle reference |
| `SimHubPlugin/Docs/plans/17*.md` | Improvements backlog |

## Decisions Made

1. **No stable param IDs** — Match overrides by name only; if name changes, old override becomes orphan
2. **Keep orphaned overrides silently** — Don't delete, let user clean up via review window
3. **Non-blocking UX** — Brief notification + on-demand review window, not modal dialogs
4. **No cross-session pending params** — Standard in-memory dirty tracking with save prompts
5. **Profile keys use gameId::carId** — Consistent with graph path keys, auto-migration on access

## Build Command

```bash
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" /p:Configuration=Debug /v:minimal /nologo
```

## Next Steps

1. **If continuing profile work**: Start with reset to defaults (#7) or tier 2 deprecation (#4)
2. **If implementing migration plan**: Start with hash tracking, then Parameter Review Window
3. **If doing unrelated work**: This handoff can be ignored
