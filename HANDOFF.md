# Session Handoff

Date: 2026-01-30
Last uncommitted: Plan 15 — Hash-based param override preservation

## What Was Done This Session

### Implemented: Plan 15 — Hash-Based Param Override Preservation

Preserves vehicle parameter overrides when graphs change (template swap or file edits).

**New files:**

| File | Purpose |
| ---- | ------- |
| `SimHubPlugin/GraphHashComputer.cs` | Computes SHA256 hash of graph + all includes |
| `SimHubPlugin/ParamMigrationResult.cs` | Migration result data classes |
| `SimHubPlugin/GraphEditor/ParamReviewWindow.xaml` | Parameter review UI |
| `SimHubPlugin/GraphEditor/ParamReviewWindow.xaml.cs` | Review window code-behind |

**Modified files:**

| File | Changes |
| ---- | ------- |
| `DiyFfbPluginSettings.cs` | Added `ParamSnapshot` class, extended `AircraftFfbProfile` with `LastReviewedGraphHash` and `LastReviewedParamSnapshots` |
| `DiyFfbPlugin.cs` | Added `CheckParamMigration()`, `MigrateParamOverrides()`, `InitializeParamSnapshots()`, `GetAllGraphParams()`, `GetActiveVehicleGraph()`, made `ResolveGraphFilePath()` public, added `ParamMigrationDetected` event |
| `DiyFfbPluginUI.xaml` | Added "Review Params" button |
| `DiyFfbPluginUI.xaml.cs` | Added `OnParamMigrationDetected()` handler, `btn_review_params_Click()`, notification logic |
| `DiyFfbPlugin.csproj` | Added new files |

**How it works:**

1. On graph load, `CheckParamMigration()` computes content hash of root graph + includes
2. Compares to stored `LastReviewedGraphHash` in profile
3. If hash changed, `MigrateParamOverrides()`:
   - Clamps override values to new min/max ranges
   - Detects orphaned overrides (params no longer in graph)
   - Detects changed default values
4. If changes detected, fires `ParamMigrationDetected` event
5. UI shows notification and enables "Review Params" button
6. Review window shows active params (highlighted if changed/clamped) and orphans
7. "Mark Reviewed" updates stored hash and snapshots

## Build Status

Build succeeded.

## Testing Checklist

### Hash-Based Param Override Preservation

1. [ ] New vehicle first load — hash initialized without notification
2. [ ] Override persists across graph reload (same params)
3. [ ] Override clamped when new range is tighter
4. [ ] Orphan kept when param removed from graph
5. [ ] Orphan restored when param re-added to graph
6. [ ] Hash change detected on nested include edit
7. [ ] Notification appears when graph changes with impact
8. [ ] Review window shows active params and orphans
9. [ ] Reset/delete buttons work in review window
10. [ ] "Mark Reviewed" updates stored hash and clears highlights

## Key Files

| File | Purpose |
| ---- | ------- |
| `SimHubPlugin/GraphHashComputer.cs` | Hash computation for graph tree |
| `SimHubPlugin/ParamMigrationResult.cs` | Migration result classes |
| `SimHubPlugin/GraphEditor/ParamReviewWindow.xaml.cs` | Review UI |
| `SimHubPlugin/DiyFfbPlugin.cs:1742-1905` | Migration logic |
| `SimHubPlugin/DiyFfbPluginSettings.cs:6-31` | Data model changes |

## Build Command

```bash
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" /p:Configuration=Debug /v:minimal /nologo
```

## Next Steps

1. **Test** the hash-based param override preservation
2. Update plan document status to "Implemented"
