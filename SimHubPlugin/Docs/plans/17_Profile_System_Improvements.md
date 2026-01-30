# Profile System Improvements

Date: 2026-01-30
Status: Draft

## Overview

Identified gaps in the current vehicle profile system and proposed improvements.

## Issues

### 1. ~~Profile Key Inconsistency~~ ✓ DONE

**Status:** Resolved 2026-01-30

Changed profile keying from `CarId` alone to `gameId::carId` format (matching graph paths). Migration happens automatically when a vehicle profile is accessed—old-style keys are renamed to new format on first load. Added `BuildProfileKey()`, `MigrateProfileKeyIfNeeded()`, and `GetActiveProfileKey()` helper methods.

---

### 2. ~~Pending Params File Unnecessary~~ ✓ DONE

**Status:** Resolved 2026-01-30

Removed cross-session `pending_graph_params.json` file and all related code. Profile changes are now tracked in-memory only, with save prompts on vehicle change or shutdown. If user declines to save, changes are discarded—standard app behavior.

---

### 3. ~~FunctionFfbSettings Stub~~ ✓ DONE

**Status:** Resolved 2026-01-30 — Removed (Option B)

Removed dead code:

- Deleted `FunctionFfbSettings` class (empty CopyFrom/ApplyTo methods)
- Removed four unused fields from `AircraftFfbProfile`
- Removed `AreFunctionFfbSettingsEqual` helper method
- Cleaned up `ApplyFfbProfileToCurrentSettings` and related methods

Per-vehicle FFB settings are now handled entirely through `GraphParamValues`.

---

### 4. ~~Tier 2 ParamValues Deprecation~~ ✓ DONE

**Status:** Resolved 2026-01-30

Removed runtime writes to Tier 2 (graph.ParamValues) in `SetGraphParamValue()`. Now:

- Tier 2 (graph.ParamValues) is read-only at runtime, contains only template defaults from JSON file
- All user edits go to Tier 3 (profile.GraphParamValues)
- Docstrings updated to clarify this behavior

---

### 5. Game-Specific Fields

**Current:** `XPlaneRotorIndex` is hardcoded in `AircraftFfbProfile`.

**Problem:** As more games get unique settings, the class grows with game-specific fields.

**Options:**

- A) Keep adding fields (simple but messy)
- B) Use `Dictionary<string, object> GameSpecificSettings`
- C) Use separate per-game settings classes

**Recommendation:** Option A for now—only one field exists. Revisit if more game-specific settings emerge.

---

### 6. ~~No Profile Export/Import~~ ✓ DONE

**Status:** Resolved 2026-01-30

Enhanced existing Save/Load Aircraft FFB functionality:

- **Export format** now includes `ExportedProfile` wrapper with:
  - Version number for future compatibility
  - ProfileKey (gameId::carId)
  - GraphPath (which graph the profile was created for)
  - ExportedAt timestamp
  - Profile data (AircraftFfbProfile)
- **Import** handles both new and legacy formats (backward compatible)
- **Graph mismatch warning** when loading a profile created for a different graph
- **Overwrite confirmation** when loading into an existing profile

---

### 7. ~~No Reset to Defaults~~ ✓ DONE

**Status:** Resolved 2026-01-30

Added "Reset to Defaults" button next to Save/Load Aircraft FFB buttons:

- Clears `GraphParamValues` for the current vehicle profile
- Shows confirmation dialog before reset
- Rebuilds params from graph defaults and refreshes UI

---

### 8. No Profile Deletion

**Current:** Profiles accumulate forever in `AircraftFfbProfiles`.

**Problem:** Users who try many vehicles build up cruft.

**Recommendation:** Superseded by #9 (Unified Profile Browser).

**Scope:** Low priority.

---

### 9. Unified Profile Browser (Future)

**Current UX problems:**

1. Template selector (new vehicle prompt) only offers graph files, not existing tuned profiles
2. "Save/Load Aircraft FFB" buttons are ambiguous—sounds like internal save, not file export
3. No visibility into stored profiles (can't browse, delete, or copy from other vehicles)
4. Profile management buttons are buried below System Parameters

**Proposal: Profile Browser Dialog**

A unified dialog that serves multiple purposes:

| Context | User Action | Dialog Mode |
|---------|-------------|-------------|
| New vehicle detected | "Select starting point" | Template picker |
| User clicks "Manage Profiles" | Browse/delete/export profiles | Profile manager |
| User wants to copy tuning | "Copy from another vehicle" | Profile picker |

**Dialog layout concept:**

```
┌─────────────────────────────────────────────────────────┐
│  Profile Browser                               [X]      │
├─────────────────────────────────────────────────────────┤
│  [Templates]  [My Vehicles]  [Import File...]           │
├─────────────────────────────────────────────────────────┤
│  ┌───────────────────────────────────────────────────┐  │
│  │ Name              │ Graph           │ Tuned │ Del │  │
│  ├───────────────────┼─────────────────┼───────┼─────┤  │
│  │ Cessna 172        │ xplane_GA.json  │  ✓    │ [X] │  │
│  │ Baron 58          │ xplane_GA.json  │  ✓    │ [X] │  │
│  │ R22 Beta II       │ xplane_heli.json│  ✓    │ [X] │  │
│  │ (New Vehicle)     │ —               │  —    │  —  │  │
│  └───────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────┤
│  Selected: Cessna 172                                   │
│  Graph: graphs/xplane_GA.json                           │
│  Tuning: 3 parameters customized                        │
│                                                         │
│  [Use Graph Only]  [Use Graph + Tuning]  [Export...]    │
└─────────────────────────────────────────────────────────┘
```

**Tabs:**

- **Templates**: Graph files from `graphs/` folder (no tuning, just defaults)
- **My Vehicles**: Stored profiles with tuning (from `AircraftFfbProfiles`)
- **Import File...**: Opens file dialog to load exported profile

**Actions depending on context:**

| Context | "Use Graph Only" | "Use Graph + Tuning" |
|---------|------------------|----------------------|
| New vehicle setup | Apply graph, use defaults | Apply graph + copy params |
| Copying to current | Change graph, reset params | Change graph + copy params |
| Export | n/a | Save to file |

**Benefits:**

1. Single UI for template selection, profile management, and copying
2. Clear distinction between "graph only" vs "graph + tuning"
3. Users can see all their profiles and clean up old ones
4. "Copy from vehicle" workflow becomes obvious
5. Replaces confusing "Save/Load Aircraft FFB" buttons

**Implementation notes:**

- Reuse `ExportedProfile` format for the export action
- Add `LastUsed` timestamp to `AircraftFfbProfile` for sorting
- Dialog can be opened from: new vehicle prompt, "Manage Profiles" button, menu
- Keep existing buttons temporarily for backward compatibility, deprecate later

**Scope:** Medium effort, significant UX improvement. Consider for future release.

---

## Priority

| Issue | Priority | Effort | Notes |
| ----- | -------- | ------ | ----- |
| #2 Pending params file | ~~High~~ | ~~Low~~ | ✓ Done |
| #1 Key inconsistency | ~~High~~ | ~~Medium~~ | ✓ Done |
| #4 Tier 2 deprecation | ~~Medium~~ | ~~Medium~~ | ✓ Done |
| #7 Reset to defaults | ~~Medium~~ | ~~Low~~ | ✓ Done |
| #3 FunctionFfbSettings | ~~Low~~ | ~~Low~~ | ✓ Done (removed) |
| #5 Game-specific fields | Low | - | Monitor only |
| #6 Export/import | ~~Low~~ | ~~Medium~~ | ✓ Done |
| #8 Profile deletion | ~~Low~~ | ~~Medium~~ | Superseded by #9 |
| #9 Unified Profile Browser | Medium | High | Future UX improvement |

## Implementation Order

1. ~~**Remove pending params file**~~ ✓ Done
2. ~~**Fix key inconsistency**~~ ✓ Done
3. ~~**Add reset to defaults**~~ ✓ Done
4. ~~**Tier 2 deprecation**~~ ✓ Done
5. ~~**FunctionFfbSettings decision**~~ ✓ Done (removed)
6. ~~**Export/import**~~ ✓ Done
7. ~~**Profile deletion**~~ — Superseded by #9
8. **Unified Profile Browser** — Future UX overhaul

## Related Documents

- [15_Graph_Param_Override_Migration_Plan.md](15_Graph_Param_Override_Migration_Plan.md)
- [16_Vehicle_Profile_Lifecycle.md](16_Vehicle_Profile_Lifecycle.md)
- [19_Shared_Graph_Save_Protection.md](19_Shared_Graph_Save_Protection.md) — Warn when saving shared graphs
