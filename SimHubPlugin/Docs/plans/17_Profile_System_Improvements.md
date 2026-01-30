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

### 3. FunctionFfbSettings Stub

**Current:** Structure exists but `CopyFrom`/`ApplyTo` methods are empty.

**Problem:** Dead code creates confusion. Unclear if it's planned or abandoned.

**Options:**

- A) Implement it (per-control FFB settings like gain, curve, deadzone)
- B) Remove it entirely

**Recommendation:** Decide scope. If per-control settings are planned, keep and document. If not, remove the four `FunctionFfbSettings` fields from `AircraftFfbProfile`.

---

### 4. Tier 2 ParamValues Deprecation

**Current:** Three-tier param resolution:

1. Param default (graph definition)
2. Graph-level ParamValues (stored in graph JSON)
3. Profile GraphParamValues (per-vehicle)

**Problem:** Tier 2 and Tier 3 overlap. Editing in UI writes to Tier 3, but graph files can have Tier 2 values. Confusing.

**Recommendation:** Per migration plan (#15):

- Treat Tier 2 as "template defaults" only (read-only after graph creation)
- All user edits go to Tier 3 (profile)
- Document this clearly

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

### 6. No Profile Export/Import

**Current:** Only the entire settings file can be backed up.

**Problem:** Users can't share individual vehicle configs or back up specific profiles.

**Recommendation:** Add UI buttons:

- "Export Profile" → saves `AircraftFfbProfile` + graph path to JSON file
- "Import Profile" → loads and applies (with overwrite confirmation)

**Scope:** Low priority, nice-to-have.

---

### 7. No Reset to Defaults

**Current:** No way to wipe a vehicle profile and start fresh.

**Recommendation:** Add "Reset to Defaults" button that:

- Clears `GraphParamValues` for the vehicle
- Optionally resets graph path to game default
- Prompts for confirmation

---

### 8. No Profile Deletion

**Current:** Profiles accumulate forever in `AircraftFfbProfiles`.

**Problem:** Users who try many vehicles build up cruft.

**Recommendation:** Add profile management UI:

- List all stored profiles
- Allow deletion (with confirmation)
- Show last-used date if tracked

**Scope:** Low priority.

---

## Priority

| Issue | Priority | Effort | Notes |
| ----- | -------- | ------ | ----- |
| #2 Pending params file | ~~High~~ | ~~Low~~ | ✓ Done |
| #1 Key inconsistency | ~~High~~ | ~~Medium~~ | ✓ Done |
| #4 Tier 2 deprecation | Medium | Medium | Part of migration plan #15 |
| #7 Reset to defaults | Medium | Low | Simple UI addition |
| #3 FunctionFfbSettings | Low | Low | Decide and act |
| #5 Game-specific fields | Low | - | Monitor only |
| #6 Export/import | Low | Medium | Nice-to-have |
| #8 Profile deletion | Low | Medium | Nice-to-have |

## Implementation Order

1. ~~**Remove pending params file**~~ ✓ Done
2. ~~**Fix key inconsistency**~~ ✓ Done
3. **Add reset to defaults** — Small UX win
4. **Tier 2 deprecation** — Align with migration plan #15
5. **FunctionFfbSettings decision** — Remove or implement
6. **Export/import, deletion** — Future polish

## Related Documents

- [15_Graph_Param_Override_Migration_Plan.md](15_Graph_Param_Override_Migration_Plan.md)
- [16_Vehicle_Profile_Lifecycle.md](16_Vehicle_Profile_Lifecycle.md)
