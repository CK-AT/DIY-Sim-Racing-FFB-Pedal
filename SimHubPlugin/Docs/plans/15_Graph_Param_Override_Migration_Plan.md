# 15_Graph_Param_Override_Migration_Plan

Date: 2026-01-29
Owner: Codex
Status: Implemented (2026-01-30)

## Goal

Preserve vehicle parameter overrides when graphs change (template swap or file edits).

## Scope

**In scope:**

- Per-vehicle override storage independent of graph file
- Keep orphaned overrides (params no longer in graph) for recovery
- Notify user when defaults change; provide review window
- Update docs and tests

**Out of scope:**

- Firmware changes
- Stable param IDs or alias systems (name-based matching only)

## Current Behavior

- Overrides resolve via three tiers: param default → graph ParamValues → profile overrides
- Graph file swaps can lose or mismatch ParamValues

## Design

### Storage

- Per-vehicle profile holds `Dictionary<string, double>` of overrides
- Auto-create profile on first param edit if missing
- Overrides persist by param name; unmatched keys stay as orphans

### On Graph Change

1. Compute content hash of root graph + all includes
2. Compare to stored hashes
3. If changed:
   - Match override keys to new param names
   - Clamp values to new min/max
   - Keep unmatched keys (orphans) silently
   - Show brief notification only if defaults/ranges changed or new orphans appeared
4. Update stored hashes

### Parameter Review Window (on-demand)

A non-modal window (like live preview) showing:

- Current graph path in header
- "Mark reviewed" button to clear change highlights until next actual change

Two sections:

**Active Parameters:**

| Param | Value | Default | Range | [Reset] |

- Highlight params where default changed since last load
- Highlight clamped values, show original value and which limit was hit (e.g., "150 → 100 (max)")
- "Reset all to defaults" button

**Orphaned Overrides** (collapsible):

| Param | Value | [Delete] |

- Shows overrides with no matching param in current graph
- Users can delete individually or "Delete all orphans" for bulk cleanup

### What We Skip

- No stable `GraphParam.Id` — match by name only
- No alias/rename mapping — if name changes, old override becomes orphan
- No blocking migration dialogs — notification + on-demand review

### Design Decision: Hash-Based vs Snapshot All

**Alternative considered:** Snapshot all param values when template is first assigned.

**Why hash-based instead:**

- Smaller storage footprint (only store changed values)
- Users see which params they've actually tuned vs defaults
- Explicit review of default changes is better UX than silent isolation
- Can still "lock in" current values via future "Snapshot All" action if needed

The hash-based approach notifies users when defaults change, letting them consciously decide whether to adopt new defaults or keep current values.

## Touch Points

**Code:**

- `DiyFfbPlugin.cs` — hash check on graph load, notification trigger
- `DiyFfbPluginSettings.cs` — add `GraphHashes`, keep orphans in existing override dict
- `DiyFfbPluginUI.xaml.cs` — parameter review window

**Docs:**

- `FFB_Graph_Design.md`, `FFB_Graph_Progress.md`

## Tests

- Override persists across graph reload (same params)
- Override clamped when new range is tighter
- Orphan kept when param removed, restored when param returns
- Hash change detected on nested include edit

## Risks

| Risk                         | Mitigation                                 |
| ---------------------------- | ------------------------------------------ |
| Override lost on param rename | Stays as orphan; user can manually reset  |
| Stale UI after migration     | Force refresh on graph change              |

## Done Definition

- Overrides survive graph swaps with name-based matching
- Orphans preserved silently
- Non-blocking notification on default changes
- Review window available on demand
- Tests and docs updated
