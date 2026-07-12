# Profile Manager Polish

## Problem Statement

The Profile Manager (`ProfileBrowser/ProfileBrowserDialog`, recently renamed from "Profile Browser") does its job but is rough around the edges. It grew feature-by-feature — templates, stored profiles, import/export, staged imports, tuning inheritance, custom graphs, current-vehicle highlight — and the result is a crowded, inconsistent surface with an over-complex action model. This plan catalogs the rough edges and proposes a phased cleanup.

It is a **UX/polish** plan. The result contract the dialog returns (`SelectedEntry`, `UseSourceGraph`, `UseTuning`, `UseGraphOnly`) already models the two real axes (which graph, whether to copy tuning) and is consumed by `ApplyProfileFromBrowser` / `PromptForGraphTemplate`. Keeping that contract stable means most of this is presentation, not plumbing.

## Current State (as built)

- **Window**: fixed `Width=731`, `MinHeight=520`, resizable.
- **Tab bar** (one horizontal strip): `Templates` / `My Vehicles` radios **+** `Import...` **+** `Custom Graph...` buttons.
- **Game filter** combo — only visible on `My Vehicles`.
- **List**: 4-column item template — name/game/description, graph filename (120px), "Tuned" badge (50px), per-row delete "X" (28px); plus the new "Current" badge.
- **Details panel**: a 3-line strip — Selected / Graph / Tuning.
- **Action row** (7 buttons, several conditional): `Cancel`, `Save to Library`, `Delete`, `Export...`, `Use Graph Only`, `Use Tuning Only`, `Use Graph + Tuning`.
- Modes: `NewVehicle` ("Select Starting Point"), `ManageProfiles` ("Profile Manager"), `CopyFromVehicle` ("Copy From Vehicle").

## Rough Edges (catalog)

### A. Action model (the biggest problem)
1. **Seven bottom buttons, most conditional.** `Save to Library` shows only when the game filter is `<Imports>` (`UpdateButtonStates:395-397`); `Delete` only on `My Vehicles`; the three "Use…" verbs hide for templates (`420-429`). The row reflows as selection/tab changes — jarring, and the primary action isn't obvious.
2. **Three overlapping "apply" verbs.** `Use Graph Only` / `Use Graph + Tuning` / `Use Tuning Only` differ only by two booleans (`UseSourceGraph`, `UseTuning`). Jargon-y and hard to reason about.
3. **Double-click applies a fourth, implicit rule** (`OnItemDoubleClick:432-441`): template → graph-only, else tuning — matches none of the buttons and ignores the `UseSourceGraph` nuance.
4. **Two delete affordances.** Per-row "X" (`OnDeleteClick`, single/immediate for imports, confirm for stored) **and** a bottom `Delete` (`OnDeleteSelectedClick`, multi + confirm). Redundant, subtly different code paths.

### B. Selection model
5. **Multi-select is legitimate for *manage* actions but unclear.** `SelectionMode=Extended` is correct — Delete and Export operate on a set of profiles. The problem is that the *apply* actions ("Use…") silently grey out when `count != 1` with no explanation, and the two kinds of action (single-item "apply this to my vehicle" vs multi-item "delete/export these") aren't visually distinguished. Also inconsistent: Delete already handles multi (`OnDeleteSelectedClick`) but Export is single-only (`OnExportClick` reads `SelectedItem`).

### C. Information architecture
6. **Navigation and actions are mixed** in the tab strip (radios next to Import/Custom Graph buttons) with no visual separation.
7. **Starting points are split across tabs.** Setting up a vehicle, you compare template vs another-vehicle vs custom-file — but they live on different tabs, forcing tab-hopping.
8. **Filtering is inconsistent.** `My Vehicles` has a game combo; `Templates` uses an *invisible* implicit game filter (`GetTemplates(gameId)`).
9. **Staged-import flow is hidden.** Import stages entries; you must switch to the `<Imports>` pseudo-filter and click `Save to Library` — a two-step most users won't discover, and staged imports vanish on close with no warning.
10. **No text search** for long libraries — only a game combo.

### D. Visual / layout
11. **Cramped details strip.** Three lines (Selected/Graph/Tuning) under-use space and print the **raw** graph path (can be long/ugly).
12. **Weak source signaling.** Only "Tuned"/"Current" text badges distinguish rows; no icons or grouping for template vs stored vs custom.
13. **Fixed column widths** truncate long graph names; no column headers.
14. **Custom graphs are invisible after assignment.** A referenced/copied custom graph is never listed (no "Custom" category), so it can't be re-selected, previewed, or deleted from here.
15. **Magic `Width=731`**, no empty-state messaging ("No saved vehicles", "No templates for this game").

### E. Consistency / keyboard
16. **Mixed modality.** Delete/Export/Save-to-Library mutate settings *immediately*, while "Use…" returns a result to apply on close. Cancel discards the pick but keeps the already-applied deletes — conceptually muddy.
17. **No keyboard affordances.** `Cancel` lacks `IsCancel="True"` (the sibling `SharedGraphSaveDialog` sets it); no `IsDefault` primary; no Enter/Esc/Del handling.

## Proposed Improvements

### Phase 1 — Collapse the action model (highest value, contained)
- **One primary action + two inheritance toggles.** Replace the three "Use…" buttons with a single **`Apply to <vehicle>`** primary button plus two checkboxes in the details pane: **☑ Use this graph** and **☐ Copy tuning** (each auto-enabled/disabled by what the selected source offers — templates have no tuning). These map directly onto the existing `UseSourceGraph` / `UseTuning` result flags, so no downstream change.
  - *Alternative:* a split-button `Apply ▾` with the three variants. The toggles are clearer; recommend toggles.
- **Keep multi-select; make the two action kinds explicit — and separate them spatially.** Retain `SelectionMode=Extended` — Delete and Export legitimately act on a set. Distinguish the kinds instead:
  - *Apply* (single-item): a dedicated **"apply target" region pinned above the list**, populated only when exactly one profile is selected. It shows the target headline (name → *current vehicle*), the **Use graph / Copy tuning** toggles, and the **Apply** button. When 0 or >1 rows are selected it collapses to a hint ("Select one profile to apply"), so the region's presence itself signals apply-mode vs manage-mode. (This is *not* list reordering — the list stays put; the target is a separate pinned card.)
  - *Manage* (multi-item): **Delete** and **Export** live at the bottom and act on the whole selection. Make **Export multi-capable** too (batch-export each selected exportable profile), matching Delete, so the manage actions behave consistently.
  - Net effect: Apply (single target) sits up top with its context; manage actions sit at the bottom against the list selection — a clean spatial split of the two action kinds. Keep the target card compact (1–2 lines) so it doesn't duplicate the detail pane or eat the list's vertical space; watch the Apply-top / Cancel-bottom split when mocking.
- **One delete affordance.** Keep the selection-driven bottom `Delete`, remove the per-row "X" (or vice-versa) — not both.
- **Consistent double-click** = the primary (single-item) Apply action, respecting the toggles — not a separate implicit rule.
- **Keyboard**: `IsCancel` on Cancel, `IsDefault` on the primary, `Delete` key deletes the selection.

### Phase 1b — Choose the apply destination
- **Problem**: apply is hard-wired to the active vehicle (`ApplyProfileFromBrowser` writes to `activeGameId/activeCarId` only). You can't configure a profile you're not currently in — e.g. "assign a graph/tuning to a specific vehicle before it becomes active."
- **Add an "Apply to" destination selector** to the apply-target card: a dropdown listing all stored vehicle profiles, defaulting to the active vehicle. The **list selection stays the *source*** (template / custom / another vehicle); the **dropdown is the *destination***. One selection state, so there's no source-vs-target ambiguity in the list. (A right-click "Apply to this vehicle…" accelerator can follow later, but the dropdown carries the feature — a first-class input shouldn't hide behind a gesture.)
- **Behavior split by destination:**
  - Destination **== active vehicle** → apply as today: persist settings **and** reload the runtime graph (`ResolveActiveGraph`).
  - Destination **≠ active vehicle** → write `GraphPath` / `GraphParamValues` into that profile's stored settings **only**; **no runtime reload**. Takes effect when that vehicle next becomes active.
- **Subsumes the no-active-vehicle case** (Phase 1's amber notice): with no session the dropdown has no active default — the user simply picks a destination and apply persists to it. The amber "nothing to apply to" state only remains if there are also no stored profiles to pick.
- **Contract impact**: the result contract must convey the destination. Cleanest split — for a **non-active** destination the dialog performs the apply itself (it already mutates settings directly for delete/save-to-library); for the **active** destination keep the return-to-caller path so the runtime reload still happens. Introduce a plugin method `ApplyProfileToVehicle(gameId, carId, graphPath, sourceProfile, useTuning)` that both paths call; the active path additionally triggers `ResolveActiveGraph` + `BuildGraphParams` (i.e. today's `ApplyProfileFromBrowser` becomes a thin wrapper that passes the active key).
- **Destination list** = `Settings.AircraftFfbProfiles` keys, formatted like the browser entries; always include the active vehicle even if it has no stored profile yet.

### Phase 2 — Information architecture
- **Unify starting points.** Present `Templates` / `My Vehicles` / **`Custom`** as a left-hand segmented list/category rail rather than radio tabs, so all sources are one glance apart. Add the new **Custom** category listing graphs under `graphs/custom/` (from Plan 33's library) so copied customs are browsable, re-selectable, and deletable.
- **Persistent search box** filtering by name/game across the active category.
- **Consistent filtering**: expose (or remove) the implicit template game filter so both categories behave the same.
- **Direct import.** Make `Import...` land straight into the library (with the existing dup-overwrite prompt), eliminating the staged `<Imports>` pseudo-filter and the hidden second step. (If staging is worth keeping, surface a clear "N imported profiles not yet saved" banner instead.)

### Phase 3 — Details / preview pane
- Replace the 3-line strip with a richer right-hand pane: friendly source label, **graph filename** (full path on tooltip), a **"file missing" warning** when the referenced graph doesn't resolve on disk, tuning summary, and the target vehicle for the action.
- *Stretch:* a small read-only graph preview (reuse `GraphPreviewEvaluator`/thumbnail) or the tuned-param list.

### Phase 4 — Visual polish
- Source-type icons; responsive widths (drop the `731` magic number); column headers or fold the columns into the details pane; friendly empty states; consistent button styling with the rest of the plugin.

## Suggested layout (Phase 1–3 target)

```
┌─ Profile Manager ──────────────────────────────────────────┐
│ [search…]                                                   │
│ ┌─ Apply (1 selected) ────────────────────────────────────┐ │
│ │ Cessna 172   (source: template · vehicle_default.json)  │ │
│ │ ☑ Use graph  ☐ Copy tuning   Apply to:[Active ▾] [Apply]│ │
│ └─────────────────────────────────────────────────────────┘ │
│   (0 or >1 selected → "Select one profile to apply";        │
│    no active vehicle + none stored → amber "nothing to apply")│
│ ┌────────────┐  ┌───────────────────────────────────────┐   │
│ │ Templates  │  │ ▸ Cessna 172            [Current]      │   │
│ │ My Vehicles│  │   XPlane · vehicle_default.json  Tuned │   │
│ │ Custom     │  │ ▸ …                                    │   │
│ │ [Import…]  │  │                                        │   │
│ │ [Custom…]  │  └───────────────────────────────────────┘   │
│ └────────────┘   (details / missing-file warning inline)    │
│                            [Delete] [Export…]        [Cancel]│
└─────────────────────────────────────────────────────────────┘
```

Apply lives in the pinned target card up top (single-item); Delete/Export/Cancel sit at the bottom (multi-item / dismiss).

## Files touched

| File | Change |
|------|--------|
| `ProfileBrowser/ProfileBrowserDialog.xaml` | Layout rework: apply card + **"Apply to" destination combo**, search box, category rail, richer details pane, collapsed action row |
| `ProfileBrowser/ProfileBrowserDialog.xaml.cs` | One Apply + toggles, **destination selector + non-active apply path**, unified categories, direct import, keyboard, Custom listing |
| `ProfileBrowser/ProfileBrowserEntry.cs` | Source icon/label, `GraphFileExists` flag, primary-action-enabled helpers |
| `DiyFfbPlugin.cs` | **`ApplyProfileToVehicle(gameId, carId, …)`** (arbitrary destination); `ApplyProfileFromBrowser` becomes a wrapper passing the active key |
| `GraphEditor/GraphPathUtil.cs` | (Phase 2) enumerate `graphs/custom/` for the Custom category |
| `DiyFfbPluginUI.xaml.cs` | `ShowProfileBrowser`: apply the returned selection only when the destination is the active vehicle; non-active applies are performed inside the dialog |

The Phase-1 result contract (`SelectedEntry` + `UseSourceGraph` + `UseTuning`) still drives the **active-vehicle** apply. Phase 1b adds the destination: for a non-active destination the dialog applies directly via `ApplyProfileToVehicle` and returns no "apply" result (so the caller doesn't double-apply to the active vehicle).

## Out of scope
- Plan 33 (custom graph assignment) and Plan 34 (per-profile MSFS var bindings) — separate.
- Bundling graph bytes into exported profiles (the known export-portability limitation).

## Testing
- Unit: game/text filter selection, import dup-detection, custom-graph enumeration (extend `GraphTest`).
- Manual: each mode (new vehicle / manage / copy-from), apply with/without tuning, **apply to the active vehicle (reloads runtime) vs a non-active profile (persists only, no reload; takes effect on next activation)**, destination = no-active-vehicle then pick a stored profile, **multi-select delete and multi-select (batch) export**, apply disabled + hint when 0 or >1 selected, import (direct), a missing-graph profile (should warn), keyboard (Enter/Esc/Del).

## Suggested sequencing
Phase 1 first — it removes the most confusion for the least code and doesn't touch the result contract. Phases 2–4 are independent and can land incrementally.
