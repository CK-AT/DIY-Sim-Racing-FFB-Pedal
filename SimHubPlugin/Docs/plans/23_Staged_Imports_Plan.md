# Staged Imports Feature

**Status:** ✅ Completed (2026-02-02)

## Goal

Improve the Profile Browser import UX by making imported profiles "staged" with clear visual indication and explicit save action, rather than losing them when switching tabs/filters.

## Implementation Summary

### Features Implemented

1. **Staged imports list** - Imported profiles go to `_stagedImports` instead of being lost on tab/filter change
2. **`<Imports>` filter** - Appears in game dropdown when staged imports exist, auto-selected after import
3. **Game ID display** - Each profile shows its game ID in the list
4. **Save to Library button** - Persists staged imports to the library with duplicate detection
5. **Delete button** - Bulk delete for selected profiles (stored or staged)
6. **Multi-select** - Extended selection mode for bulk operations
7. **Duplicate handling** - Warns before overwriting existing profiles

### Files Modified

| File | Changes |
| ---- | ------- |
| `ProfileBrowserDialog.xaml` | Multi-select, "Save to Library" button, "Delete" button, game ID display |
| `ProfileBrowserDialog.xaml.cs` | `_stagedImports` list, `<Imports>` filter, save/delete logic, duplicate detection |
| `ProfileBrowserEntry.cs` | Extract GameId for imports, `CanDelete` includes imports |

### Button Visibility Summary

| Condition | Save to Library | Delete | Export | Graph Only | Graph+Tuning |
| --------- | --------------- | ------ | ------ | ---------- | ------------ |
| No selection | Hidden | Disabled | Disabled | Disabled | Disabled |
| Single stored profile | Hidden | Enabled | Enabled | Enabled | Enabled |
| Single staged import | Visible+Enabled | Enabled | Enabled | Enabled | Enabled |
| Single template | Hidden | Hidden | Disabled | Enabled | Hidden |
| Multi-select stored | Hidden | Enabled | Disabled | Disabled | Disabled |
| Multi-select staged | Visible+Enabled | Enabled | Disabled | Disabled | Disabled |

### Testing Checklist

- [x] Import multiple profiles from different games
- [x] Verify `<Imports>` filter appears and is auto-selected
- [x] Verify game ID shows on each entry
- [x] Select single import → verify all buttons enabled appropriately
- [x] Select multiple imports → verify only "Save to Library" and "Delete" enabled
- [x] Click "Save to Library" → verify profiles move to regular list
- [x] Verify duplicate warning when saving existing profile
- [x] Verify `<Imports>` filter disappears when all imports saved/deleted
- [x] Switch to Templates tab → verify no data loss when returning to My Vehicles
- [x] Delete stored profiles → verify they don't reappear on filter change
