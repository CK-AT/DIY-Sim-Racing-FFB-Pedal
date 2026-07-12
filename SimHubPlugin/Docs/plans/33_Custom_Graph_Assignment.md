# Custom Graph Assignment to Vehicles

## Problem Statement

A vehicle profile can only be given a graph two ways in the Profile Browser today:

1. Assign a **registry template** directly (`GraphTemplateRegistry.AllTemplates`).
2. Copy the graph (and optionally tuning) **from an existing stored/imported profile** — whose graph is itself usually a template.

There is no way to point a vehicle at an **arbitrary user-authored `.json` graph** from the profile-management surface. The only existing backdoor is buried and conditional: editing a graph that is **shared by more than one vehicle** and hitting Save triggers the `SharedGraphSaveDialog` (`GraphEditorWindow.xaml.cs:571-586`), whose *"Save as Copy..."* button runs `SaveAsCopyForCurrentVehicle()` (`690-756`) to repoint the current vehicle at a fresh copy. For a graph used by only one vehicle the dialog never appears, so most users never encounter this path — it is a save-protection safety net, not a general "assign a custom graph" command.

**Key finding: the data model already supports this.** `AircraftFfbProfile.GraphPath` (`DiyFfbPluginSettings.cs:27`) is a bare file-path string — templates and custom graphs are stored identically. `SetVehicleGraphPath(...)` + `ResolveActiveGraph()` (`DiyFfbPlugin.cs:2426-2529`) already load and hot-swap any `.json` on disk. This is therefore a **UI gap, not an architecture gap.**

## Decision (from design review)

- Add a **"Browse for graph file…"** entry point to the Profile Browser.
- On pick, **offer both** storage modes: *reference in place* vs *copy into a managed library*.

## Design

### Storage modes

When the user picks a `.json`, prompt (radio in a small dialog, or two buttons) for:

- **Reference in place** — store the path relative to the plugin base dir when the file lives under it (reuse the relativizer described below); otherwise store the absolute path. Zero duplication; breaks if the user moves/deletes the file (already handled: `ResolveActiveGraph` records a validation error when the resolved path is missing, `DiyFfbPlugin.cs:2482+`).
- **Copy into managed library** — copy the file into a plugin-managed folder and point the profile at the copy. Self-contained; edits to the original no longer propagate.
  - **Managed folder:** `graphs/custom/` under the plugin base dir (sibling of `graphs/templates/`).
  - **Filename:** derive from the source name; on collision append ` (2)`, ` (3)`, … Never overwrite silently.

### Validation before assign

Before writing `GraphPath`, load-and-validate the file the same way the runtime will:

```
json = File.ReadAllText(path)
graph = GraphSerializer.Deserialize(json, out validation)
```

If `!validation.IsValid`, show the errors in a themed message box and abort the assignment (do not leave the profile pointing at a broken graph). Mirrors the validation already done in `ResolveActiveGraph`.

### Wiring

Route through the existing path — no new runtime plumbing:

```
Browse → (validate) → (reference|copy) → path
      → DiyFfbPluginUI.ShowProfileBrowser applies via
        Plugin.ApplyProfileFromBrowser(path, profile:null, useTuning:false)
      → SetVehicleGraphPath(game, car, path)   [DiyFfbPlugin.cs ~3990]
      → ResolveActiveGraph(game, car)          [DiyFfbPlugin.cs:2426]
```

### Shared path relativizer

`MakeRelativeGraphPath` currently lives privately in `GraphEditorWindow.xaml.cs:761+`. Extract it into a shared static helper (e.g. `GraphPathUtil.MakeRelative(path, baseDir)` in `GraphEditor/`) and call it from both the editor and the new browser flow. Keeps path-storage rules identical everywhere.

### Displaying "custom" vs "template" in the browser (optional polish)

The profile model has no source discriminator, but the browser can derive it: a `GraphPath` that resolves to a file under `graphs/templates/` **and** matches a `GraphTemplateRegistry` entry → show "Template"; otherwise → "Custom". No schema change needed. Nice-to-have, not required for the feature.

## UI changes

- **`ProfileBrowserDialog.xaml` / `.xaml.cs`**
  - Add a **"Browse for graph file…"** button (visible in `NewVehicle` and `ManageProfiles` modes; hidden/irrelevant in `CopyFromVehicle`).
  - Handler: `OpenFileDialog` (filter `Graph JSON (*.json)|*.json`), then storage-mode prompt, then validate, then set `SelectedEntry` to a synthetic entry (new `ProfileBrowserEntry.Source` value `CustomFile`, extending the enum at `ProfileBrowserEntry.cs:10-20`) carrying the resolved `GraphPath` and `UseSourceGraph = true`, `UseTuning = false`.
  - `UpdateButtonStates` (`ProfileBrowserDialog.xaml.cs:399-401`) enables the standard "Use Graph" result for a `CustomFile` entry.
- **`DiyFfbPluginUI.ShowProfileBrowser`** (`DiyFfbPluginUI.xaml.cs:874-925`) — already forwards `entry.GraphPath` when `TemplateEntry == null`; a `CustomFile` entry flows through unchanged. Verify no template-only assumptions on that branch.

## Files touched

| File | Change |
|------|--------|
| `ProfileBrowser/ProfileBrowserEntry.cs` | Add `Source.CustomFile`; ensure `GraphPath` carries the picked path |
| `ProfileBrowser/ProfileBrowserDialog.xaml(.cs)` | "Browse…" button + handler, storage-mode prompt, validation |
| `GraphEditor/GraphPathUtil.cs` (new) | Extracted `MakeRelative(path, baseDir)` |
| `GraphEditor/GraphEditorWindow.xaml.cs` | Use shared relativizer |
| `DiyFfbPlugin.cs` | Small helper `CopyGraphIntoLibrary(sourcePath) → relativePath`; reuse existing `SetVehicleGraphPath` |
| `DiyFfbPluginUI.xaml.cs` | Confirm `ShowProfileBrowser` handles the custom-file entry |

## Edge cases / limitations

- **Profile export portability.** `ExportedProfile` (`DiyFfbPluginSettings.cs:60-67`) carries the `GraphPath` **string only**, not the graph bytes. A *referenced* custom graph will not survive export to another machine; a *library copy* is more robust but export still doesn't bundle the file. Out of scope here — flag as a known limitation (a future "embed graph in exported profile" is a separate item).
- **Missing file at load** — already surfaced as a validation error by `ResolveActiveGraph`; the Browse flow's pre-validation makes it far less likely at assign time.
- **Relative-path portability** across plugin installs — same behavior as templates today.

## Testing

- Unit: `MakeRelative` for paths under/outside base dir, absolute vs relative inputs (extend `GraphTest`/`TestCommon`).
- Unit: `CopyGraphIntoLibrary` collision-suffix logic.
- Manual: assign a hand-authored `.json` to a vehicle via Browse (both modes), confirm `ResolveActiveGraph` loads it and the runtime evaluates it; confirm an invalid graph is rejected with errors and the profile is left unchanged.
