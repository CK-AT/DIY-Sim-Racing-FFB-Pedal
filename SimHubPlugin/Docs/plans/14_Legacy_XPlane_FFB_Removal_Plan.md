# 14_Legacy_XPlane_FFB_Removal_Plan

Date: 2026-01-29
Owner: Codex
Status: Draft

## Goal
Remove legacy X‑Plane FFB processing from the SimHub plugin and rely on the graph-based runtime for FFB generation. Update documentation to reflect the new single path (docs updated; legacy docs revised rather than deleted). No dead code shall remain.

## Scope
In scope:
- Remove legacy X-Plane FFB computation codepaths and related settings/UI hooks.
- Keep graph runtime as the single source of truth for X-Plane FFB outputs.
- Update documentation to describe graph-based processing only; revise legacy docs to historical notes if needed.

Out of scope:
- Changes to ESP32 firmware.
- Changes to non-X-Plane games unless directly tied to shared code paths.

## Inventory (expected touch points)
Code:
- `SimHubPlugin/DiyFfbPlugin.cs` (legacy FFB math/branching, trim/vane/buffet paths)
- `SimHubPlugin/XPlaneFfbMath.cs`
- `SimHubPlugin/XPlaneFfbGraph.cs`
- `SimHubPlugin/DiyFfbPluginSettings.cs` (legacy tuning knobs)
- `SimHubPlugin/FlightStickConfigControl.xaml` / `.xaml.cs`
- `SimHubPlugin/FlightPedalsConfigControl.xaml` / `.xaml.cs`
- Any legacy UI labels/tooltips that refer to non‑graph tuning
 - `SimHubPlugin/FunctionConfigControl.xaml.cs` (RefreshXPlaneFfbSettings plumbing)
 - `SimHubPlugin/DiyFfbPluginUI.xaml` / `.xaml.cs` (X‑Plane UDP/system selectors)

Docs (update only, do not delete):
- `SimHubPlugin/Docs/XPlane_FFB.md`
- `SimHubPlugin/Docs/XPlane_FFB.html`
- `SimHubPlugin/Docs/FFB_Design_Legacy.md`
- `SimHubPlugin/Docs/FFB_Graph_Design.md` (ensure graph is the sole FFB path)
- `SimHubPlugin/Docs/FFB_Graph_Progress.md` (progress update)
- `SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md` (verify signals align)

Tests:
- `SimHubPlugin/GraphTest/GraphTest.csproj` (if any graph/runtime coverage needs extension)

## Detailed inventory (from repo scan)
Legacy runtime/math and diagnostics:
- `DiyFfbPlugin.ProcessXPlaneFfb` and its helpers: `GetXPlaneFfbParams`, `IsXPlaneFfbEnabled`,
  `IsXPlaneFfbDefault`, `TryGetXPlaneFfbDiagnostics`, `UpdateXPlaneDiagnostics`.
- `XPlaneFfbMath` and `XPlaneFfbGraph` classes (graph visualization and q‑scale helpers).
- `XPlaneFfbParams` / `XPlaneFfbDiagnostics` structs and related fields (per‑axis diagnostics cache).
- Settings fields and defaults: `XPlaneFfbKq`, `XPlaneFfbKrate`, `XPlaneFfbKcenter`,
  `XPlaneFfbEnabled`, `XPlaneBuffet*`, `XPlaneWeathervaneGain`, `XPlaneAeroMomentGain`,
  `XPlaneFriction*`, `XPlaneRpmBlend`, `XPlaneLoadForceClamp`, `XPlaneReferenceFlightMode`,
  `XPlaneTorqueRefNm` (in `DiyFfbPluginSettings`).

UI wiring still referencing legacy settings:
- `FlightStickConfigControl.xaml.cs`: `Toggle_ffb_enabled`, `UpdateXPlaneSettingsUi`,
  `RefreshXPlaneFfbSettings`, X‑Plane timer plumbing (now mostly graph output display).
- `FlightPedalsConfigControl.xaml.cs`: X‑Plane tuning sliders/handlers and auto‑tune hook
  (mostly stubbed but still wired).
- `FunctionConfigControl.xaml.cs`: forwards `RefreshXPlaneFfbSettings`.
- `DiyFfbPluginUI.xaml/.cs`: X‑Plane UDP, rotor, aircraft type, Vref, nominal RPM, torque ref UI.

## Plan
1) Baseline review
   - Identify all legacy X-Plane FFB calculation paths and the decision points that select them.
   - Confirm current graph runtime entry point and output application path.

2) Remove legacy runtime path
   - Delete or disable legacy computation methods and any branching that selects them.
   - Ensure graph outputs are the only source for spring/damper/friction/load/trim/buffet.
   - Keep diagnostics that are still useful with graph runtime; remove those tied strictly to legacy math.

3) Settings and UI cleanup
   - Remove legacy tuning settings (e.g., k_q, k_rate, buffet curves) if they are not used by graph runtime.
   - Update UI labels and tooltips to describe graph-driven outputs only.
   - Ensure inspector inputs still commit on Enter per AGENTS rule if any text fields are touched.

4) Documentation updates (no deletions)
   - Update X-Plane docs to describe graph-based FFB and remove legacy tuning guidance.
   - Convert `FFB_Design_Legacy.md` into historical/reference notes and clearly mark it as deprecated.
   - Update graph design/progress docs to reflect removal of legacy path.

5) Verification
   - Build plugin (MSBuild.exe command from AGENTS).
   - Run GraphTest as applicable.
   - Sanity check that X-Plane FFB outputs still flow via graph runtime.

## Risks / Mitigations
- Risk: Removing legacy settings could break saved configs.
  - Mitigation: Add migration notes in docs and keep minimal compatibility mapping if needed.
- Risk: UI controls rely on removed settings.
  - Mitigation: Remove bindings and update view models accordingly.
- Risk: Diagnostics expect legacy values.
  - Mitigation: Keep diagnostics that can be fed from graph outputs or clearly mark as unavailable.

## Open Questions
- Do you want to keep any legacy diagnostics panel fields for reference?
- Should we keep deprecated settings in the config schema but ignore them, or fully remove?

## Removal Checklist
Code removals:
- `DiyFfbPlugin.ProcessXPlaneFfb` and all callers.
- `XPlaneFfbMath` and `XPlaneFfbGraph` classes; remove from `DiyFfbPlugin.csproj`.
- `XPlaneFfbParams` / `XPlaneFfbDiagnostics` structs and cached fields in `DiyFfbPlugin`.
- `TryGetXPlaneFfbDiagnostics`, `UpdateXPlaneDiagnostics`, `IsXPlaneFfbEnabled`,
  `IsXPlaneFfbDefault`, `GetXPlaneFfbParams`.
- Legacy scale normalization logic tied to `XPlaneFfbKq/Krate` defaults.

Settings cleanup:
- Remove legacy X‑Plane FFB settings from `DiyFfbPluginSettings` (and any migration helpers):
  `XPlaneFfbEnabled`, `XPlaneFfbKq`, `XPlaneFfbKrate`, `XPlaneFfbKcenter`,
  `XPlaneBuffetStartDeg`, `XPlaneBuffetFullDeg`, `XPlaneBuffetGain`,
  `XPlaneWeathervaneGain`, `XPlaneAeroMomentGain`,
  `XPlaneFrictionQ`, `XPlaneFrictionTorque`, `XPlaneFrictionLowRpm`,
  `XPlaneRpmBlend`, `XPlaneLoadForceClamp`, `XPlaneReferenceFlightMode`,
  `XPlaneTorqueRefNm`.

UI cleanup:
- Remove legacy tuning controls and handlers in:
  `FlightStickConfigControl.xaml/.cs`, `FlightPedalsConfigControl.xaml/.cs`.
- Remove `RefreshXPlaneFfbSettings` plumbing in `FunctionConfigControl.xaml.cs`.
- Remove X‑Plane “system/tuning” widgets in `DiyFfbPluginUI.xaml/.cs`
  (UDP enable + port, rotor selector, aircraft type, Vref/RPM/torque ref inputs).

Docs:
- Update `XPlane_FFB.md/html` to graph‑only workflow.
- Mark `FFB_Design_Legacy.md` as deprecated/historical.
- Update `FFB_Graph_Design.md` + `FFB_Graph_Progress.md` for removal completion.

## Migration / Compatibility Notes
- Existing configs with legacy X‑Plane tuning values will no longer be used.
- UI should no longer show legacy tuning sliders/toggles; graph parameters replace them.
- If any legacy values must be surfaced, document the equivalent graph parameters.

## Done Definition
- Legacy X-Plane FFB processing code removed or unreachable.
- Graph runtime is the only FFB source for X-Plane.
- Docs updated to reflect new path; legacy docs marked deprecated but retained.
- Build passes and basic graph tests run.
