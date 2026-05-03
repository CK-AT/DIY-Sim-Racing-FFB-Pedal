# Plan 10: Consolidate FlightStick + FlightPedals into FlightControl

## Goal

Replace `FlightStickConfig` and `FlightPedalsConfig` with a single
`FlightControlConfig` message used by all four flight functions:
FlightStickPitch, FlightStickRoll, FlightStickCollective, FlightPedals.
Differentiate by `FunctionID` alone.

This is a follow-on to Plan 02 (which collapsed Pitch/Roll/Collective into one
message). Plan 02 unified three of the four flight-function configs; Plan 10
merges the fourth.

## Motivation

`FlightStickConfig` and `FlightPedalsConfig` describe semantically identical
structures with two cosmetic differences:

| | `FlightStickConfig` | `FlightPedalsConfig` |
|---|---|---|
| Bounds | `pos_min`, `pos_max` | `pos_near_lim`, `pos_far_lim` |
| Damping | ✓ | ✓ |
| Centering spring | ✓ | ✓ |
| `phase_offset`, `vib_harmonic_ratios`, `vib2_harmonic_ratios` | ✓ | — |

The motion-range fields are renamed but represent the same quantity. The
vibration fields are absent from pedals only because the proto was extended for
sticks first — there's no firmware reason pedals can't run a DDS oscillator.

**What this unblocks:** pedal vibration (the immediate driver — see Plan 08
deferral note). Without consolidation, enabling pedal vibration means
duplicating the entire DDS field block into `FlightPedalsConfig`, plus the
parallel processor / registry / merger code in the plugin. Consolidation costs
roughly the same and removes the duplication.

**What stays:** `FunctionID` values, signal groups (`FlightPedals.SpringGain`,
`FlightStickPitch.Vib1Ampl1`, etc.), UI control labels ("Stick" vs "Pedals" in
the UI). Only the wire-level config message type unifies.

**Naming:** `FlightControlConfig`, not `FlightAxisConfig`. "Axis" is overloaded
in this codebase already (`AxisConfig`, `AxisID`, `AxisConfigManager`) and
refers to a different protocol concept — the physical hardware axis, not the
function consuming it. "Control" maps to "flight control" (stick / pedals /
collective are all cockpit controls).

---

## Phase 1: Protobuf Definition

**File: `proto/diy_ffb_protocol.proto`**

- Add new message `FlightControlConfig` with the union of fields:
  ```proto
  message FlightControlConfig {
    float pos_min = 1;
    float pos_max = 2;
    float damping = 3;
    float centering_spring_const = 4;
    float phase_offset = 5;
    repeated float vib_harmonic_ratios = 6;   // length 5
    repeated float vib2_harmonic_ratios = 7;  // length 2
  }
  ```
- Delete `FlightStickConfig` and `FlightPedalsConfig` messages.
- In `FunctionConfig.oneof specific`: replace the two arms with a single
  `FlightControlConfig flight_control = N;` arm. Reserve the old field numbers.
- Nanopb options file: replace the per-message size hints with a single block
  for `FlightControlConfig`.
- **Add `min_damping` to `AxisConfig`.** New field at the next available tag,
  `float min_damping = N;`. Default 0 (proto3 default) preserves "no axis-level
  floor" — opt-in by user. Semantics: integrator-enforced damping floor at the
  `Sim` level, applied to the cumulative `accum.k_damp_sum` regardless of
  which function is active or what frames are flowing. Decoupled from
  `FlightControlConfig.damping` (the rest-state value used when frames stop) —
  see Phase 2.1 for the design rationale.

**Regeneration:**
- C# `DiyFfbProtocol.cs` regenerates via `protoc`.
- ESP32 nanopb regenerates via PlatformIO build.

**Wire compatibility:** Not binary-compatible. New plugin and new firmware
must roll out together. Old `flight_stick`/`flight_pedals` tags are reserved
so re-using them in the future doesn't collide. Coordinated cutover required —
no mixed-version operation.

### Generated C# API changes

- `FunctionConfig.FlightStick` / `.FlightPedals` → `FunctionConfig.FlightControl`
- `SpecificOneofCase.FlightStick` / `.FlightPedals` → `SpecificOneofCase.FlightControl`
- Types `FlightStickConfig`, `FlightPedalsConfig` → `FlightControlConfig`

---

## Phase 2: ESP32 Firmware

The consolidation is not just at the wire-format level — the firmware
function classes also unify. `FlightStickFunction` and `FlightPedalsFunction`
are replaced by a single `FlightControlFunction` class.

**Architectural reminder:** each ESP32 board controls one physical axis and
runs at most one `IFunction` at a time. The four flight FunctionIDs
(FlightStickPitch, FlightStickRoll, FlightStickCollective, FlightPedals)
live on four separate boards. Today's two global instances —
`flight_stick_function` and `flight_pedals_function` in
[Main.cpp:41-42](ESP32/src/Main.cpp#L41-L42) — exist only because the
firmware needs storage for whichever flavor a given board happens to be
configured as; `on_config_update()` selects one of them as
`active_function` based on the `FunctionConfig.specific` oneof tag.

After consolidation there is **one** global `flight_control_function`
instance. It serves all four flight FunctionIDs because any single board
is only ever one of them at a time. No per-FunctionID slots, no
FunctionID parameter on `update_config` / `on_ffb_action` — the class
shape mirrors today's `FlightStickFunction`, just renamed and with the
union of fields.

### 2.0 Class consolidation

**New files:**

- `ESP32/include/FlightControlFunction.h`
- `ESP32/src/FlightControlFunction.cpp`

**Deleted files:**

- `ESP32/include/FlightStickFunction.h`
- `ESP32/src/FlightStickFunction.cpp`
- `ESP32/include/FlightPedalsFunction.h`
- `ESP32/src/FlightPedalsFunction.cpp`
- `FlightStickConfigCommon` struct (or the pedal equivalent) — papered over
  the previous type difference; redundant once configs and classes unify.

**Class shape:**

- `update_config(const FlightControlConfig&)` — single overload, no
  per-flavor variant. Same signature shape as today's
  `FlightStickFunction::update_config`.
- Member physics elements are the union of what stick + pedals had: damper,
  centering spring, friction contribution, buffet, load force, trim offset,
  plus DDS slots (vib1 × 5, vib2 × 2). Pedals previously lacked the DDS
  path; under consolidation they get it for free, inert as long as
  harm-ratio arrays are zeroed.
- `on_ffb_action(const FFBAction&)` — single implementation consuming all
  `FlightFfbAction` fields. The damping-clamp change in Phase 2.1 lands
  here.
- `_ffb_overridden` timeout / fallback (~200 ms) — single implementation,
  restores all physics elements to config defaults on silence.
- Motion-range field references use `pos_min` / `pos_max` exclusively.
  `pos_near_lim` / `pos_far_lim` were just rename-deltas for the pedal
  flavor; gone entirely.

The class needs no `FunctionID` branching for physics — a board configured
as FlightPedals receives a `FlightControlConfig` with zeroed DDS harm-ratios
and behaves identically to today's pedal flavor; a stick board receives a
config with non-zero ratios and gets vibration. The function-flavor
distinction collapses into config values.

**File: `ESP32/src/Main.cpp`**

- Replace the two globals at [Main.cpp:41-42](ESP32/src/Main.cpp#L41-L42)
  with one:
  ```cpp
  FlightControlFunction flight_control_function = {};
  ```
  and update the `function_elements.add_element(...)` calls at
  [Main.cpp:387-390](ESP32/src/Main.cpp#L387-L390) accordingly (one entry
  for `flight_control_function` instead of two).
- Collapse the dispatch in `on_config_update()` at
  [Main.cpp:244-251](ESP32/src/Main.cpp#L244-L251) — the two cases
  `FunctionConfig_flight_pedals_tag` and `FunctionConfig_flight_stick_tag`
  become one:
  ```cpp
  case FunctionConfig_flight_control_tag:
      flight_control_function.update_config(function_cfg->specific.flight_control);
      active_function = &flight_control_function;
      break;
  ```

- Update the `#include` lines at [Main.cpp:33-34](ESP32/src/Main.cpp#L33-L34):
  remove `FlightPedalsFunction.h` and `FlightStickFunction.h`; add
  `FlightControlFunction.h`.

**Risk:** Firmware behavior parity for FlightPedals must remain unchanged
once the consolidation lands (no functional regression). The vibration path
is the only new code on the pedal side; it stays inert as long as harm-ratio
arrays are zeroed (the default for existing pedal profiles). Centering
spring, damper, friction, buffet, load force, trim offset, and motion-range
clamp behavior must produce bit-identical motor output for an unchanged
pedal config compared to the pre-consolidation `FlightPedalsFunction`.

### 2.1 Damping semantic change (safety damper + axis-level floor)

Today, `FlightStickFunction::on_ffb_action()` clamps damping to a floor of
the function-config value:

```cpp
damper.set_k(max(_config.damping, flight.k_damper));
```

This makes `FlightStickConfig.damping` an unconditional minimum — per-frame
`k_damper` cannot push damping below it. That defeats the case where a user
wants high damping at rest (no resonance with hands off the stick) but low
damping during active play (intentional vibrations come through). However,
naively dropping the floor also removes the only safety net against a
buggy graph emitting near-zero damping into a resonance-prone motor.

The plan splits the concerns: **rest-state damping** (per-function, what the
ESP32 falls back to when frames stop) stays in `FlightControlConfig.damping`
but stops acting as a clamp; **safety-floor damping** (per-axis, an
unconditional minimum enforced regardless of function or frame) moves to
`AxisConfig.min_damping` and is applied by `Sim` directly at the
integrator.

**Function-level change.** Drop the `max()` in `FlightControlFunction`.
Per-frame `k_damper` becomes authoritative while frames are flowing:

```cpp
damper.set_k(flight.k_damper);
```

`FlightControlConfig.damping` now means the *rest/safety* damping value
applied when no FlightFFB frames are flowing. The existing `_ffb_overridden`
timeout (~200 ms) already restores it on silence, so this field naturally
becomes the safe-fallback value. This is what the safety-damper toggle
(Phase 3.7) leverages — gating frame emission lets the function fall back
to `FlightControlConfig.damping` cleanly.

**Axis-level floor.** Add enforcement in `Sim::update()` at
[ESP32/src/Physics.cpp:171-173](ESP32/src/Physics.cpp#L171-L173). Insert
the floor *before* the existing stability cap so a misconfigured floor
cannot violate the `1.9 * m / dt` integration stability bound:

```cpp
// Floor: axis-level safety minimum. Unconditional regardless of which
// function is active, what frames are flowing, or what _config.damping says.
if (_min_damping > 0.0f) {
    accum.k_damp_sum = max(accum.k_damp_sum, _min_damping);
}
// Cap: integration-stability ceiling (existing behavior).
if (k_limit > 0.0f) {
    accum.k_damp_sum = min(accum.k_damp_sum, k_limit);
}
if (accum.k_damp_sum > 0.0f) {
    accum.f_sum -= _v * accum.k_damp_sum;
}
```

`Sim` gains a `set_min_damping(float)` setter, called from
`on_config_update()` in `Main.cpp` whenever an `AxisConfig` arrives:

```cpp
sim.set_min_damping(max(axis_cfg->min_damping, 0.0f));
```

This applies to **every** function, not just flight — automotive pedal,
shifter, etc. all get the same axis-level floor for free. Default 0
preserves the prior "no axis-level floor" behavior; a user with a
resonance-prone motor sets `AxisConfig.min_damping` once during axis
tuning and forgets it.

**Why split the floor and the rest-state.** Today these are conflated by
the `max()` clamp — one number does both jobs. Splitting them lets each
serve its proper role:

- *Rest-state* is a per-function value (a stick at rest may want different
  damping than a pedal at rest), persisted with the function config,
  reachable by graph templates if they want to override it.
- *Safety floor* is a per-axis value (a property of the physical motor +
  load + structure, not the logical function), applied at the integrator
  so no function class or graph misconfiguration can defeat it.

**Apply the function-level change** wherever else flight functions clamp
damping similarly (check FlightPedals path post-consolidation; the unified
`FlightControlFunction` should have one damping-application site).

---

## Phase 3: Plugin

### 3.1 OverrideFieldRegistry

**File: `SimHubPlugin/TieredConfig/OverrideFieldRegistry.cs`**

- Collapse the parallel blocks into one set of `FlightControl.*` fields:
  - `FlightControl.PosMin`, `FlightControl.PosMax` (replace `FlightStick.PosMin/Max`
    and `FlightPedals.PosNearLim/PosFarLim`)
  - `FlightControl.Damping`, `FlightControl.CenteringSpringConst`
  - `FlightControl.Vib1Phase`, `FlightControl.Vib1HarmRatio1..5`,
    `FlightControl.Vib2HarmRatio1..2`
- Field group: a single `FlightControl` group that all four flight functions
  match against (as opposed to today's `FlightStick` / `FlightPedals` split).
- **Aliases for backward compat:** register `flight_stick.*` and
  `flight_pedals.*` aliases pointing at the corresponding `flight_control.*`
  field. Old profile JSONs and old graph ConfigOut field paths continue to
  resolve.

### 3.2 FunctionConfigOverrides

**File: `SimHubPlugin/TieredConfig/TieredConfigTypes.cs`**

- Rename `FlightStickMotionRange` → `FlightControlMotionRange`.
  `FlightStickDamping` → `FlightControlDamping`. Etc.
- Rename `FlightPedalsMotionRange` → drop entirely (subsumed). Same for
  `FlightPedalsDamping`, `FlightPedalsCenteringSpringConst`.
- `FlightStickVibHarmonicRatios` → `FlightControlVibHarmonicRatios`. Same for
  `FlightStickVib2HarmonicRatios`, `FlightStickPhaseOffset`.
- Update `IsEmpty` accordingly.

### 3.3 Processors

**Files:** `SimHubPlugin/TieredConfig/FlightStickProcessor.cs`,
`FlightPedalsProcessor.cs`.

- Delete `FlightPedalsProcessor`.
- Rename `FlightStickProcessor` → `FlightControlProcessor`. Make it apply the
  `FlightControlConfig` delta to all four flight functions.
- `ConfigMerger.MergeFunctionConfig`: replace the two parallel processor calls
  with one `FlightControlProcessor`.

### 3.4 Comparer

**File: `SimHubPlugin/TieredConfig/ConfigComparer.cs`**

- Collapse the FlightStick + FlightPedals comparison branches into one
  FlightControl branch.

### 3.5 UI controls

**Files:** `FlightStickConfigControl.xaml.cs`, `FlightPedalsConfigControl.xaml.cs`.

- Keep both controls (the UI presentation differs — pedal-specific labels,
  rudder-brake support, motion range visualization). They write through the
  unified `flight_control.*` field paths via the registry.
- The duplication remaining here is presentation, not logic. Optional later
  consolidation but not required for this plan.

### 3.6 Vehicle profile import path

The "ESP32 config authority" rule (per CLAUDE.md memory: when stored baselines
exist, incoming ESP32 configs are ignored and merged config is pushed back) is
unaffected — same code path, just a different config message type.

### 3.7 Safety damper toggle (paired with Phase 2.1)

With the firmware change in Phase 2.1, gating FlightFFB frame emission for
a flight function causes the ESP32 to fall back to `FlightControlConfig.damping`
(the rest/safety value) within ~200 ms. The plugin already has the gating
mechanism — `SetFunctionOutputDisabled(FunctionID, bool)` checked in
`SendGraphFfbForFunction()` — so the toggle reduces to:

1. **SimHub action.** Register a single global action,
   `FlightControl.SafetyDamper.Toggle`, that flips the safety state for
   *all four* flight FunctionIDs together. No per-function variants — the
   safety damper always engages or disengages as a group, since a partial
   state (some functions emitting, others gated) would leave a mix of
   resonance-prone and resonance-safe behavior across the cockpit and has
   no useful UX. The action calls `SetFunctionOutputDisabled(functionId, engaged)`
   for each of the four flight FunctionIDs. Registration site:
   `DiyFfbPlugin.cs` alongside the existing `*toggle` actions
   (ABStoggle, RPMtoggle, WheelSliptoggle).

2. **UI indicator.** Add a small "Safety damper engaged" badge or toggle
   button to `FlightStickConfigControl.xaml` and (post-consolidation)
   `FlightPedalsConfigControl.xaml`, near the damping slider. Bound to the
   global safety-damper state. Two-way: clicking the toggle in the UI has
   the same effect as the SimHub action.

3. **Default state and persistence.** Safety damper defaults to **ENGAGED**
   on plugin / SimHub start. The user must explicitly disengage to begin
   active play. This is the safe default: a board that boots without an
   active SimHub session, or a SimHub that just launched, holds the
   stick at the rest/safety damping until the user is ready. State is
   runtime-only, not persisted — every SimHub start is engaged again.

4. **Why no separate "safety_damping" field.** The config's `damping` field
   already serves dual duty: it's the rest-state value (when frames stop)
   and the implicit ceiling that graphs should typically not exceed during
   active play. One value, two contexts — no protobuf change needed.

### 3.8 Min-damping UI (paired with Phase 1 + Phase 2.1)

The new `AxisConfig.min_damping` field needs UI exposure. Mirror the
existing OscillationGuard pattern — axis-level control with per-function
override capability — so behavior matches the established UX users
already know.

1. **Axis-level slider.** Add a "Min Damping" slider (or numeric TextBox
   matching the OscillationGuard tab style at
   [AxisConfigControl.xaml:162-213](SimHubPlugin/AxisConfigControl.xaml#L162-L213))
   to the axis config panel. Label units N·s/mm to match the existing
   damping fields. Range 0 to a sensible max (e.g. 1.0 N·s/mm); 0
   indicates "no floor."

2. **Per-function override.** Add `MinDamping` (nullable `float?`) to
   `AxisParameterOverrides` in
   [SimHubPlugin/TieredConfig/TieredConfigTypes.cs](SimHubPlugin/TieredConfig/TieredConfigTypes.cs).
   When the user is in `FunctionOverride` mode for the active function,
   slider edits route to the override slot via
   `ConfigOrchestrator.UpdateAxisParameterOverride()` — same path
   `SaveOscillationGuardChange()` uses today
   ([AxisConfigControl.xaml.cs:323](SimHubPlugin/AxisConfigControl.xaml.cs#L323)).
   Merge logic in `ConfigMerger.MergeAxisOverrides()`: if override is
   non-null, replace the axis baseline value; otherwise pass through.

3. **Visual override indicator.** Reuse the badge / "function override
   active" affordance OscillationGuard uses, so the user can see at a
   glance whether the slider value is the axis baseline or a function
   override.

4. **Why scalar, not nested-object replacement.** OscillationGuard uses
   full-replacement semantics because it's a complex nested protobuf
   message — partial overrides wouldn't make sense. `min_damping` is a
   single float, so the nullable-override-slot pattern is the natural
   per-field equivalent. UX is identical from the user's perspective; the
   storage shape is just simpler.

5. **No OverrideFieldRegistry entry.** `min_damping` is an axis-level
   field, not a function-config field. The registry handles
   `FunctionConfig` overrides reachable from graph ConfigOuts; axis
   fields use `AxisParameterOverrides` instead. Graph templates cannot
   write `min_damping` — by design, since it's a safety-tuning concern,
   not a per-frame dynamic value.

---

## Phase 4: Profile JSON Migration

**Goal:** existing profile JSONs in the wild reference `flight_pedals.*` and
`flight_stick.*` field aliases via `FunctionOverrides`. They must continue to
load.

### Strategy

On load, walk each `FunctionOverrides` entry. For every old-shape field key,
rewrite to the new shape. Persist the migrated form on next save.

```
flight_pedals.near_lim         → flight_control.pos_min
flight_pedals.far_lim          → flight_control.pos_max
flight_pedals.damping          → flight_control.damping
flight_pedals.centering_spring_const → flight_control.centering_spring_const
flight_stick.pos_min           → flight_control.pos_min
flight_stick.pos_max           → flight_control.pos_max
flight_stick.damping           → flight_control.damping
flight_stick.centering_spring_const  → flight_control.centering_spring_const
flight_stick.phase_offset      → flight_control.phase_offset
flight_stick.vib_harmonic_ratios.{0..4} → flight_control.vib_harmonic_ratios.{0..4}
flight_stick.vib2_harmonic_ratios.{0,1} → flight_control.vib2_harmonic_ratios.{0,1}
```

Implement as a one-pass migration in the deserializer or in
`DiyFfbPluginSettings.AfterDeserialize()`. Idempotent: running on already-new
JSON is a no-op.

### Test coverage

- Round-trip test: old JSON in → migrated → save → load → equivalent state.
- Mixed test: profile with both old and new keys (shouldn't happen in
  practice, but defensible) → migrates the old, leaves the new alone.
- Empty test: profile with no flight-function overrides loads unchanged.

---

## Phase 5: Graph Template ConfigOut Field Paths

Existing graph templates use `FlightStick.Vib1HarmRatio1..5`, `FlightStick.Vib1Phase`
in ConfigOut nodes (e.g. `heli_vibration_cyclic.json` from Plan 08).

**Two options:**

**(a) Update templates in place.** Bulk rename `FlightStick.Vib*` →
`FlightControl.Vib*` in the JSON files. Breaking change for any user-saved
graph template that referenced the old paths.

**(b) Keep aliases.** Registry maps `FlightStick.Vib*` → `FlightControl.Vib*`
internally. Old graphs continue to work; new graphs use the canonical name.

Recommendation: **(b)** during the rollout, then a one-time alias removal
in a follow-up commit once stable. New graph templates committed to the repo
should use the canonical `FlightControl.*` paths.

The graph editor's ConfigOut field-options dropdown (`GetConfigFieldOptionsForType`
in `GraphEditorControl.xaml.cs`) shows the canonical names, so authoring new
graphs naturally lands on the new path.

---

## Phase 6: Tests

### Unit tests requiring updates

- `FlightStickProcessorTests` → rename to `FlightControlProcessorTests`,
  exercise all four flight functions through one path.
- `FlightPedalsProcessorTests` → delete (subsumed).
- `OverrideFieldRegistryTests` → update field paths and group assignments.
- `ConfigMergerTests` → replace `FlightStickXyz` field references.
- `ConfigComparerTests` → same.
- `OrchestratorRerouteTests` → same (some tests use `flight_stick.damping`
  and `flight_pedals.damping`).
- `ProtobufJsonSerializationTests` → update for the new message shape.

### New tests

- **Profile migration**: as listed in Phase 4.
- **Cross-function ConfigOut routing**: a top-level ConfigOut writing
  `FlightControl.Vib1HarmRatio1` fans out to all four flight functions
  (today: stick path covers Pitch/Roll/Collective; pedals are excluded).
  Verify pedals are now included.
- **Alias resolution**: register a graph that uses old `FlightStick.Vib1HarmRatio1`
  ConfigOut field path; verify it resolves to the new field via the alias and
  writes correctly.

---

## Phase 7: Documentation + Cleanup

- Update `Plugin_Design.md`, `Flight_FFB_Architecture.md` references.
- Update Plan 08 (heli vibration) to remove the "pedal vibration deferred"
  note.
- Author `heli_vibration_pedal.json` (deferred from Plan 08) — small variant
  with tail-rotor harmonic ratios per Plan 08 §2.

---

## Risks and Open Questions

1. **Coordinated firmware + plugin rollout.** Wire format breaks. Users who
   update one side without the other get either no flight FFB or mismatched
   config. Mitigation: bump protocol version; refuse mismatched pairing on
   handshake.

2. **Field-renaming dust.** `FlightStickMotionRange` → `FlightControlMotionRange`
   and similar property renames touch many files. Unavoidable but mechanical.

3. **Pedal vibration default off.** New `vib_harmonic_ratios` on pedals must
   default to all-zero so existing pedal profiles see zero amplitude until the
   user explicitly opts in. Same convention as FlightStick today.

4. **Alias TTL.** How long do `flight_stick.*` and `flight_pedals.*` aliases
   stay? Suggested: keep through one minor release after Plan 10 ships, then
   remove in the release after that. Document in the alias registration site.

5. **UI control consolidation.** Out of scope for Plan 10. The two
   `*ConfigControl.xaml.cs` files keep their own UI presentation. A future
   plan can collapse them if/when their UIs converge.

6. **Damping floor relocation.** Phase 2.1 drops the
   `max(_config.damping, flight.k_damper)` clamp inside
   `FlightControlFunction`, which would on its own remove the only
   guardrail against a buggy graph emitting near-zero damping. The plan
   addresses this by moving the floor — not deleting it — to
   `AxisConfig.min_damping`, enforced by `Sim` at the integrator (Phase
   2.1 axis-level section, Phase 1 proto addition). The safety properties
   are stronger than today's design:
   - The floor lives at the integrator, so no function class, graph
     misconfiguration, or future code path can bypass it.
   - It applies to all functions on the axis (automotive pedal, shifter,
     etc.), not just flight.
   - It is decoupled from rest-state damping, so users can tune them
     independently — high rest damping with a moderate safety floor, or
     vice versa.

   Migration concern: `AxisConfig.min_damping` defaults to 0. Existing
   users on resonance-prone motors who relied on `FlightStickConfig.damping`
   acting as a clamp will lose that protection until they set
   `min_damping` on their axis. Mitigations:
   - Add a release-note callout for users to set `min_damping` during the
     first run after upgrade if they have a resonance-prone setup.
   - The safety-damper toggle (Phase 3.7) is the user's runtime escape
     hatch in the meantime.
   - Document the new field and the semantic shift in `Plugin_Design.md`
     so graph authors know `FlightControlConfig.damping` is the rest-state
     value, not a floor.

---

## Implementation Order

1. **Phase 1** (proto) on its own branch; regenerate bindings; commit.
2. **Phase 2** (firmware) in lockstep with Phase 3.1–3.4 (plugin core). The
   wire format breaks, so these must land together.
3. **Phase 4** (migration) immediately after Phase 3 — tests verify old
   profiles still load.
4. **Phase 6** (tests) interleaved throughout each prior phase.
5. **Phase 5** (graph templates) and **Phase 7** (docs) after the core lands.

Single PR if the diff stays manageable; otherwise split Phase 1+2+3 (core
rewire) from Phase 4+5+6 (migration + tests + templates).
