# DDS Position-Delta Rework

Rework of the SyncVib coupling. The existing implementation (see
[07_dds_subsystem.md](07_dds_subsystem.md) and the 07a/b/c sub-plans) is
landed and tested, but in practice tuning damping is awkward because
vibration force is summed into the integrator post-friction. Damping
changes shift how vibration translates into position swing, so every
damping tweak forces a re-tune of the harmonic amplitudes.

This plan re-couples SyncVib output as a **position delta on the servo
command path only** — accumulated through the integrator's accumulator
struct, never injected into `f_sum`. Damping is then orthogonal to
vibration: the integrator's behaviour is unchanged, the user's mass /
spring / damper feel is unchanged, and the shake is a fixed mm swing
added on top.

**Status:** ready to implement
**Branch base:** current `main` (DDS subsystem fully merged)

---

## Goal

After this plan, SyncVib's output drives a position offset on the servo
command path, not a force injection into the Sim. Tuning damping no
longer affects vibration amplitude. Buffet is left on the existing
`f_vib` force path for now (separate decision; revisit only if the same
tuning friction shows up there).

---

## Resolved decisions

- **D1. Where the delta is applied.** SyncVib produces a sample in the
  local axis's positive direction (same convention the existing
  per-axis `phase_offset` was tuned against). The delta is added in
  the **contact-point frame**, **after** `calc_final_position` runs:
  `x_contact_servo = x_contact_point + sign * vib_delta_mm`. Going
  before `calc_final_position` does **not** work for non-primary
  axes — that function ignores `own_position` and returns the linked
  primary's broadcast (which is un-vibrated per D2), so the delta
  would be silently dropped.
- **D1a. Sign for subtractive axes.** For linked-subtractive axes the
  contact frame is mirrored from primary (`final_position = 2*center -
  primary_contact`). The local actuator's positive direction is
  opposite to the contact frame, so a positive SyncVib sample must be
  applied with **inverted** sign in contact frame. `sign = -1` for
  subtractive, `+1` for primary / independent / linked-additive. This
  mirrors the existing pattern used for `calc_input_force_sum` (which
  also flips the sign for subtractive axes). The sign is a property
  of the function config and only changes on profile / function
  switch — cache it on `ConfigManager` at config-update time rather
  than re-walking `linked_axes` every loop iteration.
- **D2. External position is not vibrated.** `send_force_and_position`
  continues to broadcast the un-vibrated `x_contact_point`. Other axes
  must not see a peer's vibration in linked-axis math (primary's
  broadcast feeds non-primary's `calc_final_position`). Each axis runs
  its own SyncVib instances; the gateway PLL keeps phase coherent.
- **D3. Buffet stays on `f_vib`.** No migration. `accum.f_vib` and the
  post-friction injection in `Sim::update` / `compute_force_sum` remain.
  Comment is updated to clarify "Buffet only" once SyncVib leaves.
- **D4. Wire format unchanged in shape; LSB rescaled.** Amplitude fields
  stay `uint8`, 7 slots (5+2). Scaling changes from 0.05 N/LSB to
  **0.01 mm/LSB** (range 0..2.55 mm). Profiles must be re-tuned —
  amplitude *meaning* changes, not the field layout.
- **D5. No proto reshape.** `vib_amp_slot1..5` / `vib2_amp_slot1..2`,
  `vib_harmonic_ratios`, `phase_offset`, `DdsFundamentals` all unchanged.
  Comment strings on amplitude fields update from "N" to "mm".
- **D6. Position clamping.** The vibrated servo position is clamped to
  `sim.get_x_min()` / `sim.get_x_max()`. Linked axes share one
  `FunctionConfig`, so `pos_min` / `pos_max` propagate through to
  every axis's Sim — the clamp is the exact contact-frame range
  regardless of independent / primary / linked-additive / linked-
  subtractive role. Keeps a high-amplitude shake near a stop from
  commanding the servo past travel.
- **D7. Sample plumbing — dedicated `x_vib` accumulator.** SyncVib
  writes its instantaneous sample into a new
  `SimAccumulators::x_vib` field, mirroring how Buffet writes to
  `f_vib`. `Sim::update` captures the final value into a private
  `_x_vib` member after running the element list and exposes it via
  `get_x_vib()`. Multiple SyncVib instances on the same Sim (e.g.,
  `vib1` + `vib2` on a flight function) sum naturally because they
  all `+=` into the same accumulator. **No `IFunction` interface
  change** — the integrator API alone is enough to surface the delta.
- **D8. Sub-iteration handling.** When `physics_iterations_per_sample > 1`,
  `sim.update()` is called N times per loop tick. `accum.x_vib` is a
  fresh stack-local each call and `_x_vib` is overwritten on each
  call; only the final sub-iteration's value is used as the delta.
  This matches how `f_vib` was integrated previously (last sub-step
  wins anyway, since the integrator only commits the final position)
  and keeps phase advance internally consistent.

---

## Implementation steps

Order chosen so firmware compiles cleanly after every step.

### S1. Add `x_vib` accumulator + Sim accessor

- File: [ESP32/include/Physics.h](../../ESP32/include/Physics.h)
- Add to `SimAccumulators` next to `f_vib`:

  ```cpp
  // Vibration position delta (mm), summed across SyncVib instances
  // each tick. Captured by Sim::update into _x_vib for the servo
  // path; never enters the integrator.
  float x_vib = 0.0f;
  ```

- Add to `Sim`:
  - private member `float _x_vib = 0.0f;`
  - public accessor `float get_x_vib(void) const { return _x_vib; }`
- File: [ESP32/src/Physics.cpp](../../ESP32/src/Physics.cpp)
- In `Sim::update`, after the (gated) element-update loop, capture:
  `_x_vib = accum.x_vib;`. `accum` is a fresh stack-local each call
  and `x_vib` defaults to `0.0f`, so the `final_f` branch (where the
  element loop is skipped) naturally carries 0 with no special case.
  Mirror the same line in `Sim::compute_force_sum` for the unit-test
  path.
- Verify: build clean. No behaviour change yet (no element writes
  `x_vib`).

### S2. SyncVib: write `accum.x_vib`, not `accum.f_vib`

- File: [ESP32/src/Physics.cpp](../../ESP32/src/Physics.cpp)
- In `SyncVib::update`: replace
  `accum.f_vib += f;` with `accum.x_vib += f;`. No other behaviour
  change (phase advance, amplitude LPF, PLL all identical).
- Update class doc comment in
  [Physics.h](../../ESP32/include/Physics.h): output is "position
  delta (mm)" routed via `accum.x_vib`, not a force on `accum.f_vib`.
- Verify: `pio run -e esp32` builds.

### S3. `is_subtractive_axis()` cached on ConfigManager

ConfigManager exposes the axis-config fact. The vibration-sign
mapping (-1 / +1) belongs to the consumer (`physics_task_func` in
Main.cpp), not here — keeps ConfigManager free of vibration semantics.

- File: [ESP32/include/ConfigManager.h](../../ESP32/include/ConfigManager.h),
  [ESP32/src/ConfigManager.cpp](../../ESP32/src/ConfigManager.cpp)
- New private member: `bool _is_subtractive_axis = false;`
- New public accessor (matches the cached-getter pattern already used
  for `_x_contact_point_min/max`):

  ```cpp
  bool is_subtractive_axis(void) const { return _is_subtractive_axis; }
  ```

- New private updater, e.g. `update_is_subtractive_axis()`, called
  from `on_config_update()` right alongside
  `update_x_contact_point_limits()`. Walks
  `_function_config.base.linked_axes` once: returns `true` only for
  the linked-non-primary case where the entry matching `_axis_id`
  carries the `AxisID_AXIS_SUBTRACTIVE` flag; `false` for independent,
  primary, and linked-additive. Same decision tree as
  `CommManager::calc_final_position` / `calc_input_force_sum`, but
  evaluated once per config change rather than every loop iteration.
- Verify: build clean. Sanity-print the cached value once after each
  config update via `LogOutput::printf` during initial bring-up; drop
  the print before merging.

### S4. Apply delta on servo path in Main.cpp

- File: [ESP32/src/Main.cpp](../../ESP32/src/Main.cpp) around line 560.
- Pattern (replaces the current single-chain block):

  ```cpp
  // External broadcast — un-vibrated, computed once.
  comm_manager.calc_final_position(sim.get_x(), x_contact_point);

  // Servo command path — vibration delta added in contact frame
  // post-calc_final_position (own_position is dropped on the floor
  // for non-primary axes, so the delta cannot be injected upstream).
  // Sign flips for subtractive axes so a positive SyncVib sample
  // (in local sim convention) drives the local actuator in its
  // local positive direction.
  float vib_sign = config_manager.is_subtractive_axis() ? -1.0f : 1.0f;
  float x_contact_servo = x_contact_point + vib_sign * sim.get_x_vib();

  // Clamp so a high-amplitude shake near a stop doesn't drive the
  // servo past travel.
  float lo = sim.get_x_min();
  float hi = sim.get_x_max();
  if (x_contact_servo < lo) x_contact_servo = lo;
  if (x_contact_servo > hi) x_contact_servo = hi;

  x_sled = config_manager.calc_sled_position(x_contact_servo);
  ```

- `send_force_and_position(f_contact_point, x_contact_point)` later in
  the loop is **unchanged** — broadcasts the un-vibrated value.
- The `DEBUG_INFO_0_LOADCELL_READING` debug log keeps logging
  `x_contact_point` (un-vibrated). Optional: log `sim.get_x_vib()`
  and/or `x_contact_servo` as new columns. Defer unless it's needed
  for bench tuning.
- Verify: builds clean. Bench feel-test confirms damping changes no
  longer attenuate vibration; vibration amplitude tracks the
  configured mm value (rough check with a mm ruler on the stick).
  If a subtractive flight axis is in the test rig, verify the local
  actuator vibrates in the *expected* direction (sign helper hooked
  up correctly) — easy to miss otherwise because the vibration is
  symmetric around the operating point.

### S5. Update `f_vib` comment / scope

- File: [ESP32/include/Physics.h](../../ESP32/include/Physics.h),
  [ESP32/src/Physics.cpp](../../ESP32/src/Physics.cpp)
- `SimAccumulators::f_vib` doc comment: "vibration force, bypasses
  damping/friction. Used by Buffet." (was "Used by Buffet and SyncVib.")
- The `accum.f_sum += accum.f_vib;` injection in `Sim::update` and
  `Sim::compute_force_sum` stays — Buffet still depends on it.
- Verify: build clean.

### S6. Update SyncVib unit tests

- File: [ESP32/test/test_physics_native/test_physics.cpp](../../ESP32/test/test_physics_native/test_physics.cpp)
- `RunSyncVib` helper: rename the captured field from `accum.f_vib`
  to `accum.x_vib`. No other change.
- All three tests (continuity, phase offset, amp smoothing) work
  unchanged — they assert on the trace shape, which is identical
  (same value, different accumulator).
- Verify: `pio test -e native` passes.

### S7. Plugin: rescale amplitudes to mm

- File: [SimHubPlugin/DiyFfbPlugin.cs](../../SimHubPlugin/DiyFfbPlugin.cs)
  around `PackVibAmp` (line 1343).
- `PackVibAmp`: change scale from `* 20f` (1/0.05) to `* 100f` (1/0.01).
  Update comment from "0.05 N/LSB ... range 0..12.75 N" to
  "0.01 mm/LSB ... range 0..2.55 mm".
- File: [ESP32/src/FlightControlFunction.cpp](../../ESP32/src/FlightControlFunction.cpp)
- `kAmpScale`: change from `0.05f` to `0.01f`. Update comment.
- Verify: plugin builds via MSBuild (see CLAUDE.md memory entry).
  ESP32 builds. End-to-end smoke test: send a known amplitude from the
  plugin, confirm the sample lines up at the expected mm magnitude
  (peak of `sin` × amp).

### S8. Update unit / scope comments in proto-adjacent code

Wire format and layout unchanged, but inline doc comments call the
amplitude unit "N" in several places. Update only the obvious ones —
generated `DiyFfbProtocol.cs` regenerates from
`proto/diy_ffb_protocol.proto`, so update the proto file's field
comments to "mm" and re-run nanopb generation.

- File: [proto/diy_ffb_protocol.proto](../../proto/diy_ffb_protocol.proto)
  — update `vib_amp_slot*` / `vib2_amp_slot*` doc comments from "N" to
  "mm".
- Regenerate plugin and ESP32 protobuf bindings via the existing
  `extra_script.py` / nanopb step.
- Verify: regenerated `DiyFfbProtocol.cs` shows the new comments;
  builds still clean.

### S9. Update signal-catalog and architecture docs

- File: [SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md](../../SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md)
  lines 75–76: change unit + range to "mm, 0..2.55 mm".
- File: [SimHubPlugin/Docs/Flight_FFB_Architecture.md](../../SimHubPlugin/Docs/Flight_FFB_Architecture.md)
  lines 48–49: change column "N" → "mm". Line 62 wire-format detail:
  update to "0.01 mm/LSB".
- File: [docs/plans/07_dds_subsystem.md](07_dds_subsystem.md)
  sections 4 (proto comments), 5 (graph output unit column), and 7
  (vibration force bypass — note that SyncVib no longer uses `f_vib`,
  link forward to this plan).
- Verify: docs reviewed; cross-references intact.

### S10. Bench feel-test

This step splits across the work: the pre-rework reference must be
captured **before** S1 starts, otherwise the A/B is gone.

- **Pre-step (before S1).** Flash current `main`. Pick a known-good
  helicopter profile. With damping setting D1, note "feels right"
  vibration amplitudes. Switch to damping setting D2 and note how
  amplitude perception changes. Save the profile values for both
  settings.
- **Post-step (after S9).** Flash post-rework binary. Re-tune
  amplitudes once for damping D1 (new mm units, expected). Then
  switch to D2 *without* re-tuning amplitudes. Vibration amplitude
  should be perceptually unchanged from D1 — the whole point of
  the rework.
- Confirm no audible click / discontinuity when amplitudes step (LPF
  smoothing is identical, but the coupling change might surface a
  pre-existing edge case).
- Confirm no servo-stop overshoot when stick is parked at a limit
  and high-amp vibration is sent (D6 clamp).

---

## Risks

1. **Servo bandwidth.** A position-step waveform asks the servo to
   chase mm-scale moves at the harmonic frequency. Above the servo's
   closed-loop bandwidth, the rendered amplitude rolls off — feel may
   change vs the force regime. Mitigation: bench-test at the highest
   harmonic of interest (5/rev or 2N/rev for helis); document the
   useful upper bound. This is the same physical reality that the
   force regime had — just exposed differently.

2. **Aggressive feel near stops.** Position injection commands the
   servo to push against a hand at a limit; force injection just adds
   force the integrator can absorb. The clamp in S4 prevents *commanded*
   travel past stops, but the feel is sharper because the servo will
   actually try to move there. Profile amplitudes may need to come down
   slightly for comfort.

3. **Buffet feel divergence.** Buffet stays on `f_vib`, so it is still
   subject to friction-clipping below the friction threshold. SyncVib
   isn't. Aircraft profiles that mixed buffet (chaotic) and SyncVib
   (coherent) at similar amplitudes will now have a perceptual
   imbalance — buffet quieter than coherent harmonics. Acceptable
   per D3; revisit if it surfaces in tuning.

4. **External-position consumers.** Anything that reads
   `send_force_and_position` output — peer axes via CAN, the SimHub
   plugin's position read-back — sees the un-vibrated value. This is
   intentional (D2) but needs a sanity check that no consumer was
   silently relying on the vibrated value being broadcast. Grep the
   plugin's position decode for any "stick wobble" telemetry use.

5. **Profile re-tune across the fleet.** Existing aircraft profiles
   carry amplitudes in newtons (0..12.75 N range). After this plan
   they are interpreted as mm (0..2.55 mm). Numerically the field
   stays in 0..255 raw, so loaded profiles produce a similar magnitude
   *number* but different *unit* — feel will be wildly off until each
   profile is retuned. Heads-up in commit message; bundle a "profile
   retune" pass for the heli/plane templates in a follow-up.

6. **Sub-iteration sample latency.** D8 settles the multi-substep case,
   but it's worth confirming on hardware: at high
   `physics_iterations_per_sample` the SyncVib sample updates at the
   sub-iteration rate (still 1 kHz or higher), so phase resolution
   improves rather than degrades. No risk expected.

---

## Out of scope

- Buffet migration to position-delta path (D3 — revisit only if
  needed).
- Profile retune for shipped templates (separate follow-up).
- Servo bandwidth characterisation (deferred to general bench
  documentation, not this plan).
- Optional debug logging of `vib_delta_mm` in the loop — add only if
  bench tuning calls for it.

---

## Verification checklist

- [ ] `pio run -e esp32` clean
- [ ] `pio test -e native` passes (3 SyncVib tests)
- [ ] Plugin builds via MSBuild
- [ ] Damping change does not perceptibly change vibration amplitude
      (S10 bench test)
- [ ] External position read-back via CAN snoop shows no vibration
      content (D2)
- [ ] Stick parked at travel limit + high-amp shake → servo does not
      overshoot stop (D6 clamp)
- [ ] On a subtractive flight axis (if available in the rig): local
      actuator vibrates in the same direction the SyncVib sample
      points (sign helper, D1a)
- [ ] Aircraft profile retuned to new mm scale and lands feeling
      similar to pre-rework after one tuning pass
