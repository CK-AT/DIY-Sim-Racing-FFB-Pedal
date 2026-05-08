# CommManager Linked-Axes Topology Cache

`CommManager::calc_input_force_sum` and `CommManager::calc_final_position`
walk `linked_axes` every iteration of the FFB loop (≈1 kHz) to redo
the same masking and branching that's already implied by the active
function config. The topology only changes on profile / function
switch — moving the work to config-update time produces a tighter,
clearer hot path.

This plan extends the same caching pattern introduced in
[12_dds_position_delta_rework.md](12_dds_position_delta_rework.md)
(which already adds `_is_subtractive_axis`) into a single topology
cache that subsumes that field.

**Status:** ready to implement
**Branch base:** lands cleanly with or without plan 12; coordinate
ordering at merge time (this plan absorbs `_is_subtractive_axis` if
plan 12 is in first).

---

## Goal

Replace the per-tick `linked_axes` walks in `calc_input_force_sum` /
`calc_final_position` with a tight precomputed structure on
`ConfigManager`. Same external behaviour; tighter inner loops. The
existing `linked_axes`-overload of `calc_input_force_sum` (used by
RudderBrake on `AuxFunctionConfig.linked_axes`) is untouched — it
operates on a different config that lives off the FFB hot path.

---

## What's actually static vs dynamic

| Concept | Lives in | Changes when |
| --- | --- | --- |
| Mode for `calc_final_position` (use own / fetch primary / fetch primary + mirror) | `linked_axes` | Config update |
| Primary axis ID to fetch from | `linked_axes[0]` | Config update |
| Force-sum input list (axis IDs + sign) | `linked_axes` | Config update |
| Subtractive flag for own axis | `linked_axes` | Config update |
| `2.0f * _x_contact_point_center` mirror offset | derived | Config update |
| `get_force(axis_id, ...)` value | per-axis state | Per tick |
| `get_position(primary, ...)` value | per-axis state | Per tick |

Everything in the upper rows is recomputed every loop iteration today.

---

## Resolved decisions

- **D1. Cache lives on ConfigManager.** Populated in `on_config_update()`
  next to `update_x_contact_point_limits()`. CommManager reads but
  never writes the cache. Matches the existing pattern.
- **D2. Topology cache shape.**

  ```cpp
  enum PositionMode {
      POSITION_MODE_USE_OWN,            // independent or primary
      POSITION_MODE_FETCH_PRIMARY,      // linked-additive
      POSITION_MODE_FETCH_PRIMARY_MIRRORED, // linked-subtractive
  };

  struct ForceFetchEntry {
      AxisID axis_id;
      float sign;     // +1.0f or -1.0f
  };

  // ConfigManager additions
  PositionMode  _position_mode    = POSITION_MODE_USE_OWN;
  AxisID        _primary_axis_id  = AxisID_AXIS_UNDEFINED; // valid for FETCH_*
  ForceFetchEntry _force_fetch[4] = {};   // linked_axes is fixed_count:4
  uint8_t       _force_fetch_count = 0;
  ```

  4-entry fixed-size array because `linked_axes` is `max_count:4
  fixed_count:true` per `diy_ffb_protocol.options`. No allocations.
- **D3. Subsumes plan 12's `_is_subtractive_axis`.** That field is
  redundant once `_position_mode` exists — subtractive is just
  `mode == POSITION_MODE_FETCH_PRIMARY_MIRRORED`. The public
  `is_subtractive_axis()` accessor stays (plan 12's Main.cpp call is
  API-stable), but its body becomes a one-line derivation from the
  mode. Drop the cached bool. If plan 12 lands first, this plan
  removes the field; if this plan lands first, plan 12's S3 reduces
  to "the accessor and its consumer already exist; no new field."
- **D4. RudderBrake / AuxFunctionConfig path stays as-is.**
  `RudderBrake::process` calls
  `comm_manager.calc_input_force_sum(config.linked_axes, f_sum)` with
  `AuxFunctionConfig.linked_axes` — a different config that is not
  the active function config and not in the FFB hot path. The
  explicit-`linked_axes` overload remains and continues to walk in
  place. Caching for aux-function topologies is deferred (out of
  scope, see below).
- **D5. Failure-mode parity.** `calc_final_position` returns `false`
  today when this is a non-primary axis and `get_position(primary,
  ...)` reports the primary offline. New version preserves that exact
  semantics — only the "decide which mode" walk is cached, not the
  per-tick liveness query.
- **D6. Mirror offset.** Optionally cache `_x_contact_point_center_2x
  = 2.0f * center` in `update_x_contact_point_limits()` to drop the
  per-call multiply. Trivial; include if it doesn't bloat the diff.
- **D7. No external API change.** `calc_input_force_sum(float &)` and
  `calc_final_position(float, float &)` keep their signatures. Only
  the bodies change. Callers (Main.cpp `physics_task_func`) are
  unaffected.

---

## Implementation steps

Order chosen so each step compiles cleanly.

### S1. Add cache types + members to ConfigManager

- File: [ESP32/include/ConfigManager.h](../../ESP32/include/ConfigManager.h)
- Add the `PositionMode` enum and `ForceFetchEntry` struct at file
  scope (or nested under `ConfigManager` if that reads cleaner).
- Add private members per D2.
- Add public accessors:

  ```cpp
  PositionMode get_position_mode(void) const { return _position_mode; }
  AxisID get_primary_axis_for_fetch(void) const { return _primary_axis_id; }
  bool is_subtractive_axis(void) const {
      return _position_mode == POSITION_MODE_FETCH_PRIMARY_MIRRORED;
  }
  uint8_t get_force_fetch_count(void) const { return _force_fetch_count; }
  const ForceFetchEntry &get_force_fetch_entry(uint8_t i) const { return _force_fetch[i]; }
  ```

- Verify: build clean. No callers yet.

### S2. Populate the cache on config update

- File: [ESP32/src/ConfigManager.cpp](../../ESP32/src/ConfigManager.cpp)
- New private method `update_topology_cache()`:
  - Replicates the decision trees in
    `CommManager::calc_input_force_sum` /
    `CommManager::calc_final_position` exactly, but writes into the
    cache instead of returning per-call.
  - For `PositionMode`:
    1. If own axis is marked `AxisID_AXIS_INDEPENDENT` in any
       `linked_axes` entry, or `linked_axes[0]` is independent:
       `POSITION_MODE_USE_OWN`.
    2. Else if `linked_axes[0]` axis ID == own axis ID:
       `POSITION_MODE_USE_OWN`.
    3. Else: walk entries 1..3 for own axis ID; if found subtractive →
       `POSITION_MODE_FETCH_PRIMARY_MIRRORED`, else
       `POSITION_MODE_FETCH_PRIMARY`. Record `_primary_axis_id =
       linked_axes[0] & MASK`.
  - For `_force_fetch[]`: populate per the existing
    `calc_input_force_sum` decision tree (independent gating + sign
    by `AxisID_AXIS_SUBTRACTIVE`). For each entry that the existing
    code would have summed, append `{axis_id, +1.0f}` or `{axis_id,
    -1.0f}`.
- Call from `on_config_update()` next to
  `update_x_contact_point_limits()`.
- Optionally cache `_x_contact_point_center_2x` in
  `update_x_contact_point_limits` (D6).
- Verify: build clean. Add a one-line `LogOutput::printf` summarising
  the cached topology after each config update during bring-up; drop
  before merge.

### S3. Refactor `CommManager::calc_final_position` to use the cache

- File: [ESP32/src/CommManager.cpp](../../ESP32/src/CommManager.cpp)
- Replace body of `calc_final_position(float own_position, float
  &final_position)`:

  ```cpp
  switch (_config_manager->get_position_mode()) {
      case POSITION_MODE_USE_OWN:
          final_position = own_position;
          return true;
      case POSITION_MODE_FETCH_PRIMARY: {
          float other;
          if (!get_position(_config_manager->get_primary_axis_for_fetch(), other)) return false;
          final_position = other;
          return true;
      }
      case POSITION_MODE_FETCH_PRIMARY_MIRRORED: {
          float other;
          if (!get_position(_config_manager->get_primary_axis_for_fetch(), other)) return false;
          final_position = (2.0f * _config_manager->get_x_contact_point_center()) - other;
          // or use _x_contact_point_center_2x if D6 included
          return true;
      }
  }
  return false;
  ```

- Verify: build clean. Manual table check — for each of the four
  topology modes, trace one example and confirm output matches the
  pre-refactor function.

### S4. Refactor `CommManager::calc_input_force_sum(float &)` to use the cache

- File: [ESP32/src/CommManager.cpp](../../ESP32/src/CommManager.cpp)
- Replace body of the no-arg overload (line 892-894) with a tight
  loop over the cached fetch list:

  ```cpp
  bool CommManager::calc_input_force_sum(float &input_force) {
      float f_sum = 0.0f;
      uint8_t n = _config_manager->get_force_fetch_count();
      for (uint8_t i = 0; i < n; i++) {
          const auto &entry = _config_manager->get_force_fetch_entry(i);
          float temp = 0.0f;
          get_force(entry.axis_id, temp);
          f_sum += entry.sign * temp;
      }
      input_force = f_sum;
      return _config_manager->is_subtractive_axis();
  }
  ```

- The explicit-`linked_axes` overload at line 856 is **untouched**
  per D4 — RudderBrake still works.
- Verify: build clean. Same manual table check as S3 across modes.

### S5. Native unit tests for the topology cache

- New file: `ESP32/test/test_topology_native/test_topology.cpp` (or
  add cases to an existing native suite if there's a fitting one).
- Build a fake `FunctionBase` for each topology and call
  `update_topology_cache()` (may need a friend class or a
  `set_function_config_for_test()` shim — pick whichever fits the
  existing testing style).
- Cases:
  - **Independent.** Own entry has `AXIS_INDEPENDENT`. Expect
    `USE_OWN`, `is_subtractive == false`, `_force_fetch` contains
    own axis only with `+1`.
  - **Primary.** `linked_axes[0]` == own. Expect `USE_OWN`,
    not subtractive, fetch list = all linked entries with their signs.
  - **Linked additive.** `linked_axes[0]` ≠ own; own appears in
    entry 1..3 without `AXIS_SUBTRACTIVE`. Expect `FETCH_PRIMARY`,
    not subtractive.
  - **Linked subtractive.** Same but with `AXIS_SUBTRACTIVE` flag.
    Expect `FETCH_PRIMARY_MIRRORED`, `is_subtractive == true`.
- Verify: `pio test -e native` passes.

### S6. Bench verification

- Multi-axis flight setup if available.
- Confirm position tracking (linked) is identical pre- and
  post-refactor: snapshot CAN traffic / sled positions before, then
  after, and diff. Should be byte-identical for the same input
  forces.
- Confirm force-sum on a multi-axis function still produces the
  same result (e.g., dual-load-cell setups).
- For setups without multi-axis, single-axis (independent) regression
  is sufficient — that path is also exercised.

---

## Risks

1. **Stale cache on config update.** If `update_topology_cache()`
   is missed in any code path that mutates `_function_config`, the
   hot path runs with old topology. Mitigation: there's exactly one
   callsite (`on_config_update`) and the existing
   `update_x_contact_point_limits` is already called there — drop
   the new updater alongside it. Add a debug-only assertion or log
   that the cache regen ran each time `_function_config` changes.

2. **Decision-tree drift.** The new `update_topology_cache` must
   replicate the old per-call walks **exactly**. Any divergence is a
   silent behaviour change. Mitigation: S5 unit tests cover all four
   modes; S6 bench diff catches anything subtle.

3. **`_force_fetch` ordering.** The old walk iterated `linked_axes`
   in order and appended in that order. If any caller depends on
   ordering for floating-point summation determinism, preserve it.
   The cache populates in the same order.

4. **Plan 12 ordering.** If plan 12 is partially landed (S3 added
   `is_subtractive_axis` but not the consumer), this plan's S2
   replaces the populator and is otherwise compatible. Make sure
   not to land both `update_is_subtractive_axis()` and
   `update_topology_cache()` — collapse into the latter.

---

## Out of scope

- **Aux-function (RudderBrake) topology caching.** Could mirror the
  same pattern keyed on the aux config, but it's not the FFB hot
  path. Defer until a profiler call out shows it.
- **`get_force` / `get_position` query optimization.** The dynamic
  per-tick part of the call. Outside this plan.
- **Mirror offset caching beyond D6's optional bit.** If profiling
  shows it matters, lift it; otherwise skip.

---

## Verification checklist

- [ ] `pio run -e esp32` clean
- [ ] `pio test -e native` passes (existing suites + S5 topology tests)
- [ ] Manual mode-by-mode trace for `calc_final_position` matches
      pre-refactor output (independent / primary / linked-additive /
      linked-subtractive)
- [ ] Manual force-sum trace across the same four modes
- [ ] Multi-axis bench diff (if rig available): pre- vs post-refactor
      sled positions identical for the same input forces
- [ ] RudderBrake still works (explicit-`linked_axes` overload
      untouched, but a smoke test on a configured rudder-brake setup
      catches unintended fall-through)
