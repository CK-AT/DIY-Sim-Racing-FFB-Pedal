# Kinematic Polynomial Float Cache

`ConfigManager::calc_force_conversion_factor` and
`ConfigManager::calc_sled_position` evaluate degree-4 polynomials in
**`double`** precision every FFB tick (≈1 kHz). The ESP32 has a
single-precision FPU only — every `double` op is software-emulated,
roughly 10–50× slower than its `float` counterpart. The wire format
keeps `double` (offline polynomial fitting genuinely needs it), but
the on-device evaluation can run in `float` with margin.

This plan caches `float` copies of both coefficient arrays at axis-
config-update time and switches the hot-path evaluation to `float`
Horner form. The `double` originals stay in `_axis_config` for
inspection / round-trip / re-emit.

**Status:** ready to implement (after numerical analysis below)
**Branch base:** independent of plans 12 / 13.

---

## Numerical analysis

### Inputs

Real config sample (`z:/Projects/FFB/full_default_config.json`),
five axes:

| Axis | `contactPointPos*Abs` (0.1 mm) | Range (mm) |
| --- | --- | --- |
| 1, 2, 3 | ±1029 | ±102.9 |
| 4 | -940 .. 942 | -94.0 .. +94.2 |
| 5 | -451 .. 376 | -45.1 .. +37.6 |

Worst |x| ≈ **103 mm**. Polynomial degree fixed at 4 (`max_count:5
fixed_count:true` in `proto/diy_ffb_protocol.options`).

### Worst-case coefficient magnitudes (across all 5 axes)

| Term | Force factor (max) | Sled position (max) |
| --- | --- | --- |
| c0 | 5.35e-1 | 2.00e+1 |
| c1 | 8.20e-4 | 5.55e-1 |
| c2 | 7.16e-6 | 4.38e-4 |
| c3 | 2.75e-8 | 3.13e-6 |
| c4 | 4.10e-11 | 3.73e-9 |

### Float precision at the largest intermediate

At |x| = 103 mm:
- x² = 10 609 (exact in float)
- x³ ≈ 1.09e6 (exact in float — under the 16,777,216 exact-integer
  ceiling)
- x⁴ ≈ 1.13e8 — **above** the float exact-integer ceiling. ULP at
  this magnitude is 2^(27-23) = **16**, so float carries x⁴ with
  absolute error ≤ 8.

### Resulting error contribution

`c4 × x⁴` is the only term where the input loses bits. Worst-case
error from that one term:

- Force factor: 4.10e-11 × 8 = **3.3e-10**
- Sled position: 3.73e-9 × 8 = **3.0e-8**

Cumulative error across the whole 5-term polynomial (each mul/add
contributes ≤ 1 ULP relative error, ~12 ops total):

- Relative ≤ 12 × 1.19e-7 ≈ **1.4e-6**
- Force factor result ~ 0.52 → absolute error ≤ **7e-7** (unitless)
- Sled position result ~ 58 mm → absolute error ≤ **8.1e-5 mm = 81 nm**

### What precision do we actually need?

Sled travel resolution: `steps_per_mm = 1000` → 1 µm step.
The float worst-case absolute error at the output is **81 nm**,
i.e. **~12× better than the step size**. Force factor is unitless and
multiplied by load-cell N — float precision at 1e-7 is far below any
load-cell noise floor (load cells are typically ~16-bit).

### Why the original was `double`

The polynomial *fit* (offline, Python / SimHub plugin) genuinely
needs double — least-squares with x⁴ terms is ill-conditioned in
float and produces wrong coefficients. The wire format must stay
`double` to faithfully transport the fit result. **The on-device
evaluation has no such requirement** — once the coefficients are
correct, evaluation in float is accurate to <100 nm at the
output, which is well below all physical resolutions in the system.

### Sanity-check at runtime (defensive)

To remove any doubt, S5 below adds a one-shot self-check at axis-
config update time: evaluate the polynomial at ~32 points spanning
the configured range in both `double` and `float`, log the max
absolute deviation. If the deviation ever exceeds 0.001 mm (sled)
or 0.01 (force factor), keep using the `double` path for that axis
and emit a warning. Belt-and-suspenders for any future axis with
a pathological fit.

---

## Resolved decisions

- **D1. Wire format unchanged.** `repeated double` in
  `proto/diy_ffb_protocol.proto` stays. Plugin / calibration tools
  emit, store, and import doubles. The change is on-device only.
- **D2. Cache lives on ConfigManager.** `_axis_config` keeps the
  authoritative `double` coefficients; new private `float`
  arrays cache them. Populated in `update_axis_config` (or wherever
  `_axis_config` is committed) — same pattern as plans 12 / 13.
- **D3. Switch evaluation to Horner form.** Reduces multiplies from
  2N-2 to N-1 (~25% fewer FP ops on top of the float speedup), and
  is slightly more numerically stable (no large `temp = x⁴`
  intermediate). One loop, one accumulator. Net code is shorter.
- **D4. Self-check at config update.** S5 below; cheap, runs once
  per config commit, prints a one-liner summary. Falls back to
  the `double` path if any axis exceeds the deviation threshold.
- **D5. Public signature unchanged.** `calc_force_conversion_factor`
  and `calc_sled_position` keep their existing `(float &x) → float`
  signatures. The body changes; callers are not touched.
- **D6. `calc_poly`'s second life.** The free-floating `calc_poly`
  function in `ConfigManager.cpp:5` becomes unused after this plan.
  Delete it (or leave it for the unit-test fallback path; pick on
  taste).

---

## Implementation steps

### S1. Add cached float arrays + accessor

- File: [ESP32/include/ConfigManager.h](../../ESP32/include/ConfigManager.h)
- Add private members:

  ```cpp
  // Cached single-precision copies of the kinematic polynomial
  // coefficients, populated whenever _axis_config is committed.
  // Hot path uses these; the double originals in _axis_config stay
  // for inspection and re-emit.
  static constexpr uint8_t KINEMATIC_POLY_DEGREE = 5;  // max_count:5 fixed_count:true
  float _coeffs_force_factor_f[KINEMATIC_POLY_DEGREE] = {};
  float _coeffs_sled_pos_f[KINEMATIC_POLY_DEGREE] = {};
  bool  _kinematic_use_double_fallback = false;  // S5 self-check writes this
  ```

- No new public accessor — the cache is private, used by
  `calc_force_conversion_factor` / `calc_sled_position` directly.
- Verify: build clean.

### S2. Populate cache on axis config update

- File: [ESP32/src/ConfigManager.cpp](../../ESP32/src/ConfigManager.cpp)
- New private method, e.g. `update_kinematic_poly_cache()`. Copy each
  `double` coefficient into its `float` slot. Trim trailing zeros if
  desired (loop bound by last-non-zero index) — minor extra savings,
  optional for first cut.
- Call from `update_axis_config` after `_axis_config = new_config;`
  is committed (and before the function returns success). Also call
  from `set_axis_config_defaults` so first-boot has a valid cache
  before the FFB loop ever starts.
- Verify: build clean. Print the cached arrays once via
  `LogOutput::printf` during bring-up; drop before merging.

### S3. Switch evaluation to float Horner

- File: [ESP32/src/ConfigManager.cpp](../../ESP32/src/ConfigManager.cpp)
- Replace bodies of `calc_force_conversion_factor` and
  `calc_sled_position`:

  ```cpp
  static inline float horner5_f(float x, const float *c) {
      // c[0] + c[1]*x + c[2]*x^2 + c[3]*x^3 + c[4]*x^4
      // evaluated as: ((((c[4]*x + c[3])*x + c[2])*x + c[1])*x + c[0])
      float r = c[4];
      r = r * x + c[3];
      r = r * x + c[2];
      r = r * x + c[1];
      r = r * x + c[0];
      return r;
  }

  float ConfigManager::calc_force_conversion_factor(float &x) {
      if (_kinematic_use_double_fallback) {
          return calc_poly(x,
              _axis_config.kinematic_parameters.coeffs_force_factor_over_contact_point_pos,
              KINEMATIC_POLY_DEGREE);
      }
      return horner5_f(x, _coeffs_force_factor_f);
  }

  float ConfigManager::calc_sled_position(float &x) {
      if (_kinematic_use_double_fallback) {
          return calc_poly(x,
              _axis_config.kinematic_parameters.coeffs_sled_pos_over_contact_point_pos,
              KINEMATIC_POLY_DEGREE);
      }
      return horner5_f(x, _coeffs_sled_pos_f);
  }
  ```

- The `x` parameter stays a `float &` for ABI parity (existing
  callers pass `float`s by reference). Pass-by-reference seems
  legacy; not worth changing in this plan.
- Verify: build clean.

### S4. Native unit tests

- New cases in `ESP32/test/test_physics_native/test_physics.cpp` or a
  new `test_kinematic_poly_native/`:
  - **Equivalence sweep.** For each axis from `full_default_config.json`,
    sweep x across `[contactPointPosMinAbs, contactPointPosMaxAbs] /
    10` in 33 steps, compute `horner5_f(float, coeffs_f)` and
    `calc_poly(float_x, coeffs_d, 5)`. Assert max absolute deviation:
    - Force factor: < 1e-5 (unitless, far below load-cell noise floor)
    - Sled position: < 1e-3 mm (well below 1 µm step resolution)
  - **Sanity at zero.** `horner5_f(0.0f, coeffs)` returns `coeffs[0]`
    cast to float.
  - **Reference vector.** Hard-code one (x, expected_sled_pos) pair
    computed offline in double, assert match within 1e-3 mm.
- Verify: `pio test -e native` passes.

### S5. Runtime self-check + fallback

- File: [ESP32/src/ConfigManager.cpp](../../ESP32/src/ConfigManager.cpp)
- Inside `update_kinematic_poly_cache()`, after populating the float
  arrays, sweep x across the configured range
  (`contact_point_pos_min_abs / 10.0f` to `... max_abs / 10.0f`) in
  ~32 points. For each, compute the `double`-precision result via
  `calc_poly` and the `float`-Horner result via `horner5_f`. Track
  the max absolute deviation per polynomial.
- If either deviation exceeds the threshold (1e-3 mm sled, 1e-5
  force factor), set `_kinematic_use_double_fallback = true` and
  log:

  ```
  ConfigManager: kinematic poly float cache failed self-check
  (force max err X, sled max err Y) — falling back to double
  ```

- Otherwise log a single-line success summary and clear the flag.
- Verify: with default + real configs, the self-check passes silently
  (no fallback). To exercise the fallback path: temporarily inject a
  known-bad config (e.g., set c4 to 1e-3) and confirm the warning
  fires and the `double` path is taken.

### S6. Tidy / removal

- File: [ESP32/src/ConfigManager.cpp](../../ESP32/src/ConfigManager.cpp)
- The free-function `calc_poly` (line 5-13) is still needed by S3's
  fallback and S5's self-check. **Keep it.** Move it to an anonymous
  namespace at file scope to make it private to this TU if it isn't
  already — minor hygiene.
- Verify: build clean.

### S7. Bench feel-test

- Pre-rework: capture sled position log over a full pedal sweep
  (e.g., FlightControl axis pulled limit-to-limit). Save trace.
- Post-rework: same sweep, same input. Diff sled position trace.
  Should match within 1 µm at every sample.
- FFB cycle timer (DEBUG_INFO_0_CYCLE_TIMER): note average µs/tick
  before and after. Expect a measurable drop (estimate 0.5–4 µs per
  tick, depending on background load). Document the actual delta.

---

## Risks

1. **Future axis with pathological coefficients.** Someone re-fits a
   polynomial with truly extreme coefficients (e.g., c4 such that
   `c4 * x⁴` straddles cancellation), and float deviates from
   double. **Mitigation:** S5's runtime self-check catches this on
   the offending config and falls back to double for that axis only.
   Other axes keep their float speedup.

2. **Self-check overhead at config update.** 64 polynomial
   evaluations + abs/max twice. Sub-millisecond on ESP32; runs
   once per config commit (rare). Negligible.

3. **Existing `calc_poly` callers outside ConfigManager.** Verified
   none in this codebase: only `calc_force_conversion_factor` /
   `calc_sled_position` use it ([ConfigManager.cpp:291](../../ESP32/src/ConfigManager.cpp#L291),
   [:295](../../ESP32/src/ConfigManager.cpp#L295)). Both wrapped in
   this plan. No risk to other call sites.

4. **Coefficient ordering / convention drift.** `calc_poly`
   evaluates `c[0] + c[1]·x + c[2]·x² + ...`. Horner above evaluates
   the same polynomial. Double-check by hand for one config row in
   S4's reference test before merging.

5. **Pass-by-reference signature.** `(float &x)` is awkward but
   matches existing call sites. Don't change it in this plan; would
   ripple through Main.cpp and add noise to the diff.

---

## Out of scope

- Switching the wire format from `double` to `float` (D1 — we keep
  the wire authoritative as double for offline-fit fidelity).
- Caching trailing-zero detection / loop-bound shortening — minor
  win, can layer on later.
- The other two FFB-loop optimisations from the parallel discussion:
  - Load-cell sign + 9.81 + conversion-factor merge
  - Load-cell filter dispatch via function pointer

  Both are orthogonal; could be a separate plan. Not included here
  to keep this one focused.

---

## Verification checklist

- [ ] `pio run -e esp32` clean
- [ ] `pio test -e native` passes (S4 sweep + reference tests)
- [ ] Self-check at first axis-config commit logs "OK" for all
      bundled real configs
- [ ] Pre/post sled-position trace diff < 1 µm at every sample
- [ ] FFB cycle timer shows measurable drop in average µs/tick
- [ ] Deliberately bad coefficient triggers the fallback and the
      `double` path is taken (one-shot manual test)
