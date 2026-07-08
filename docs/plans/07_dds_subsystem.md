# DDS Vibration Subsystem

Coherent, phase-locked vibration rendering for flight FFB axes. Adds a
new force element to the ESP32 that synthesises multi-harmonic sinusoidal
forces tracking live rotor/engine RPM, parameterised from the SimHub graph.

This is the shared infrastructure used by helicopter vibration (plan 08)
and fixed-wing vibration (plan 09).

**Prerequisites:** Plan 03 (ConfigOut & FunctionScope)

**Status:** Implemented. Phase 1 (firmware-side SyncVib + protobuf), phase 3
(gateway phase sync), and phase 2 (graph integration) all landed on
branch `ck_sync_vib`. Sub-plans for the implementation:

* [07a_dds_phase1_implementation.md](07a_dds_phase1_implementation.md)
* [07b_dds_phase2_implementation.md](07b_dds_phase2_implementation.md)
* [07c_dds_phase3_implementation.md](07c_dds_phase3_implementation.md)

Several decisions in this spec were superseded during implementation —
see notes inline below. Hardware end-to-end verification is pending.


---

## 1. Motivation

The existing `Buffet` element produces band-limited noise (5-25 Hz), which is
appropriate for incoherent/chaotic vibration cues (stall, ground rumble,
retreating blade stall onset) but cannot render:

* Rotor-rate coherent vibration (1/rev, 2/rev, ..., N/rev, 2N/rev)
* Frequencies above 25 Hz (blade-passing N/rev is ~40 Hz on a 5-blade rotor)
* Phase-locked tracking of rotor RPM through governor droop, autorotation,
  startup/shutdown
* Correct 90-degree phase relationship between pitch and roll axes for 1/rev
* Engine vibration with beat frequency between twin engines

Coherent vibration is the single largest "realism delta" for both helicopter
and fixed-wing FFB — it's what distinguishes "force feedback stick" from
"living machine."


---

## 2. Architecture

### Signal flow

```
 SimHub graph                                ESP32
 (20 Hz)                                     (1 kHz force loop)

 rotor RPM  --> fundamental_hz  --+
 disc tilt  --> amp_1rev         +-> FlightFfbAction --> DDS oscillator --> force
 IAS/qhat   --> amp_2rev, 3rev    |   (protobuf/          (per axis)
 const      --> amp_Nrev, 2Nrev  -+    USB serial)
                                                              |
                                                              v
                                                CenteringSpring
                                                Damper, Friction
                                                ConstForce, Buffet
                                                SyncVib  <-- NEW
                                                SUM -> m_eff -> pos
```

### Rate hierarchy

* **1 kHz**: DDS phase accumulator, sine/cosine evaluation, force summation
* **20 Hz**: Amplitude and fundamental frequency updates from graph
* **Static**: Blade count and rotation direction per aircraft (set at load)


---

## 3. Phase Sync Across Axes

Each flight function runs on a separate ESP32 axis controller with an
independent clock. For correct 1/rev phase relationship (90-degree
pitch/roll offset), all axes must share a phase reference.

**Approach: gateway phase broadcast + local PLL.**

The gateway ESP32 maintains the single authoritative DDS phase accumulator,
advancing at `fundamental_hz` from the latest FlightFfbAction. Every CAN
state broadcast cycle (~10 ms), the gateway includes the current rotor
phase (uint16, 0..65535 mapping to 0..2pi) in a sync message.

Each axis controller runs its own local DDS at 1 kHz but does NOT snap
directly to the received phase (that would cause audible clicks from
quantisation and jitter). Instead, it runs a software PLL that smoothly
steers the local oscillator toward the gateway reference:

```
On each gateway sync receipt (100 Hz):
    phase_error = wrap_to_pm_pi(gateway_phase - local_phase)
    error_integral += phase_error * dt_sync

On each force loop tick (1 kHz):
    f_adjusted = fundamental_hz + Kp * phase_error + Ki * error_integral
    local_phase += 2 * pi * f_adjusted * dt
    (wrap local_phase to 0..2pi)
```

**PLL parameters:**

| Parameter | Value | Rationale |
| --- | --- | --- |
| Kp | 10 Hz/rad | Loop bandwidth ~5 Hz -- fast enough to track RPM governor droop, well below 100 Hz sync rate |
| Ki | 20 Hz/rad/s | Eliminates steady-state phase error from crystal frequency offset |
| Lock-in time | ~200 ms | From cold start (arbitrary initial phase) to <2-degree error |
| Steady-state error | <1 degree | At 100 Hz sync and 8 Hz fundamental, 10 ms between syncs = ~29 degrees of free-run, PLL corrects well within that |

The PLL naturally handles:
- **RPM changes**: local frequency tracks `fundamental_hz` immediately, PLL
  corrects any accumulated phase drift from the transient
- **Startup**: converges from arbitrary initial phase without discontinuity
- **Gateway dropout**: if no sync arrives, PLL holds last frequency and free-runs
  (graceful degradation, same as a real PLL losing reference)
- **CAN jitter**: Kp bandwidth filters out sub-ms timing variations

The gateway needs `fundamental_hz` to run its master DDS. It extracts this
from the FlightFfbAction stream (the gateway sees all FFB messages for all
functions).

Each axis uses its local PLL-locked phase to evaluate harmonics. The pitch
axis uses `sin(phase)` for 1/rev and the roll axis uses
`cos(phase) * rotation_sign` — the sin/cos split is per-function, not
computed centrally.

**Fallback (no gateway / standalone axis):** DDS free-runs from phase zero.
1/rev ellipse orientation is arbitrary but stable. Adequate for single-axis
setups or non-critical applications.


---

## 4. Protocol

### Config (static, sent on aircraft load)

`FlightStickConfig` carries the per-axis vibration ratios and phase
offset:

```protobuf
message FlightStickConfig {
  int32 pos_min = 1;
  int32 pos_max = 2;
  float damping = 3;
  float centering_spring_const = 4;
  // Vibration phase offset in radians, applied uniformly to all
  // harmonics. Encodes both axis (pitch=0, roll=±π/2) and rotor
  // handedness — replaces the originally-specified rotation_sign field.
  float phase_offset = 5;
  // multipliers on fundamental per DDS 1 slot, max 5
  repeated float vib_harmonic_ratios = 6;
  // multipliers on fundamental per DDS 2 slot, max 5
  repeated float vib2_harmonic_ratios = 7;
}
```

Set once when aircraft profile is loaded. Each slot's frequency is
`fundamental_hz * harmonic_ratio`. The number of active slots is
determined by the array length (up to 5 per DDS).

**Harmonic ratios replace `blade_count`.** The graph or profile sets the
ratios directly — a 5-blade helicopter uses `[1.0, 2.0, 3.0, 5.0, 10.0]`,
a 2-blade uses `[1.0, 2.0, 3.0, 2.0, 4.0]`. This is more general and
enables non-integer ratios for geared systems (e.g., tail rotor).

> **Implementation divergence — phase offset replaces rotation_sign.**
> The originally-spec'd `int32 rotation_sign = 5` was dropped during phase 1
> in favour of `float phase_offset = 5` (radians on the wire, degrees at
> the plugin override layer). All slots use the uniform expression
> `sin(ratio·phase + phase_offset)` — no axis-split branch in firmware.
> The handedness of helicopter rotors is encoded in the sign of phase_offset
> (+π/2 vs -π/2 for roll). See section 6 below.

### Streaming (per FlightFfbAction, ~20 Hz)

`FlightFfbAction` carries per-axis vibration amplitudes only. The
fundamental frequency is sent in a separate `DdsFundamentals` message
(see below) — gateway snoop reads it from there.

```protobuf
message FlightFfbAction {
  float k_spring = 1;
  float k_damper = 2;
  float trim_offset = 3;
  float buffet_amp = 4;
  float load_force = 5;
  float k_friction = 6;
  // DDS 1 amplitudes, uint8 at 0.01 mm/LSB (range 0..2.55 mm).
  // Plugin pre-scales (×100); firmware reads raw and multiplies by 0.01.
  // SyncVib output is a position delta on the servo command path
  // (plan 12 — feel decoupled from damping).
  uint32 vib_amp_slot1 = 7;
  uint32 vib_amp_slot2 = 8;
  uint32 vib_amp_slot3 = 9;
  uint32 vib_amp_slot4 = 10;
  uint32 vib_amp_slot5 = 11;
  // DDS 2 amplitudes, same encoding
  uint32 vib2_amp_slot1 = 12;
  uint32 vib2_amp_slot2 = 13;
}
```

The `uint32` fields are constrained to 8-bit storage by
`int_size:IS_8` in `diy_ffb_protocol.options`.

Both DDS frequencies and phases come from the gateway sync frame (`0x0F0`),
not from FlightFfbAction. This keeps all timing in one place and the per-axis
message carries only amplitudes.

Amplitudes of zero disable the respective harmonic. All amplitudes zero
effectively disables the oscillator for that axis.

### Dedicated fundamentals message (plugin → gateway, ~50 Hz)

The plugin emits a small standalone message every FFB tick carrying
the master DDS fundamentals. The gateway (in `CommManager::on_gateway_message`)
snoops this and feeds its `MasterDds`; the gateway never reads the
fundamental from FFB action messages.

```protobuf
message DdsFundamentals {
  float dds1_fundamental_hz = 1;
  float dds2_fundamental_hz = 2;
}

// In Message.payload oneof:
DdsFundamentals dds_fundamentals = 14;
```

> **Implementation divergence — dedicated message instead of FFB-stamped.**
> Original spec embedded `vib_fundamental_hz` / `vib2_fundamental_hz` in
> every FlightFfbAction. That was changed during phase 2 because the
> fundamentals are global (not per-axis) — stamping them into N FFB messages
> per tick was redundant. Now: one `DdsFundamentals` message per tick.
> Tags 14, 15 in `FlightFfbAction` are reserved.

### Gateway → axis CAN frames

Gateway-to-axis FFB transport uses the existing `0x200 + (FFBFrameTypes::X << 4) + func` ID format. Phase 2 adds a third sub-type for vibration amplitudes:

```text
FFBFrameTypes::FLIGHT_FFB      = 1   spring, damper, trim, buffet     (8 bytes)
FFBFrameTypes::FLIGHT_FFB_LOAD = 2   load_force, k_friction           (4 bytes)
FFBFrameTypes::FLIGHT_VIB      = 3   5+2 amps, raw 0.01 mm/LSB        (7 bytes)  NEW
```

The axis-side `FlightFfbCache` stitches the three sub-frames; on each
arrival the cache emits a fully-assembled `FFBAction` to `on_ffb_action`.
Vib amps survive across frames via dedicated cache fields.

### Gateway sync message (dedicated CAN frame)

A new high-priority CAN frame at **`0x0F0`**, broadcast by the gateway at
**100 Hz** (10 ms interval). Carries the authoritative DDS state for both
oscillator slots — used for rotor vibration (helicopters) and engine
vibration (fixed-wing, including dual-engine beat).

```
CAN ID: 0x0F0 (high priority — above FFB actions, below axis state)
Rate:   100 Hz (10 ms)

Byte  Field                  Type     Encoding
----  ---------------------  ------   ---------------------------
0-1   dds1_fundamental_hz    uint16   0.001 Hz/LSB (0..65.535 Hz), 0 = disabled
2-3   dds1_phase             uint16   0..65535 maps to 0..2*pi
4-5   dds2_fundamental_hz    uint16   0.001 Hz/LSB, 0 = disabled
6-7   dds2_phase             uint16   0..65535 maps to 0..2*pi
```

Exactly 8 bytes — fits standard CAN frame with no waste. Frequency
resolution: 0.001 Hz = 0.06 RPM, adequate for both rotor and engine tracking.

CAN bus ID allocation after this change:

```
0x0F0           Gateway → All    DDS sync, dual oscillator (NEW, 100 Hz)
0x100 + axis    Axis → Gateway   High-prio axis frames
0x200 + func    Gateway → Axis   FFB action frames
0x300 + axis    Axis → Gateway   Low-prio axis frames
0x700 + axis    Gateway ↔ Axis   ISOTP (config, logging)
0x7FE           Gateway → All    Ping (presence detection, 10 Hz)
```

**DDS slot assignment:**

| Slot | Helicopter | Fixed-wing single | Fixed-wing twin |
| --- | --- | --- | --- |
| DDS 1 | Main rotor (5 harmonics) | Engine 1 (1-2 harmonics) | Engine 1 |
| DDS 2 | Engine or tail rotor | Disabled (freq = 0) | Engine 2 |

The gateway maintains two master DDS phase accumulators. It extracts
`VibFundamental` (DDS 1) and `Vib2Fundamental` (DDS 2) from the
FlightFfbAction stream and broadcasts both frequencies and phases.

Each axis PLL-locks both local oscillators to the sync frame. For engine
vibration the phase sync is not physically necessary but comes for free
and adds no overhead (one PI loop per DDS per tick).

`dds1_fundamental_hz = 0` disables DDS 1; `dds2_fundamental_hz = 0`
disables DDS 2. Both zero = no periodic vibration (pure spring/damper mode).

### Timeout behaviour

If no FlightFfbAction arrives within 200 ms, the ESP32 fades all harmonic
amplitudes to zero with a 100 ms time constant. Matches existing FFB timeout
behaviour.


---

## 5. Graph Output Interface

### Per flight function (amplitudes — Scoped Output)

Wired via Scoped Output nodes; routed by the parent Include's
`FunctionScope`.

SyncVib output is a position delta (mm) on the servo command path; see
plan 12 — feel decoupled from damping.

| Output | Proto field | DDS | Unit |
| --- | --- | --- | --- |
| `VibSlot1` | `vib_amp_slot1` | 1 | mm |
| `VibSlot2` | `vib_amp_slot2` | 1 | mm |
| `VibSlot3` | `vib_amp_slot3` | 1 | mm |
| `VibSlot4` | `vib_amp_slot4` | 1 | mm |
| `VibSlot5` | `vib_amp_slot5` | 1 | mm |
| `Vib2Slot1` | `vib2_amp_slot1` | 2 | mm |
| `Vib2Slot2` | `vib2_amp_slot2` | 2 | mm |

### Shared scope (fundamentals — plain Output node)

Fundamentals are global, not per-function. They live in a `Shared.*`
scope and are wired via the plain (non-Scoped) Output node. The
plugin reads them once per FFB tick and emits a `DdsFundamentals`
message; the gateway snoops, advances `MasterDds`, and broadcasts via
the `0x0F0` sync frame.

| Output | Wire path | Unit |
| --- | --- | --- |
| `Shared.VibFundamental` | `DdsFundamentals.dds1_fundamental_hz` → CAN `0x0F0` | Hz |
| `Shared.Vib2Fundamental` | `DdsFundamentals.dds2_fundamental_hz` → CAN `0x0F0` | Hz |

`OutputSuffixes` excludes the `Shared.*` prefix so Scoped Output
dropdowns only show per-function suffixes.

Axes PLL-lock both local oscillators to the shared sync frame.

Add amplitude outputs to `GraphSignalCatalogData.OutputNames` for each
function group. Follow the protocol extension checklist in
Flight_FFB_Architecture.md section 9.

### Config writes (ConfigOut, plan 03 infrastructure)

`FlightStickConfig.phase_offset` and the harmonic ratio arrays are
graph-driven via ConfigOut nodes:

| ConfigOut field path | Proto field | Layer | Unit |
| --- | --- | --- | --- |
| `flight_stick.phase_offset` | `phase_offset` | Profile | degrees (override layer); converted to radians at proto-build time |
| `flight_stick.vib_harmonic_ratios.0..4` | `vib_harmonic_ratios[N]` | Profile | unitless |
| `flight_stick.vib2_harmonic_ratios.0..1` | `vib2_harmonic_ratios[N]` | Profile | unitless |

Each ratio slot is registered as an independent scalar field
(`OverrideFieldRegistry` scalar decomposition) — N ConfigOut nodes
populate N slots. ConfigOut writes coalesce into a single
`FunctionConfig` upload per function via the throttled merger.


---

## 6. Vibration Slot Assignment Concept

The DDS subsystem provides **5 + 2 amplitude slots** across two oscillators.
Each slot is a "voice" — a sine wave at `fundamental_hz × ratio` with an
independently controllable amplitude. The ESP32 has no knowledge of what
physical source a slot represents; it just runs `sin(ratio * phase)` with
a smoothed amplitude. All meaning is established by the coordination of
two things set at profile load time:

1. **Harmonic ratios** in `FlightStickConfig` — what frequency each slot runs at
2. **Graph template** — what telemetry signal drives each slot's amplitude

These two must match. A profile bundles both: it writes the ratios to config
and uses a graph template that knows which envelope signal goes to which slot.

### Slot evaluation (uniform expression with phase offset)

All slots evaluate to a single uniform expression — no axis enum, no
sin/cos branch in firmware:

```text
force = amplitude[i] * sin(ratio[i] * phase + phase_offset)
```

The 1/rev disc tilt ellipse on a helicopter cyclic is achieved by
profile config:

* Pitch axis: `phase_offset = 0` → `sin(phase)` for 1/rev slot.
* Roll axis: `phase_offset = ±π/2` → `sin(phase ± π/2) = ±cos(phase)`
  for 1/rev slot. Sign encodes rotor handedness (CCW from above vs CW).
* Engine vibration: `phase_offset = 0` on both axes → isotropic.
* Pedals / collective: `phase_offset = 0` (single-DOF, no ellipse).

Higher harmonics inherit the same phase offset (`sin(2·phase + π/2)`
for 2/rev on roll, etc.). For pure rotor tilt modes this is physically
correct; for blade-passing aerodynamic loads it's a reasonable
approximation.

DDS 2's `SyncVib` instance always uses `phase_offset = 0` (engine
vibration is never directional).

> **Implementation divergence — uniform expression replaces axis split.**
> The originally-spec'd sin/cos axis split rule (gated by `rotation_sign`)
> was dropped during phase 1 in favour of a single `phase_offset` field.
> Firmware no longer carries an axis enum; the plugin profile expresses
> the desired pitch/roll relationship by writing the appropriate
> `phase_offset` value (in degrees) to each axis's FlightStickConfig.

See plan 08 and plan 09 for specific slot profiles per aircraft type.


---

## 7. ESP32 Implementation

### Vibration force bypass

> **Update (plan 12).** SyncVib no longer uses `f_vib`. Its output is now a
> position delta (mm) routed through `accum.x_vib`, captured by
> `Sim::update` into `_x_vib`, and added on the servo command path in
> `Main.cpp` after `calc_final_position`. This makes damping orthogonal to
> vibration amplitude. `f_vib` is retained for **Buffet only** (band-limited
> noise that still benefits from the post-friction force injection). The
> rest of this section describes the original force-bypass design.

The existing `Sim::update()` applies damping and friction after summing all
element forces. This would attenuate or completely absorb vibration forces
(Coulomb friction of 2-5 N swallows any harmonic below that amplitude).

Solution: add `f_vib` field to `SimAccumulators`. SyncVib accumulates into
`f_vib` instead of `f_sum`. In `Sim::update()`, `f_vib` is added AFTER
damping and friction processing:

```cpp
// In Physics.h:
struct SimAccumulators {
    float f_sum = 0.0f;
    float k_damp_sum = 0.0f;
    float f_static_sum = 0.0f;
    float f_kin_sum = 0.0f;
    float v_eps_max = 0.0f;
    float f_vib = 0.0f;         // NEW: bypasses damping and friction
    // ...
};

// In Sim::update(), after friction processing, before acceleration:
accum.f_sum += accum.f_vib;    // inject vibration force post-friction
_a = accum.f_sum / _m * 1000.0;
```

This also applies to `Sim::compute_force_sum()` (unit test path).

The existing `Buffet` element should also be migrated to use `f_vib` — it
has the same problem (incoherent noise eaten by friction). This is a
backward-compatible change: existing setups with no vibration have
`f_vib = 0` and behave identically.

### SyncVib class

New `SimElement` registered in `FlightStickFunction`:

```cpp
class SyncVib : public SimElement {
public:
    static constexpr uint8_t MAX_SLOTS = 5;

    void set_config(float phase_offset, const float *ratios, uint8_t num_slots);
    void set_amplitudes(const float *targets, uint8_t count);
    void on_sync(float gateway_phase, float gateway_hz);

    void update(const SimState &state, SimAccumulators &accum) override;

private:
    // Local DDS
    float _phase = 0.0f;
    float _fundamental_hz = 0.0f;

    // Configurable harmonic ratios (set once per aircraft load)
    float _ratios[MAX_SLOTS] = {};
    uint8_t _num_slots = 0;
    float _phase_offset = 0.0f;

    // PLL state
    float _phase_error = 0.0f;
    float _error_integral = 0.0f;
    static constexpr float PLL_KP = 10.0f;
    static constexpr float PLL_KI = 20.0f;
    static constexpr float PLL_INT_MAX = 5.0f;

    // Smoothed amplitudes (first-order LPF, tau ~50 ms)
    float _amp[MAX_SLOTS] = {};
    float _target[MAX_SLOTS] = {};

    static float wrap_pm_pi(float x);
};
```

> **Implementation divergence.** No `_axis` member, no `_rotation_sign`
> member, no `set_axis()` method. The `phase_offset` field stored on
> the instance (set via `set_config`) drives the uniform sin expression.

`FlightStickFunction` holds two `SyncVib` instances:

| Slot | Helicopter use | Fixed-wing use |
| --- | --- | --- |
| DDS 1 | Main rotor (5 harmonics, PLL-synced) | Engine 1 (1-2 harmonics, free-run) |
| DDS 2 | Engine or tail rotor | Engine 2 (1-2 harmonics, free-run) |

### PLL sync

Called on each gateway CAN sync receipt (~100 Hz):

```cpp
void SyncVib::on_sync(float gateway_phase, float gateway_hz) {
    _fundamental_hz = gateway_hz;
    _phase_error = wrap_pm_pi(gateway_phase - _phase);
    _error_integral += _phase_error * 0.01f;  // dt_sync ~ 10 ms
    // Anti-windup: clamp integral to prevent overshoot on large transients
    _error_integral = fmaxf(-PLL_INT_MAX / PLL_KI,
                      fminf(PLL_INT_MAX / PLL_KI, _error_integral));
}

float SyncVib::wrap_pm_pi(float x) {
    x = fmodf(x + (float)M_PI, 2.0f * (float)M_PI);
    if (x < 0.0f) x += 2.0f * (float)M_PI;
    return x - (float)M_PI;
}
```

### DDS oscillator with PLL correction

Phase accumulator advances at 1 kHz, frequency adjusted by PLL:

```cpp
void SyncVib::update(const SimState &state, SimAccumulators &accum) {
    float dt_s = state.dt_ms * 0.001f;

    // PLL-corrected frequency: nominal + proportional + integral correction
    float f_adj = _fundamental_hz
                + PLL_KP * _phase_error
                + PLL_KI * _error_integral;
    if (f_adj < 0.0f) f_adj = 0.0f;  // don't run backwards

    // Advance phase
    _phase += 2.0f * M_PI * f_adj * dt_s;
    if (_phase > 2.0f * M_PI) _phase -= 2.0f * M_PI;
    if (_phase < 0.0f) _phase += 2.0f * M_PI;

    // Smooth amplitudes (first-order LPF, tau = 50 ms)
    float a = dt_s / (0.05f + dt_s);
    for (int i = 0; i < _num_slots; i++) {
        _amp[i] += (_target[i] - _amp[i]) * a;
    }

    // Evaluate harmonics — each slot has a configurable ratio.
    // Uniform expression for all slots; axis-vs-axis relationship is
    // entirely encoded in _phase_offset (set per-axis via FlightStickConfig).
    float f = 0.0f;
    for (int i = 0; i < _num_slots; i++) {
        f += _amp[i] * fastmath::fast_sinf(_ratios[i] * _phase + _phase_offset);
    }

    accum.f_vib += f;  // bypass damping and friction
}
```

### PLL behaviour summary

| Scenario | Behaviour |
| --- | --- |
| Cold start | PLL converges from arbitrary phase in ~200 ms, no click |
| Steady state | <1 degree inter-axis error, frequency = nominal |
| RPM change | Kp tracks immediately, Ki corrects accumulated phase drift |
| Gateway dropout | PLL holds last frequency, free-runs (graceful degradation) |
| CAN jitter | Kp bandwidth (~5 Hz) filters sub-ms timing noise |
| Rotor shutdown | Amplitudes fade via LPF; PLL idles at f=0, phase frozen |

Optimisations (apply if 1 kHz loop budget is tight):

* Pre-compute `sin(k*phase)` using angle-addition (one `sinf`/`cosf` per tick)
* Sine LUT with linear interpolation (512 entries, ~4x faster than `sinf`)


---

## 8. Testing Plan

### DDS and PLL tests

1. **DDS continuity**: sweep `fundamental_hz` from 1 to 50 Hz over 10 s
   while logging force output. No discontinuities, no clicks.
2. **Phase relationship**: at steady 10 Hz, log pitch and roll output for
   one second. Verify 90-degree offset on 1/rev, zero offset on others.
3. **Amplitude smoothing**: send step change 0 to full amplitude. Verify
   smooth envelope with no click.
4. **Timeout**: cut FlightFfbAction stream, verify fade-to-zero in <150 ms.
5. **PLL lock-in**: two axes, arbitrary initial phase, verify convergence
   to <2-degree inter-axis error within 300 ms.
6. **PLL steady state**: at 8 Hz fundamental, verify <1-degree inter-axis
   phase error over 60 seconds.
7. **PLL during RPM sweep**: sweep fundamental 5-12 Hz over 10 s, verify
   no clicks and inter-axis error stays <5 degrees during transient.
8. **PLL integral clamp under RPM transient**: verify that the integral
   anti-windup clamp (`PLL_INT_MAX / PLL_KI = 0.25 rad·s`) does not
   cause convergence problems during realistic RPM transients. Test
   scenarios:
   * **Governor droop**: step fundamental from 8.2 Hz to 7.5 Hz (100%
     to ~91% RPM) over 500 ms. Verify PLL tracks without overshoot or
     sustained phase error after the transient settles.
   * **Autorotation entry**: ramp fundamental from 8.2 Hz to 6.5 Hz
     over 2 s (governor failure, RPM decaying). Verify integrator does
     not saturate — if it does, the PLL cannot eliminate steady-state
     frequency error from crystal offset during the transient.
   * **Startup from zero**: ramp fundamental 0 → 8.2 Hz over 5 s.
     Verify smooth phase lock acquisition with no overshoot or
     oscillation at the integrator clamp boundary.
   * **Measurement**: log `_error_integral` alongside phase error. If
     the integral hits the clamp rail during any realistic transient,
     increase `PLL_INT_MAX` (e.g., 10.0) and re-test steady-state
     to ensure no overshoot is introduced.


---

## 9. Implementation

All three phases are landed on `ck_sync_vib`. See the per-phase
sub-plans (07a, 07b, 07c) for the actual file-by-file edit history
and any divergences from this spec.

### Phase 1 — core rendering ([07a](07a_dds_phase1_implementation.md))

1. Add `f_vib` field to `SimAccumulators`, inject after damping/friction in `Sim::update()`
2. Migrate existing `Buffet` element to use `f_vib` instead of `f_sum`
3. `SyncVib` class on ESP32 (DDS, PLL, configurable ratio slots, amplitude smoothing → `f_vib`)
4. Wire two `SyncVib` instances into `FlightStickFunction` (DDS 1 + DDS 2)
5. Extend `FlightFfbAction` protobuf with DDS 1 + DDS 2 amplitude fields
6. Extend `FlightStickConfig` with `phase_offset` and `vib_harmonic_ratios` /
   `vib2_harmonic_ratios`
7. Native unit tests for SyncVib (DDS continuity, phase offset, amplitude smoothing)

### Phase 3 — gateway phase sync ([07c](07c_dds_phase3_implementation.md))

1. New `MasterDds` class hosted in `CommManager` (pure model, no transport
   knowledge)
2. New `ICommChannel::send_dds_sync` virtual + `OnDdsSync` callback
3. `CANManager::send_dds_sync` packs and emits `0x0F0` at 100 Hz; axis-side
   `try_process_dds_sync_frame` parses and fires the callback
4. `IFunction::on_dds_sync` virtual no-op; `FlightStickFunction` overrides
   to dispatch into `vib1.on_sync` / `vib2.on_sync`
5. CommManager periodic_task ticks MasterDds at 1 ms and emits 100 Hz

### Phase 2 — graph integration ([07b](07b_dds_phase2_implementation.md))

1. Proto reshape: drop fundamentals from `FlightFfbAction` (tags 14, 15 reserved),
   trim amps to 8-bit (`int_size:IS_8`, 0.01 N/LSB), add new `DdsFundamentals`
   message
2. New `FFBFrameTypes::FLIGHT_VIB = 3` CAN frame; gateway packs amps and
   axis-side cache stitches across FFB / FFB_LOAD / FLIGHT_VIB
3. Gateway snoop moves from `Message_ffb_action_tag` to `Message_dds_fundamentals_tag`
4. Plugin signal catalog gains per-function `VibSlot1..5` / `Vib2Slot1..2` and
   shared `Shared.VibFundamental` / `Shared.Vib2Fundamental`; `OutputSuffixes`
   filters out `Shared.*`
5. Plugin `SendDdsFundamentals` emits the new message once per FFB tick;
   `SendFlightFfb` pre-scales amps (×100, clamped to 0..255)
6. ConfigOut wiring for `flight_stick.phase_offset` (degrees at the
   override layer, radians at the proto), `flight_stick.vib_harmonic_ratios.0..4`,
   `flight_stick.vib2_harmonic_ratios.0..1`


---

## 10. Non-Goals

* Buffet replacement. Band-limited noise stays for incoherent cues.
* Acoustic output / cabin noise simulation.
* Non-sinusoidal waveshapes (triangle, saw). Stick to fundamentals
  + harmonics; any waveshape can be approximated with enough harmonics.


---

## 11. Open Questions

1. **82 Hz feasibility**: 2N/rev on a 5-blade rotor at nominal RPM is 82 Hz.
   The servo/mechanical system may attenuate forces above ~50 Hz
   significantly. Need bench test to determine useful upper frequency.
