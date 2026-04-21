# DDS Vibration Subsystem

Coherent, phase-locked vibration rendering for flight FFB axes. Adds a
new force element to the ESP32 that synthesises multi-harmonic sinusoidal
forces tracking live rotor/engine RPM, parameterised from the SimHub graph.

This is the shared infrastructure used by helicopter vibration (plan 08)
and fixed-wing vibration (plan 09).

**Prerequisites:** Plan 03 (ConfigOut & FunctionScope)


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

Extend `FlightStickConfig` in `diy_ffb_protocol.proto`:

```protobuf
message FlightStickConfig {
  int32 pos_min = 1;
  int32 pos_max = 2;
  float damping = 3;
  float centering_spring_const = 4;
  // NEW: rotor vibration config
  int32 rotation_sign = 5;              // +1 CCW-from-above (US), -1 CW (EU)
  repeated float vib_harmonic_ratios = 6;  // multipliers on fundamental per DDS 1 slot
  repeated float vib2_harmonic_ratios = 7; // multipliers on fundamental per DDS 2 slot
}
```

Set once when aircraft profile is loaded. Each slot's frequency is
`fundamental_hz * harmonic_ratio`. The number of active slots is
determined by the array length (up to 5 per DDS).

**Harmonic ratios replace `blade_count`.** The graph or profile sets the
ratios directly — a 5-blade helicopter uses `[1.0, 2.0, 3.0, 5.0, 10.0]`,
a 2-blade uses `[1.0, 2.0, 3.0, 2.0, 4.0]`. This is more general and
enables non-integer ratios for geared systems (e.g., tail rotor).

### Streaming (per FlightFfbAction, ~20 Hz)

Extend `FlightFfbAction` with per-axis vibration amplitudes only. The
fundamental frequency is NOT per-axis — it comes from the gateway sync
message to ensure all axes use identical DDS parameters.

```protobuf
message FlightFfbAction {
  float k_spring = 1;
  float k_damper = 2;
  float trim_offset = 3;
  float buffet_amp = 4;
  float load_force = 5;
  float k_friction = 6;
  // NEW: DDS 1 amplitudes (per axis, slot semantics set by vib_harmonic_ratios)
  float vib_amp_slot1 = 7;       // DDS 1 slot 1 amplitude (N)
  float vib_amp_slot2 = 8;       // DDS 1 slot 2 amplitude (N)
  float vib_amp_slot3 = 9;       // DDS 1 slot 3 amplitude (N)
  float vib_amp_slot4 = 10;      // DDS 1 slot 4 amplitude (N)
  float vib_amp_slot5 = 11;      // DDS 1 slot 5 amplitude (N)
  // NEW: DDS 2 amplitudes (engine 2 or tail rotor)
  float vib2_amp_slot1 = 12;     // DDS 2 slot 1 amplitude (N)
  float vib2_amp_slot2 = 13;     // DDS 2 slot 2 amplitude (N)
}
```

Both DDS frequencies and phases come from the gateway sync frame (`0x0F0`),
not from FlightFfbAction. This keeps all timing in one place and the per-axis
message carries only amplitudes.

Amplitudes of zero disable the respective harmonic. All amplitudes zero
effectively disables the oscillator for that axis.

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

### Per flight function (amplitudes only)

| Output | Proto field | DDS | Unit |
| --- | --- | --- | --- |
| `VibSlot1` | `vib_amp_slot1` | 1 | N |
| `VibSlot2` | `vib_amp_slot2` | 1 | N |
| `VibSlot3` | `vib_amp_slot3` | 1 | N |
| `VibSlot4` | `vib_amp_slot4` | 1 | N |
| `VibSlot5` | `vib_amp_slot5` | 1 | N |
| `Vib2Slot1` | `vib2_amp_slot1` | 2 | N |
| `Vib2Slot2` | `vib2_amp_slot2` | 2 | N |

### Shared (routed to gateway for sync frame broadcast)

| Output | Sync frame field | Unit |
| --- | --- | --- |
| `VibFundamental` | `dds1_fundamental_hz` | Hz |
| `Vib2Fundamental` | `dds2_fundamental_hz` | Hz |

The plugin sends both fundamentals to the gateway (via any one
FlightFfbAction or a dedicated path). The gateway runs two master DDS
accumulators and broadcasts both frequency/phase pairs in the `0x0F0`
sync frame. Axes PLL-lock both local oscillators.

Add amplitude outputs to `GraphSignalCatalogData.OutputNames` for each
function group. Follow the protocol extension checklist in
Flight_FFB_Architecture.md section 9.


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

### Axis split rule

The 1/rev sin/cos axis split (disc tilt ellipse) is gated by
`rotation_sign != 0`:

```
if rotation_sign != 0 AND ratio ≈ 1.0 AND axis == ROLL:
    force = rotation_sign * amplitude * cos(phase)
else:
    force = amplitude * sin(ratio * phase)
```

This ensures:

* **Helicopter rotor** (`rotation_sign = ±1`): 1/rev slot produces
  sin on pitch, cos on roll → elliptical disc tilt feel
* **Engine vibration** (`rotation_sign = 0`): 1/rev slot produces
  sin on both axes → isotropic vibration (no ellipse)
* **Pedal axis** (`rotation_sign = 0`): single axis, no split needed

Each `SyncVib` instance carries its own `rotation_sign`. DDS 2's instance
always uses `rotation_sign = 0` (engine vibration is never directional).

See plan 08 and plan 09 for specific slot profiles per aircraft type.


---

## 7. ESP32 Implementation

### Vibration force bypass

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

    void set_config(int8_t rotation_sign, const float *ratios, uint8_t num_slots);
    void set_amplitudes(const float *targets, uint8_t count);
    void on_sync(float gateway_phase, float gateway_hz);

    void update(const SimState &state, SimAccumulators &accum) override;

    enum Axis { PITCH, ROLL };
    void set_axis(Axis axis);

private:
    // Local DDS
    float _phase = 0.0f;
    float _fundamental_hz = 0.0f;
    int8_t _rotation_sign = 1;
    Axis _axis = PITCH;

    // Configurable harmonic ratios (set once per aircraft load)
    float _ratios[MAX_SLOTS] = {};
    uint8_t _num_slots = 0;

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

    // Evaluate harmonics — each slot has a configurable ratio
    float f = 0.0f;
    for (int i = 0; i < _num_slots; i++) {
        float h = _ratios[i];
        if (_rotation_sign != 0 && fabsf(h - 1.0f) < 0.01f && _axis == ROLL) {
            // 1/rev on roll axis with rotor: cos with rotation sign (disc tilt ellipse)
            f += _rotation_sign * _amp[i] * cosf(_phase);
        } else {
            // All other cases: sin, same phase on both axes (isotropic)
            f += _amp[i] * sinf(h * _phase);
        }
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

### Phase 1 — core rendering

1. Add `f_vib` field to `SimAccumulators`, inject after damping/friction in `Sim::update()`
2. Migrate existing `Buffet` element to use `f_vib` instead of `f_sum`
3. `SyncVib` class on ESP32 (DDS, PLL, configurable ratio slots, amplitude smoothing → `f_vib`)
4. Wire two `SyncVib` instances into `FlightStickFunction` (DDS 1 + DDS 2)
5. Extend `FlightFfbAction` protobuf with DDS 1 + DDS 2 amplitude fields
6. Extend `FlightStickConfig` with `rotation_sign` and `vib_harmonic_ratios`
7. Bench test with fixed amplitudes and ratios (no graph integration yet)

### Phase 2 — graph integration

1. Add graph outputs (`VibSlot1..5`, `Vib2Slot1..2`) to signal catalog
2. Add `TryGetGraphVibOutputs()` and `SendFlightFfb()` plumbing
3. Wire ConfigOut nodes for vibration ratios (uses plan 03 infrastructure)

### Phase 3 — phase sync

1. Add `send_rotor_sync_frame()` on gateway at 100 Hz (CAN ID `0x0F0`)
2. Gateway runs master DDS, packs `fundamental_hz` + `phase` into frame
3. Add `try_process_rotor_sync_frame()` on axes to extract DDS fields
4. Wire extracted frequency + phase into axis `SyncVib::on_sync()`
5. Test inter-axis PLL lock-in and steady-state coherence


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
