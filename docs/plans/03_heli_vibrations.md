# Rotor Vibration Subsystem Plan

Coherent, rotor-phase-locked vibration rendering for flight FFB axes. Adds a
new force element to the ESP32 that synthesises multi-harmonic sinusoidal
forces tracking live rotor RPM, parameterised from the SimHub graph.


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

Coherent rotor vibration is the single largest "realism delta" for
unboosted/low-boost helicopter feel -- it's what distinguishes "force feedback
stick" from "living rotor system."


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
                                                RotorVib  <-- NEW
                                                SUM -> m_eff -> pos
```

### Rate hierarchy

* **1 kHz**: DDS phase accumulator, sine/cosine evaluation, force summation
* **20 Hz**: Amplitude and fundamental frequency updates from graph
* **Static**: Blade count and rotation direction per aircraft (set at load)

### Phase sync across axes

Each flight function runs on a separate ESP32 axis controller with an independent
clock. For correct 1/rev phase relationship (90-degree pitch/roll offset), all
axes must share a phase reference.

**Approach: gateway phase broadcast + local PLL.**

The gateway ESP32 maintains the single authoritative DDS phase accumulator,
advancing at `fundamental_hz` from the latest FlightFfbAction. Every CAN state
broadcast cycle (~10 ms), the gateway includes the current rotor phase (uint16,
0..65535 mapping to 0..2pi) in a sync message.

Each axis controller runs its own local DDS at 1 kHz but does NOT snap directly
to the received phase (that would cause audible clicks from quantisation and
jitter). Instead, it runs a software PLL that smoothly steers the local
oscillator toward the gateway reference:

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

The gateway needs `fundamental_hz` to run its master DDS. It extracts this from
the FlightFfbAction stream (the gateway sees all FFB messages for all functions).

Each axis uses its local PLL-locked phase to evaluate harmonics. The pitch axis
uses `sin(phase)` for 1/rev and the roll axis uses `cos(phase) * rotation_sign`
-- the sin/cos split is per-function, not computed centrally.

**Fallback (no gateway / standalone axis):** DDS free-runs from phase zero. 1/rev
ellipse orientation is arbitrary but stable. Adequate for single-axis setups or
non-critical applications.


---

## 3. Protocol

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
enables non-integer ratios for geared systems.

**Examples:**

| Aircraft | DDS 1 ratios | DDS 2 ratios | Notes |
| --- | --- | --- | --- |
| MD 500E cyclic | [1.0, 2.0, 3.0, 5.0, 10.0] | — | 5-blade, 1/2/3/N/2N |
| MD 500E pedals | [1.0, 2.0, 4.62, 9.24] | — | Slot 3-4: tail rotor at 4.62:1 gear ratio |
| R22 cyclic | [1.0, 2.0, 3.0, 2.0, 4.0] | — | 2-blade, slots 4-5 = N/2N = 2/4 |
| Baron 58 pitch | [1.0, 2.0] | [1.0, 2.0] | Twin piston, 1/rev + 2/rev per engine |
| PT6 turboprop | [1.0] | — | Smooth turbine, 1/rev only |

**Tail rotor vibration via gear ratio:** for helicopter pedals, the tail
rotor frequency is `main_rotor_fundamental * gear_ratio`. By setting a
harmonic slot to the gear ratio (e.g., 4.62 for MD 500E), the tail rotor
blade-passing vibration rides on the main rotor's phase-synced DDS. No
second DDS needed for tail rotor — it's just another harmonic of the
fundamental.

The 1/rev sin/cos axis split (for disc tilt ellipse) applies ONLY to
slots with ratio = 1.0. All other ratios are evaluated identically on
both pitch and roll axes (isotropic vibration).

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
  // NEW: DDS 1 amplitudes (rotor or engine 1, per axis)
  float vib_amp_1rev = 7;        // 1/rev amplitude (N)
  float vib_amp_2rev = 8;        // 2/rev amplitude (N)
  float vib_amp_3rev = 9;        // 3/rev amplitude (N)
  float vib_amp_nrev = 10;       // N/rev amplitude (N)
  float vib_amp_2nrev = 11;      // 2N/rev amplitude (N)
  // NEW: DDS 2 amplitudes (engine 2, per axis)
  float vib2_amp_1rev = 12;      // 1/rev amplitude (N)
  float vib2_amp_2rev = 13;      // 2/rev amplitude (N)
}
```

Both DDS frequencies and phases come from the gateway sync frame (`0x0F0`),
not from FlightFfbAction. This keeps all timing in one place and the per-axis
message carries only amplitudes.

Slot semantics are fixed: slot 2 is always 2/rev, slot N is always N/rev
(as defined by blade_count). For 2-blade rotors slots 2 and N render the
same frequency and their amplitudes sum -- graph can assign to either.

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

The existing ping frame (`0x7FE`, 10 Hz) is unchanged — it continues to
serve gateway presence detection only.

### Timeout behaviour

If no FlightFfbAction arrives within 200 ms, the ESP32 fades all harmonic
amplitudes to zero with a 100 ms time constant. Matches existing FFB timeout
behaviour.


---

## 4. ESP32 implementation

### Vibration force bypass

The existing `Sim::update()` applies damping and friction after summing all
element forces. This would attenuate or completely absorb vibration forces
(Coulomb friction of 2-5 N swallows any harmonic below that amplitude).

Solution: add `f_vib` field to `SimAccumulators`. RotorVib accumulates into
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

### RotorVib class

New `SimElement` registered in `FlightStickFunction`:

```cpp
class RotorVib : public SimElement {
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

### PLL sync

Called on each gateway CAN sync receipt (~100 Hz):

```cpp
void RotorVib::on_sync(float gateway_phase, float gateway_hz) {
    _fundamental_hz = gateway_hz;
    _phase_error = wrap_pm_pi(gateway_phase - _phase);
    _error_integral += _phase_error * 0.01f;  // dt_sync ~ 10 ms
    // Anti-windup: clamp integral to prevent overshoot on large transients
    _error_integral = fmaxf(-PLL_INT_MAX / PLL_KI,
                      fminf(PLL_INT_MAX / PLL_KI, _error_integral));
}

float RotorVib::wrap_pm_pi(float x) {
    while (x > M_PI)  x -= 2.0f * M_PI;
    while (x < -M_PI) x += 2.0f * M_PI;
    return x;
}
```

### DDS oscillator with PLL correction

Phase accumulator advances at 1 kHz, frequency adjusted by PLL:

```cpp
void RotorVib::update(const SimState &state, SimAccumulators &accum) {
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
        if (h == 1.0f && _axis == ROLL) {
            // 1/rev on roll axis: cos with rotation sign (90-deg offset for disc tilt ellipse)
            f += _rotation_sign * _amp[i] * cosf(_phase);
        } else {
            // All other slots: sin, same phase on both axes (isotropic)
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

## 5. SimHub graph integration

### New graph outputs

**Per flight function** (amplitudes only — frequencies come from gateway sync):

| Output | Proto field | DDS | Unit |
| --- | --- | --- | --- |
| `Vib1Rev` | `vib_amp_1rev` | 1 | N |
| `Vib2Rev` | `vib_amp_2rev` | 1 | N |
| `Vib3Rev` | `vib_amp_3rev` | 1 | N |
| `VibNRev` | `vib_amp_nrev` | 1 | N |
| `Vib2NRev` | `vib_amp_2nrev` | 1 | N |
| `Vib2Amp1Rev` | `vib2_amp_1rev` | 2 | N |
| `Vib2Amp2Rev` | `vib2_amp_2rev` | 2 | N |

**Shared (routed to gateway for sync frame broadcast):**

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

### New telemetry signals

All vibration envelope drivers are available as X-Plane datarefs. Verified
by cross-referencing DataRefs.txt against live flight data (MD 500E dataref
logger, ~9000 samples across hover through 170 kt and back).

**Confirmed datarefs for vibration:**

| Graph signal | X-Plane dataref | Type | Range (MD 500E) | Purpose |
| --- | --- | --- | --- | --- |
| `XPlane.Rotor.BladeAlphPitch` | `sim/flightmodel/cyclic/cyclic_elev_blad_alph[N]` | float[16] | -0.2 (hover) to -6.2 (170 kt) | **1/rev lateral amplitude** — per-axis blade alpha, scales with IAS like flapping |
| `XPlane.Rotor.BladeAlphRoll` | `sim/flightmodel/cyclic/cyclic_ailn_blad_alph[N]` | float[16] | -0.3 to +0.1 | 1/rev longitudinal amplitude — weak |
| `XPlane.Rotor.Slap` | `sim/flightmodel2/engines/rotor_blade_slap_rat[N]` | float[16] | 0 (hover) to 0.27 (high speed) | **2/rev high-speed envelope** |
| `XPlane.Rotor.VRS` | `sim/flightmodel/engine/vortex_ring_state[N]` | float[16][10] | 0.50 (hover) to 0.25 (60+ kt) | **2/rev ETL envelope** — transition zone 0.50->0.25 IS the ETL |
| `XPlane.Rotor.BladeAlpha` | `sim/flightmodel2/engines/rotor_blade_alpha_deg[N]` | float[16] | 2.3 to 4.7 | Retreating blade stall indicator |
| `XPlane.Rotor.DiscAlpha` | `sim/flightmodel2/engines/rotor_disc_alpha_deg[N]` | float[16] | -73 (hover) to -0.2 (90 kt) | Disc AoA — context for blade slap |
| `XPlane.Rotor.Propwash` | `sim/flightmodel2/engines/propwash_mtr_sec[N]` | float[16] | 19 (hover) to 2.5 (170 kt) | Downwash velocity — ground effect proxy |

**NOT available as datarefs** (internal X-Plane only, visible in data export):

* `pitch,_flap` / `_roll,_flap` (blade flapping angles) — not exposed.
  **Replaced by** `cyclic_elev_blad_alph` / `cyclic_ailn_blad_alph` which
  track the same envelope shape.
* `swirl,maxkt` (wake swirl velocity) — not exposed.
  **Replaced by** `vortex_ring_state` which provides a clean ETL indicator.

**Key observations from dataref logger flight data (MD 500E):**

* **`blade_alph_pitch` is the 1/rev driver.** Scales from -0.2 (hover) to
  -6.2 (170 kt), matching the `pitch_flap` envelope from the data export.
  This is the per-axis decomposition of blade AoA — it tracks flapping
  because flapping IS the blade's response to airspeed asymmetry.

* **`vortex_ring_state` is the ETL indicator.** Value 0.50 in hover (full
  recirculation), decays through 0.47 (20 kt) → 0.36 (40 kt) → 0.25
  (60+ kt, clean forward flight). The transition zone IS the ETL. Compute
  ETL factor as `(vrs - 0.25) / 0.25` → 1.0 at hover, 0.0 at 60+ kt.

* **`slap_rat` builds monotonically with speed** — 0 at hover, 0.02 at
  80 kt, 0.05 at 170 kt. Driven by blade-vortex interaction when
  `disc_alpha` is near zero (vortices not clearing the disc).

* **`Q_rotor` / `R_rotor` are too small** (±0.03 rad) and noisy to use as
  vibration envelopes. They represent rotor moments, not flapping directly.

* **All vibration-relevant signals naturally zero on the ground** (blade_alph
  decays when no aerodynamic asymmetry). No `OnGround` gate needed.

Derived signals (computed in plugin):

| Signal | Derivation | Unit |
| --- | --- | --- |
| `XPlane.Rotor.FundamentalHz` | `MainRotor.Speed / 60` | Hz |

**Observed data (MD 500E, averaged per 10-kt bin, airborne, RPM > 400):**

```
 IAS    blade_alph_pitch  blade_alph_roll   slap_rat     vrs_0   propwash
   0kt           -0.157           -0.333     0.0000      0.500     19.3
  20kt           -1.196           -1.100     0.0049      0.470      9.3
  40kt           -1.059           -0.573     0.0162      0.359      6.2
  60kt           -1.529           -0.193     0.0211      0.258      4.9
  80kt           -1.715           -0.321     0.0236      0.250      3.9
 100kt           -3.350           -0.201     0.0443      0.250      3.8
 120kt           -4.293           -0.309     0.0388      0.250      3.3
 140kt           -5.214           -0.283     0.0597      0.250      3.0
 160kt           -5.634           -0.009     0.0373      0.250      2.5
 170kt           -6.246           -0.027     0.0518      0.250      2.6
```

### New library block: `heli_vibration.json`

Reusable sub-graph that takes rotor state and produces all six vibration outputs.

Inputs:

* `rpm` (from `XPlane.MainRotor.Speed`)
* `blade_alph_pitch` (from `XPlane.Rotor.BladeAlphPitch`)
* `blade_alph_roll` (from `XPlane.Rotor.BladeAlphRoll`)
* `slap_rat` (from `XPlane.Rotor.Slap`)
* `vrs` (from `XPlane.Rotor.VRS`)
* `blade_alpha` (from `XPlane.Rotor.BladeAlpha`)

Internal computation — amplitude envelopes driven by X-Plane datarefs:

```
VibFundamental = rpm / 60

-- 1/rev: from per-axis blade alpha (proxy for flapping)
Vib1Rev_roll  = gain_1rev * abs(blade_alph_pitch)   # lateral (dominant)
Vib1Rev_pitch = gain_1rev * abs(blade_alph_roll)     # longitudinal (weak)

-- 2/rev: two components summed
--   ETL component: vrs transition zone (0.50 -> 0.25) IS the ETL
--   High-speed component: blade slap builds with speed
etl_factor = max(0, (vrs - 0.25) / 0.25)    # 1.0 at hover, 0.0 at 60+ kt
Vib2Rev = etl_gain * etl_factor + slap_gain * slap_rat

-- 3/rev: retreating blade stall onset
--   blade_alpha approaching stall values
Vib3Rev = rbs_gain * max(0, blade_alpha - rbs_threshold)

-- N/rev: blade-passing, relatively constant when rotor is turning
VibNRev = base_nrev * rpm_norm

-- 2N/rev: 2nd blade-passing, low-level
Vib2NRev = base_2nrev * rpm_norm
```

Exposed as Param nodes for the vehicle tab UI:

```
slot 1 (ratio 1.0)   gain_1rev
slot 2 (ratio 2.0)   etl_gain | slap_gain
slot 3 (ratio 3.0)   rbs_gain | rbs_threshold
slot 4 (ratio N)     base
slot 5 (ratio 2N)    base
```

For pedals with tail rotor vibration, the graph sets different ratios:

```
slot 1 (ratio 1.0)   main rotor 1/rev (weak on pedals)
slot 2 (ratio 2.0)   main rotor 2/rev
slot 3 (ratio 4.62)  tail rotor blade-passing (2-blade TR at 4.62:1 gear)
slot 4 (ratio 9.24)  tail rotor 2nd harmonic
```

All envelope drivers come from live X-Plane datarefs — no synthetic
IAS-based approximations needed. Harmonic ratios are per-aircraft config,
set once on profile load.


---

## 6. Per-aircraft tuning

### MD500E (reference implementation)

Static config: `rotation_sign = +1` (CCW, US)

Cyclic ratios: `[1.0, 2.0, 3.0, 5.0, 10.0]`
Pedal ratios: `[1.0, 2.0, 4.62, 9.24]` (tail rotor gear ratio 4.62:1, 2-blade TR)

| Slot | Ratio | Freq at 100% RPM | Dataref driver | Physical cue |
| --- | --- | --- | --- | --- |
| 1 | 1.0 | 8.2 Hz | `cyclic_elev_blad_alph` | Disc-loading "alive" feel |
| 2 | 2.0 | 16.4 Hz | `vortex_ring_state` + `blade_slap_rat` | ETL + high-speed |
| 3 | 3.0 | 24.6 Hz | `rotor_blade_alpha_deg` | Retreating blade stall |
| 4 | 5.0 (N) | 41.0 Hz | RPM-proportional | Blade-passing "whirr" |
| 5 | 10.0 (2N) | 82.0 Hz | RPM-proportional | 2nd blade-passing |

Pedal-specific slots:

| Slot | Ratio | Freq at 100% RPM | Physical cue |
| --- | --- | --- | --- |
| 3 | 4.62 | 37.9 Hz | Tail rotor blade-passing (2-blade × 4.62:1) |
| 4 | 9.24 | 75.8 Hz | Tail rotor 2nd harmonic |

Observed dataref ranges (MD 500E, from dataref logger flight):

| IAS | blade_alph_pitch | vrs_0 | slap_rat | blade_alpha |
| --- | --- | --- | --- | --- |
| hover | -0.2 deg | 0.500 | 0.000 | 2.9 deg |
| 20 kt | -1.2 deg | 0.470 | 0.005 | 2.7 deg |
| 40 kt | -1.1 deg | 0.359 | 0.016 | 2.6 deg |
| 80 kt | -1.7 deg | 0.250 | 0.024 | 2.6 deg |
| 120 kt | -4.3 deg | 0.250 | 0.039 | 3.0 deg |
| 160 kt | -5.6 deg | 0.250 | 0.037 | 2.8 deg |
| 170 kt | -6.2 deg | 0.250 | 0.052 | 3.0 deg |

Expected force amplitudes (starting points for tuning):

| Harmonic | Hover | 80 kt | 160 kt |
| --- | --- | --- | --- |
| 1/rev | 0.1 N | 1.0 N | 2.0 N |
| 2/rev (ETL) | 0.5 N | 0.0 N | 0.0 N |
| 2/rev (slap) | 0.0 N | 0.3 N | 0.5 N |
| 5/rev (N) | 0.5 N | 0.5 N | 0.5 N |

### R22 (2-blade teetering reference)

Static config: `blade_count = 2`, `rotation_sign = +1`

Slots 2 and N both render 2/rev. Graph output strategy: populate `Vib2Rev`
only, leave `VibNRev` at zero.

The R22's teetering rotor produces pronounced 2/rev lateral vibration from
the see-saw flapping. `rotor_slap` should be the primary 2/rev driver.


---

## 7. Testing plan

### Unit / bench

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

### In-sim validation

1. **MD500E hover**: fundamental ~8 Hz, feel light 1/rev pulse (~0.3 N).
   `pitch_flap` ~-1.5 deg, `rotor_slap` = 0 → no 2/rev. Correct.
2. **MD500E ETL (15-35 kt)**: 2/rev appears from `swirl_maxkt` peaking
   at ~1.5 kt. Feel a distinct "thumping" overlaid on the 1/rev.
3. **MD500E 80 kt cruise**: 1/rev builds to ~1 N from `pitch_flap` -3.6 deg.
   2/rev light. 5/rev (blade-passing) constant background.
4. **MD500E 160 kt (near Vne)**: 1/rev strong (~2 N), 2/rev from
   `rotor_slap` building. `blade_alpha` approaching stall region.
5. **On ground, rotor turning**: all flapping signals ~0 → vibration
   naturally zero. Disc tilt tracks cyclic but doesn't create vibration.
6. **Autorotation**: RPM changes smoothly, fundamental tracks,
   amplitudes scale with RPM (no glitches).
7. **Rotor brake / shutdown**: amplitudes fade as RPM drops below ~30%;
   no clicks at the `fundamental_hz = 0` transition.

### Pilot sanity checks

* Can a real MD500 pilot identify "this feels like a 500" from vibration alone?
* Does the stick trace an ellipse in hover (1/rev quadrature), not a line?
* Does reversing `rotation_sign` feel noticeably wrong (ellipse goes wrong way)?


---

## 8. Implementation order

### Phase 1 -- core rendering

1. Add `f_vib` field to `SimAccumulators`, inject after damping/friction in `Sim::update()`
2. Migrate existing `Buffet` element to use `f_vib` instead of `f_sum`
3. `RotorVib` class on ESP32 (DDS, PLL, configurable ratio slots, amplitude smoothing → `f_vib`)
4. Wire two `RotorVib` instances into `FlightStickFunction` (DDS 1 + DDS 2)
5. Extend `FlightFfbAction` protobuf with DDS 1 + DDS 2 amplitude fields
6. Extend `FlightStickConfig` with `rotation_sign` and `vib_harmonic_ratios`
7. Bench test with fixed amplitudes and ratios (no graph integration yet)

### Phase 2 -- graph integration

1. Add rotor telemetry signals to X-Plane UDP packet and plugin
   (`BladeAlphPitch`, `BladeAlphRoll`, `Slap`, `VRS`, `BladeAlpha`)
2. Add graph outputs (`VibFundamental`, `Vib1Rev`, etc.) to signal catalog
3. Add `TryGetGraphVibOutputs()` and `SendFlightFfb()` plumbing
4. Create `heli_vibration.json` library block using X-Plane rotor signals
5. Wire into `heli_cyclic.json`

### Phase 3 -- phase sync

1. Add `send_rotor_sync_frame()` on gateway at 100 Hz (CAN ID `0x0F0`)
2. Gateway runs master DDS, packs `fundamental_hz` + `phase` into frame
3. Add `try_process_rotor_sync_frame()` on axes to extract DDS fields
4. Wire extracted frequency + phase into axis `RotorVib::on_sync()`
5. Test inter-axis PLL lock-in and steady-state coherence

### Phase 4 -- per-aircraft tuning

1. MD500E profile as reference
2. R22 profile for 2-blade validation
3. Document tuning procedure for community contributions

### Phase 5 -- refinements (deferred)

1. Collective-axis vibration (separate amplitude set)
2. Pedal-axis vibration from tail rotor (separate fundamental = TR RPM)
3. Per-harmonic phase control if specific aircraft need it
4. Sine LUT optimisation if CPU budget requires


---

## 9. Open questions

1. ~~**Gateway sync bandwidth**~~: RESOLVED. Dedicated frame at `0x0F0`,
   100 Hz, 8 bytes. Higher priority than FFB actions.
2. **Collective coupling**: collective-axis vibration has physically distinct
   amplitudes (vertical blade-passing dominates). Defer to phase 5 or
   include in phase 1?
3. ~~**Dataref availability**~~: RESOLVED. All required datarefs verified
   against `DataRefs.txt` and confirmed with live flight data logger:

   | Data export field | Dataref (verified) | Vibration role |
   | --- | --- | --- |
   | `pitch,_flap` | NOT exposed — use `cyclic_elev_blad_alph[N]` instead | 1/rev lateral |
   | `_roll,_flap` | NOT exposed — use `cyclic_ailn_blad_alph[N]` instead | 1/rev longitudinal |
   | `rotor,_slap` | `rotor_blade_slap_rat[N]` | 2/rev high-speed |
   | `swirl,maxkt` | NOT exposed — use `vortex_ring_state[N]` instead | 2/rev ETL |
   | `blade,alpha` | `rotor_blade_alpha_deg[N]` | 3/rev RBS |
   | `_disc,alpha` | `rotor_disc_alpha_deg[N]` | Context |

4. **82 Hz feasibility**: 2N/rev on a 5-blade rotor at nominal RPM is 82 Hz.
   The servo/mechanical system may attenuate forces above ~50 Hz
   significantly. Need bench test to determine useful upper frequency.


---

## 10. Load Force Strategy

This section covers `LoadForce` (constant force output) for helicopter axes.
While not vibration per se, it shares the same telemetry signals and is part
of the same graph templates.

### 10.1 Overview

| Heli type | Cyclic load force | Pedal load force |
| --- | --- | --- |
| MD 500E (unboosted) | Aero hinge moments reach pilot | Tail rotor blade loads reach pilot |
| Bell 206 (boosted) | Zero — irreversible | Zero — irreversible |
| H125 (boosted) | Zero — irreversible | Zero — irreversible |
| Bell 222 (boosted + SAS) | SAS-injected forces | Zero — irreversible |

### 10.2 Available datarefs vs requirements

**`L_aero`, `M_aero`, `N_aero`** (whole-airframe aerodynamic moments, Nm):
These are NOT control hinge moments. They represent the total aerodynamic
torque about the aircraft CG — dominated by wing/tail/rotor disc forces.
Observed range: M_aero = -8600 to +6764 Nm on the MD 500E. Far too large
and not directly proportional to stick force. Using them as load force
drivers requires extreme scaling (~0.005 N/Nm) and the relationship to
stick force varies with control geometry per aircraft.

**`blade_alph_pitch` / `blade_alph_roll`** (per-axis blade alpha, degrees):
Better proxy for hinge moments on unboosted rotors. Blade AoA directly
determines blade lift, which determines the flapping hinge moment the pilot
resists. Scales linearly with IAS (-0.2 hover to -6.2 at 170 kt). Already
used for 1/rev vibration — can double as the load force envelope.

**`torque_main`** (main rotor torque, Nm):
Drives collective/pedal coupling but not cyclic forces directly.

### 10.3 Cyclic load force

#### Data analysis: blade_alph_pitch vs g-load

Flight data analysis (MD 500E dataref logger) reveals that `blade_alph_pitch`
is primarily an **airspeed** signal, not a **g-load** signal:

```
Cruise (60-100 kt) — blade_alph_pitch by g-load:
  0.25g  →  -0.22   (pushing over — disc unloaded, nearly zero)
  0.75g  →  -1.51
  1.00g  →  -2.01   (normal 1g flight)
  1.25g  →  -0.92   (DECREASES — wrong direction for stick force)
  1.50g  →  -0.14   (nearly zero at 1.5g)

Fast (120-180 kt) — blade_alph_pitch by g-load:
  0.75g  →  -5.18
  1.00g  →  -4.92
  1.25g  →  -5.42   (slight increase — weak sensitivity)
  1.75g  →  -5.72   (marginal)
```

At cruise speeds, `blade_alph_pitch` DECREASES toward zero when pulling g.
Physically: pulling back tilts the disc aft, reducing the advancing blade's
encounter angle — flapping asymmetry decreases even though total thrust
increases. This makes `blade_alph_pitch` unsuitable as the sole load force
driver because it misses the manoeuvring force cue entirely.

`blade_alph_roll` shows no meaningful g-load sensitivity (flat at -0.1 to
-0.3 across all g values).

#### MD 500E (unboosted) — two-component model

Cyclic load force requires two separate components:

1. **Speed stability** (from `blade_alph_pitch/roll`): stick gets heavier
   with airspeed. Physically correct — advancing blade asymmetry increases
   hinge moments at higher IAS.

2. **Manoeuvre stability** (from `g_nrml`): stick gets heavier when pulling
   g. Must be added as a separate term since blade alpha doesn't capture
   this effect.

```
LoadForce_roll  = speed_gain * abs(blade_alph_pitch)   # IAS-dependent
                + g_gain * (g_nrml - 1.0)               # manoeuvre cue

LoadForce_pitch = speed_gain * abs(blade_alph_roll)    # weak IAS component
                + g_gain * (g_nrml - 1.0)               # manoeuvre cue
```

Note the axis swap: pitch flapping (advancing blade effect) produces lateral
disc tilt → lateral stick force, and vice versa. Same 90-degree precession.

For OWL (one-way lock, longitudinal axis only): clamp pitch `LoadForce` to
negative values only (resist aft creep, don't resist forward input):

```
LoadForce_pitch = Min(0, LoadForce_pitch)
```

Graph params:

```
Cyclic.LoadSpeedGain    — N per degree of blade alpha
Cyclic.LoadGGain        — N per g increment above 1.0
```

#### Bell 222 (SAS) — same two-component pattern

The SAS model already uses `g_nrml` and body rates. The data analysis
confirms this is the right approach — `blade_alph_pitch` would not provide
the manoeuvre cue that the SAS is specifically designed to inject.

```
LoadForce_pitch = sas_g_gain * (g_nrml - 1.0)     # manoeuvre cue
                + sas_q_gain * Q_rad_s              # pitch rate damping

LoadForce_roll  = sas_p_gain * P_rad_s              # roll rate damping
```

When SAS is disengaged: `LoadForce = 0` (reverts to pure spring model).
SAS engage/disengage would need a new input signal (future work).

#### Bell 206, H125 (boosted, irreversible)

`LoadForce = 0`. Pilot feels only spring + friction + trim system.

### 10.4 Pedal load force

#### All types — RPM-scaled spring

Rather than computing a tail rotor hinge moment (no good dataref), scale
the pedal `SpringGain` by normalized rotor RPM:

```
SpringGain = base_spring * rpm_norm
```

Physical rationale: tail rotor blade loads scale with RPM (more RPM = more
aerodynamic force per degree of pitch = stiffer pedals). This captures:

* **Normal flight**: full RPM → firm pedals
* **Autorotation**: RPM drops → pedals lighten (less TR authority)
* **Shutdown**: RPM → 0 → pedals free
* **Engine failure**: same as autorotation — immediate lightening

Works for both boosted and unboosted types — even hydraulically boosted
systems have an artificial spring that should feel lighter when the tail
rotor has less authority.

#### MD 500E pedal load force (optional refinement)

For enhanced realism on the unboosted MD 500E, add a small `LoadForce`
component proportional to main rotor torque (more torque = more anti-torque
pedal demand = more force to hold position):

```
LoadForce_pedal = pedal_load_gain * torque_norm
```

This is a secondary effect on top of the RPM-scaled spring. Defer to tuning
phase — the RPM-scaled spring alone may be sufficient.

### 10.5 Collective load force

The collective on most helicopters has a friction lock — the pilot adjusts
friction to hold the collective in position. Model as constant friction
with no speed-dependent load. For the unboosted MD 500E, collective
forces scale with RPM and blade pitch, but the dominant feel is the
friction lock. `SpringGain = 0`, `Friction = user-adjustable`.

RPM-scaled friction is a possible refinement:

```
Friction = base_friction * rpm_norm
```


---

## 11. Non-goals

* Buffet replacement. Band-limited noise stays for incoherent cues.
* Tail rotor vibration modelling (phase 5+).
* Acoustic output / cabin noise simulation.
* Non-sinusoidal waveshapes (triangle, saw). Stick to fundamentals
  + harmonics; any waveshape can be approximated with enough harmonics.
