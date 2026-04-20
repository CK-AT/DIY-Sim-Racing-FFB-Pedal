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

Slot semantics are determined by `vib_harmonic_ratios` in the config.
Typical mapping: slot 1 = 1/rev, slot 2 = 2/rev, etc., but the graph
can assign any ratio to any slot. For 2-blade rotors slots 2 and 4
render the same frequency (ratio 2.0) and their amplitudes sum.

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
    x = fmodf(x + (float)M_PI, 2.0f * (float)M_PI);
    if (x < 0.0f) x += 2.0f * (float)M_PI;
    return x - (float)M_PI;
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

## 5. SimHub graph integration

### New graph outputs

**Per flight function** (amplitudes only — frequencies come from gateway sync):

| Output | Proto field | DDS | Unit |
| --- | --- | --- | --- |
| `VibSlot1` | `vib_amp_slot1` | 1 | N |
| `VibSlot2` | `vib_amp_slot2` | 1 | N |
| `VibSlot3` | `vib_amp_slot3` | 1 | N |
| `VibSlot4` | `vib_amp_slot4` | 1 | N |
| `VibSlot5` | `vib_amp_slot5` | 1 | N |
| `Vib2Slot1` | `vib2_amp_slot1` | 2 | N |
| `Vib2Slot2` | `vib2_amp_slot2` | 2 | N |

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

## 5a. Vibration Slot Assignment Concept

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

### Slot assignment profiles

Five standard profiles cover all current aircraft types:

#### `heli_cyclic` — helicopter pitch/roll axes

DDS 1 fundamental: main rotor RPM / 60 (gateway-synced).
DDS 2 fundamental: engine RPM / 60 (free-running, no sync needed).

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Rotor 1/rev (disc tilt) | `blade_alph_pitch/roll` — **axis-split** |
| 1 | 2 | 2.0 | Rotor 2/rev (ETL + high-speed) | `vrs` + `slap_rat` |
| 1 | 3 | 3.0 | Rotor 3/rev (retreating blade stall) | `blade_alpha` threshold |
| 1 | 4 | N | Blade-passing N/rev | RPM-proportional constant |
| 1 | 5 | 2N | Blade-passing 2nd harmonic | RPM-proportional constant |
| 2 | 1 | 1.0 | Engine 1/rev | base + `torque_norm` |
| 2 | 2 | 2.0 | Engine 2/rev | base constant |

`rotation_sign = +1` (US/CCW) or `-1` (EU/CW).

Graph template: `heli_vibration.json` (cyclic variant).

#### `heli_pedal` — helicopter yaw axis

DDS 1 fundamental: main rotor RPM / 60 (gateway-synced).
DDS 2: disabled (fundamental = 0).

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Main rotor 1/rev (weak on pedals) | RPM-proportional |
| 1 | 2 | 2.0 | Main rotor 2/rev | `vrs` + `slap_rat` |
| 1 | 3 | gear | Tail rotor blade-passing | RPM-proportional |
| 1 | 4 | 2×gear | Tail rotor 2nd harmonic | RPM-proportional |
| 1 | 5 | — | Unused | amplitude = 0 |

Gear ratio is aircraft-specific (e.g., 4.62 for MD 500E). Tail rotor
vibration rides on the main rotor DDS — no second oscillator needed.

`rotation_sign = 0` (no axis split on pedals — single axis).

Graph template: `heli_vibration.json` (pedal variant, parameterized by
gear ratio).

#### `plane_single` — single-engine fixed-wing

DDS 1 fundamental: prop RPM / 60.
DDS 2: disabled (fundamental = 0).

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Engine 1/rev | base + `torque_norm` |
| 1 | 2 | 2.0 | Engine 2/rev | base constant |
| 1 | 3-5 | — | Unused | amplitude = 0 |

`rotation_sign = 0` (no axis split — engine vibration is isotropic).

Graph template: `plane_engine_vib.json`.

#### `plane_twin` — twin-engine fixed-wing

DDS 1 fundamental: prop 1 RPM / 60.
DDS 2 fundamental: prop 2 RPM / 60.

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Engine 1 — 1/rev | base + `torque_norm` eng 1 |
| 1 | 2 | 2.0 | Engine 1 — 2/rev | base constant |
| 1 | 3-5 | — | Unused | amplitude = 0 |
| 2 | 1 | 1.0 | Engine 2 — 1/rev | base + `torque_norm` eng 2 |
| 2 | 2 | 2.0 | Engine 2 — 2/rev | base constant |

`rotation_sign = 0`.

Beat frequency between engines emerges naturally from DDS interference —
no special logic. Prop sync → beat disappears. Engine failure → one DDS
goes silent.

Graph template: `plane_engine_vib.json` (twin variant).

#### `heli_cyclic_with_engine` — helicopter with engine vibration on DDS 2

Same as `heli_cyclic` above. The DDS 2 engine slots are optional — set
amplitudes to zero if engine vibration is not desired (e.g., turbine
helicopters where the engine is too smooth to feel).

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

Each `RotorVib` instance carries its own `rotation_sign`. DDS 2's instance
always uses `rotation_sign = 0` (engine vibration is never directional).

### Config coordination — ConfigOut node type

The consistency problem: `FlightStickConfig.vib_harmonic_ratios` (what
frequency each slot runs at) and the graph template (what signal drives
each slot's amplitude) must agree. If they diverge, vibration is
physically wrong — e.g., a "tail rotor blade-passing" amplitude driving
a slot whose ratio is set to "3/rev retreating blade stall."

**Solution: a new `ConfigOut` graph node type.** A terminal node (like
`Output`) that receives a graph-computed value and writes it to a
`FlightStickConfig` proto field. The graph computes everything — shared
Params like blade count flow through normal graph nodes (Mul, Const,
etc.) into ConfigOut terminals. When a ConfigOut value changes, the
plugin triggers a config upload to the ESP32.

Key properties:

* **Computed, not static** — ConfigOut receives its value from the graph,
  so derived config (e.g., `2 × blade_count` for the 2N/rev ratio) is
  computed by the graph, not by plugin code.
* **Change-triggered upload** — plugin compares ConfigOut values to the
  last-sent config each evaluation cycle. If any changed, it re-sends
  `FlightStickConfig`. In steady state (no Param edits), no uploads.
* **Shared Params work naturally** — `Aircraft.BladeCount` is a regular
  shared Param. Each function's sub-graph wires it through computation
  nodes to per-function ConfigOut terminals. No new sharing mechanism.

#### ConfigOut node and FunctionScope on Include

Adding ConfigOut with the existing generic-port-in-sub-graph +
function-scoped-in-parent pattern would require ~10 extra links per
function in the parent template (ratio ports + rotation_sign). The
parent already has ~14 links per cyclic include; tripling that is
untenable.

**Solution: two changes.**

1. **`FunctionScope` dropdown on Include nodes** — tells the runtime
   which function the sub-graph's scoped outputs target
2. **`ConfigOut` node type** — a new terminal (like Output) whose
   ports write to proto config fields instead of streaming outputs

When an Include has `FunctionScope` set, scoped Output and ConfigOut
nodes inside the sub-graph inherit it as their function group. The
parent only wires inputs — all output and config routing is implicit.

**Include node with FunctionScope:**

```json
{
  "Id": "cyclic_pitch_include",
  "Title": "Cyclic Pitch",
  "Kind": "Include",
  "IncludePath": "..\\_embedded\\heli_cyclic.json",
  "FunctionScope": "FlightStickPitch"
}
```

**Editor UI:** `FunctionScope` appears as a dropdown on the Include
node inspector (below IncludePath, above the port lists). Options:

```
(none)                      ← default, generic mode
FlightStickPitch
FlightStickRoll
FlightPedals
FlightStickCollective
```

The dropdown values come from `GraphSignalCatalogData`. `(none)` means
all outputs use the existing generic-port pattern (no scoping).

**Scoped Output node in sub-graph** (replaces generic output ports):

```json
{
  "Id": "out_ffb",
  "Title": "FFB Outputs",
  "Kind": "Output",
  "Ports": [
    { "Name": "spring", "Kind": "Input", "SignalSuffix": "SpringGain" },
    { "Name": "damper", "Kind": "Input", "SignalSuffix": "DamperGain" },
    { "Name": "friction", "Kind": "Input", "SignalSuffix": "Friction" },
    { "Name": "load", "Kind": "Input", "SignalSuffix": "LoadForce" },
    { "Name": "trim", "Kind": "Input", "SignalSuffix": "TrimOffset" }
  ]
}
```

An Output node is scoped when its ports have `SignalSuffix` AND the
Include that contains it has a `FunctionScope`. The runtime converter
registers `FlightStickPitch.SpringGain` etc. — identical to what the
parent's explicit Output node produced before.

Without `FunctionScope` on the Include, these ports are treated as
generic output ports (by `Name`), same as today. So an Output node
with `SignalSuffix` on its ports works in both modes.

**ConfigOut node in sub-graph:**

```json
{
  "Id": "cfg_out",
  "Title": "Vib Config",
  "Kind": "ConfigOut",
  "Ports": [
    { "Name": "rotation_sign", "Kind": "Input",
      "ConfigField": "rotation_sign" },
    { "Name": "ratio_0", "Kind": "Input",
      "ConfigField": "vib_harmonic_ratios[0]" },
    { "Name": "ratio_1", "Kind": "Input",
      "ConfigField": "vib_harmonic_ratios[1]" },
    { "Name": "ratio_2", "Kind": "Input",
      "ConfigField": "vib_harmonic_ratios[2]" },
    { "Name": "ratio_3", "Kind": "Input",
      "ConfigField": "vib_harmonic_ratios[3]" },
    { "Name": "ratio_4", "Kind": "Input",
      "ConfigField": "vib_harmonic_ratios[4]" }
  ]
}
```

ConfigOut always requires `FunctionScope` on its Include — without it,
ConfigOut nodes are ignored (no function → don't know which config to
write to). The editor can warn when a sub-graph with ConfigOut nodes is
included without a FunctionScope.

**SignalGroup usage rules:**

| Node type | SignalGroup | Constraint |
| --- | --- | --- |
| Param | freeform string | Any group name (e.g., `Aircraft`, `Vib`) |
| Input (unscoped) | freeform string | Any group (e.g., `XPlane`, `Grip`) |
| Output (unscoped) | function dropdown | Must be a valid function group |
| Output (scoped, in sub-graph) | not set | Inherited from Include's `FunctionScope` |
| ConfigOut (in sub-graph) | not set | Inherited from Include's `FunctionScope` |
| Include | `FunctionScope` dropdown | `(none)` or valid function group |

Unscoped Output nodes in the parent template still use the existing
`SignalGroup` dropdown (shown as the function group selector) — this
is unchanged. Scoped Output/ConfigOut nodes inside sub-graphs have no
`SignalGroup` of their own; they inherit from their Include site.

**Runtime conversion:**

`GraphRuntimeConverter` already builds Output nodes with
`BuildFullSignalName(node.SignalGroup, port.SignalSuffix, port.Name)`.
The change: when recursing into an included graph, if the Include node
has `FunctionScope`, pass it as the `SignalGroup` for any Output node
that has `SignalSuffix` on its ports, and for any ConfigOut node.

```
Existing (unscoped):
  sub-graph Output port "spring" → Include output port → parent link →
  parent Output(SignalGroup="FlightStickPitch", SignalSuffix="SpringGain")
  → runtime: FlightStickPitch.SpringGain

Scoped:
  sub-graph Output(port SignalSuffix="SpringGain") →
  Include(FunctionScope="FlightStickPitch") →
  runtime: FlightStickPitch.SpringGain

Same runtime result, no parent Output node, no link.
```

**Backward compatibility:** `FunctionScope` defaults to `(none)`.
Existing sub-graphs with generic ports continue to work unchanged.
Both modes can coexist in the same parent template.

#### Sharing across functions

Aircraft-level values like blade count are regular `Aircraft.*` Params,
shared across all functions by the existing Param system. They flow
into each sub-graph include as Input wiring (this already exists — no
new links needed). Inside the sub-graph, computation nodes derive
ratios and wire them to the scoped ConfigOut terminal:

```
heli_default.json (parent template)

  Aircraft.BladeCount   [Param = 5]     ← shared, one slider
  Aircraft.RotationSign [Param = 1]     ← shared
  Aircraft.TRGearRatio  [Param = 4.62]  ← shared

  cyclic_pitch_include
    Kind: Include, FunctionScope: "FlightStickPitch"
    Inputs wired: torque, rpm, aero_trq, blade_count, rotation_sign
    (no output links — scoped nodes handle it)

  cyclic_roll_include
    Kind: Include, FunctionScope: "FlightStickRoll"
    Inputs wired: torque, rpm, aero_trq, blade_count, rotation_sign

  pedals_include
    Kind: Include, FunctionScope: "FlightPedals"
    Inputs wired: torque, rpm, aero_trq, tr_gear_ratio
```

Same `Aircraft.BladeCount` Param feeds both cyclic includes. Changing
blade count → graph re-evaluates → scoped ConfigOut values change for
pitch and roll → plugin detects → config upload for both functions.

The parent template only has input links (already needed for telemetry
signals) — zero output or config links. Compare:

| | Before (generic) | After (scoped) |
| --- | --- | --- |
| Links per cyclic include | ~14 (7 in + 7 out) | ~7 (7 in, 0 out) |
| ConfigOut links per include | ~10 (new) | 0 |
| Parent Output nodes | 4 (one per function) | 0 |
| Parent ConfigOut nodes | 4 | 0 |

#### Sub-graph structure

```
heli_vibration_cyclic.json (sub-graph, IsLibraryGraph: true)

  Inputs:                        (wired from parent)
    blade_count                    ← Aircraft.BladeCount
    rotation_sign                  ← Aircraft.RotationSign
    blade_alph_pitch, slap_rat...  ← telemetry signals

  Scoped ConfigOut:              → FunctionScope's FlightStickConfig
    rotation_sign     → ConfigField "rotation_sign"
    Const(1.0)        → ConfigField "vib_harmonic_ratios[0]"
    Const(2.0)        → ConfigField "vib_harmonic_ratios[1]"
    Const(3.0)        → ConfigField "vib_harmonic_ratios[2]"
    blade_count       → ConfigField "vib_harmonic_ratios[3]"
    blade_count × 2   → ConfigField "vib_harmonic_ratios[4]"

  Params (regular):              → graph evaluation
    Vib.Gain1Rev        = 0.5
    Vib.ETLGain         = 1.0

  Internal logic:                (amplitude envelopes)
    slot 1 amplitude ← blade_alph * Gain1Rev
    slot 2 amplitude ← vrs + slap_rat * ETLGain
    ...

  Scoped Output:                 → FunctionScope's FlightFfbAction
    SpringGain, DamperGain, Friction, LoadForce, TrimOffset,
    VibSlot1..5, Vib2Slot1..2
```

Ratios and amplitude wiring live in the same sub-graph — they cannot
diverge. The sub-graph knows that slot 4 is "N/rev blade-passing" and
both computes its ratio from `blade_count` AND drives its amplitude
from the RPM-proportional envelope.

#### Per-aircraft tuning

Tuning happens through the shared Params in the parent template:

* **R22** (2-blade): profile overrides `Aircraft.BladeCount = 2`.
  Graph computes: ratio[3] = 2, ratio[4] = 4. ConfigOut detects
  change → uploads new config. Amplitude envelopes unchanged (still
  "blade-passing" source, just at different frequency).
* **MD 500E**: default `Aircraft.BladeCount = 5`, no override needed.
* **Gear ratio**: profile overrides `Aircraft.TRGearRatio = 4.62`.
  Pedal sub-graph computes: ratio[2] = 4.62, ratio[3] = 9.24.

**Key rule: ratios that change the physical source need a different
sub-graph.** Changing blade count (same source, different frequency)
is safe via Param override. Changing a slot from "retreating blade
stall" to "tail rotor" requires the pedal sub-graph variant where the
internal amplitude wiring matches.

In practice this means a small number of sub-graph variants:

| Sub-graph variant | Slot semantics |
| --- | --- |
| `heli_vibration_cyclic` | 1/rev disc tilt, 2/rev ETL, 3/rev RBS, N/rev, 2N/rev |
| `heli_vibration_pedal` | 1/rev rotor, 2/rev rotor, TR blade-pass, TR 2nd harmonic |
| `plane_engine_vib` | Engine 1/rev, engine 2/rev (per DDS) |

#### Plugin implementation

**Runtime converter changes:** When processing an Include node with
`FunctionScope`, recurse into the included graph. For each scoped
Output node, register it as `NodeType.Output` with name =
`BuildFullSignalName(functionScope, port.SignalSuffix, port.Name)`.
For each scoped ConfigOut node, register it as `NodeType.ConfigOut`
(new enum value) with the function scope and ConfigField.

This mirrors the existing Output registration at
[GraphRuntimeConverter.cs:100-117](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L100-L117)
— same `BuildFullSignalName`, just with the scope inherited from the
Include node instead of from the Output node's own `SignalGroup`.

**Config change detection:** after each graph evaluation cycle (20 Hz),
the plugin reads all ConfigOut values per function and compares to the
last-sent config. Upload only on change:

```csharp
void CheckConfigOutChanges()
{
    foreach (var functionId in flightFunctionIds)
    {
        string prefix = GetGraphFunctionPrefix(functionId);
        if (prefix == null) continue;

        var config = currentConfigs[functionId];
        bool changed = false;

        // ConfigOut nodes are registered with names like
        // "FlightStickPitch.cfg.vib_harmonic_ratios[3]"
        foreach (var (field, value) in GetConfigOutputs(prefix))
        {
            changed |= SetConfigField(config, field, value);
        }

        if (changed)
        {
            EnqueueConfigUpload(functionId, config);
        }
    }
}
```

In steady state (no Param edits), ConfigOut values are constant and
no uploads occur — this is a cheap comparison, not a real upload.

**Startup**: on profile load, ConfigOut values are evaluated and
uploaded as part of the initial config send. No special path needed.

**User edits Param slider**: next graph evaluation cycle picks up the
new Param value → flows through computation nodes → ConfigOut value
changes → plugin detects → config upload. Latency: one eval cycle
(50 ms at 20 Hz). Acceptable for static config changes.

#### Migrating existing graphs to scoped mode

Scoped Output is optional — existing graphs with generic Output nodes
and explicit parent wiring continue to work. But scoped mode can
simplify existing templates too. Migration per sub-graph:

1. Add `"Scoped": true` and `SignalSuffix` to Output node ports
2. Add `"FunctionScope"` to Include nodes in parent template
3. Remove Output nodes and output links from parent template

This is backward-compatible: new sub-graphs default to scoped,
existing ones stay generic until migrated. Both modes can coexist
in the same parent template (some includes scoped, others generic).

#### Safe defaults

* Graph without ConfigOut nodes → no config fields modified →
  existing graphs completely unaffected.
* ConfigOut with computed value 0 for ratios → DDS slot disabled
  (zero ratio = skip). Safe fallback.
* Disconnected ConfigOut input port → value = 0 → safe default.

#### Generality beyond vibration

ConfigOut is not vibration-specific. Any static config field that
should be graph-computed can use it. Future candidates:

* `FlightStickConfig.damping` — per-aircraft damping from graph
* `FlightStickConfig.centering_spring_const` — same

For now, only vibration config uses ConfigOut. Existing fields
(`pos_min`, `pos_max`, `damping`, `centering_spring_const`) continue
to be set from the function config UI as before.

**Custom profiles**: the slot system is fully generic. A community
contributor can create a custom sub-graph with non-standard slot
semantics (e.g., coaxial helicopter with contra-rotating rotors on
DDS 1 and DDS 2) — the ConfigOut wiring and amplitude computation
simply reflect the new physical model.


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
2. Independent tail rotor DDS for aircraft where gear ratio approximation
   is insufficient (main plan handles tail rotor via harmonic ratio slots)
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
