# Fixed-Wing Load Force Plan

Force model for fixed-wing aircraft `LoadForce` outputs. All computation
happens in the graph system using existing telemetry signals — no new
datarefs, no ESP32 changes.


---

## 1. Motivation

The current plane graphs (`plane_pitch.json`, `plane_roll.json`,
`plane_yaw.json`) produce `SpringGain`, `DamperGain`, `Friction`, and
`BuffetAmplitude` — but `LoadForce` is always zero. This means the stick
feels the same whether flying straight-and-level or pulling 3g in a turn.

Real aircraft have control surface hinge moments that the pilot feels
through the stick/yoke. The dominant effects:

* **Pitch**: stick force per g (manoeuvre stability) — the most important
  feel cue. FAR 23/25 require a minimum force gradient.
* **Roll**: ailerons get heavier at speed (hinge moment scales with q).
* **Yaw**: rudder hinge moment scales with q; sideslip creates a pedal
  force cue ("ball not centered").


---

## 2. Design Constraints

### No stick-position feedback

Using `yoke_pitch * qhat` as a load force creates a feedback loop: stick
position depends on total force, which includes load force, which depends
on stick position. This can cause oscillation or instability in the
admittance-control force loop.

**Rule**: load force must depend only on **flight state signals** (rates,
g-load, angles, airspeed), never on stick position.

### Separation from spring and damper

* `SpringGain` provides speed-dependent centering (already qhat-scaled).
* `DamperGain` provides rate-proportional resistance.
* `LoadForce` adds a **constant offset** based on flight state — it's the
  force you must hold against, independent of displacement from trim.

The key distinction: spring force is zero at trim center, load force is
NOT. A 2g pull requires holding back against `LoadForce` even when the
stick is at the trimmed position.


---

## 3. Available Signals

All already in the X-Plane UDP packet and exposed as graph inputs:

| Signal | Unit | Pitch | Roll | Yaw |
| --- | --- | --- | --- | --- |
| `XPlane.G_Nrml` | g | Primary driver | — | — |
| `XPlane.Rate.Pitch` (Q) | rad/s | Rate damping | — | — |
| `XPlane.Rate.Roll` (P) | rad/s | — | Rate resistance | — |
| `XPlane.Rate.Yaw` (R) | rad/s | — | — | Rate resistance |
| `XPlane.Angle.Beta` | deg | — | — | Sideslip force cue |
| `XPlane.Angle.Alpha` | deg | Stall warning | — | — |
| `XPlane.Speed.IAS` | kts | Via qhat_eff | Via qhat_eff | Via qhat_eff |


---

## 4. Per-Axis Load Force Model

### 4.1 Pitch — manoeuvre stability + rate damping

The pitch axis `LoadForce` provides two cues:

1. **G-load force gradient**: pulling g requires proportionally more force.
   This is the primary "feel" difference between straight flight and turns.

2. **Pitch rate opposition**: resists uncommanded pitch rates. Supplements
   the `DamperGain` with a constant force component (damper is velocity-
   proportional, this adds a step force when pitching).

```
LoadForce_pitch = g_gain * (g_nrml - 1.0)
                + q_gain * Q_rad_s
```

| Param | Default | Range | Unit | Description |
| --- | --- | --- | --- | --- |
| `FlightStickPitch.LoadGGain` | 5.0 | 0 - 30 | N/g | Force per g increment |
| `FlightStickPitch.LoadRateGain` | 1.0 | 0 - 10 | N/(rad/s) | Force per pitch rate |

**Expected feel:**

| Manoeuvre | g_nrml | LoadForce |
| --- | --- | --- |
| Straight and level | 1.0 | 0 N |
| 30 deg bank (1.15g) | 1.15 | 0.75 N |
| 45 deg bank (1.41g) | 1.41 | 2.1 N |
| 60 deg bank (2.0g) | 2.0 | 5.0 N |
| 2.5g pull-up | 2.5 | 7.5 N |
| Pushover (0.5g) | 0.5 | -2.5 N (forward push) |

The sign convention: positive `LoadForce` pushes the stick forward (the
pilot must pull back to hold position in a turn). This matches the real
aircraft: tail downforce increases with g, requiring aft stick force.

### 4.2 Roll — rate resistance

Roll `LoadForce` opposes sustained roll rates. This supplements the damper
with a feel cue that the pilot must actively "push through" to maintain a
roll. Provides resistance proportional to roll rate, scaled by dynamic
pressure (heavier ailerons at higher speed).

```
LoadForce_roll = p_gain * P_rad_s * qhat
```

| Param | Default | Range | Unit | Description |
| --- | --- | --- | --- | --- |
| `FlightStickRoll.LoadRateGain` | 0.5 | 0 - 5 | N/(rad/s) | Force per roll rate at Vref |

At low speed (`qhat` < 1), roll forces are light. At high speed, they
increase — matching the heavier aileron feel of real aircraft.

Note: for most GA aircraft, the spring + damper already provide adequate
roll feel. `LoadForce` on roll is a refinement, not essential. Start with
zero and tune up if the roll axis feels too "free" in manoeuvres.

### 4.3 Yaw — sideslip cue + rate resistance

Pedal `LoadForce` provides two cues:

1. **Sideslip force**: rudder force proportional to beta (sideslip angle).
   When the ball is not centered, the pilot feels a pedal force pushing
   toward coordinated flight. This is the most useful pedal force cue.

2. **Yaw rate resistance**: opposes sustained yaw rates.

```
LoadForce_yaw = beta_gain * beta_deg
              + r_gain * R_rad_s
```

| Param | Default | Range | Unit | Description |
| --- | --- | --- | --- | --- |
| `FlightPedals.LoadBetaGain` | 0.3 | 0 - 2 | N/deg | Force per degree sideslip |
| `FlightPedals.LoadRateGain` | 0.5 | 0 - 5 | N/(rad/s) | Force per yaw rate |

**Expected feel:**

| Condition | Beta | LoadForce |
| --- | --- | --- |
| Coordinated flight | 0 deg | 0 N |
| Slight slip (3 deg) | 3 deg | 0.9 N |
| Significant slip (10 deg) | 10 deg | 3.0 N |
| Engine-out (15+ deg) | 15 deg | 4.5 N |


---

## 5. Graph Implementation

### New sub-graph: `plane_load.json`

Optional Include block for fixed-wing load forces. Keeps the existing
plane_pitch/roll/yaw sub-graphs unchanged — load force is wired at the
template level.

Inputs:

* `g_nrml` (from `XPlane.G_Nrml`)
* `q_rate` (from `XPlane.Rate.Pitch`)
* `p_rate` (from `XPlane.Rate.Roll`)
* `r_rate` (from `XPlane.Rate.Yaw`)
* `beta` (from `XPlane.Angle.Beta`)
* `qhat` (from `qhat_eff` node in parent graph)

Outputs:

* `load_pitch` → wired to `FlightStickPitch.LoadForce` output
* `load_roll` → wired to `FlightStickRoll.LoadForce` output
* `load_yaw` → wired to `FlightPedals.LoadForce` output

All params as inputs (not embedded Param nodes) so the parent graph
controls naming per function group.

### Wiring in `plane_default.json`

```
                    g_nrml ──┐
XPlane inputs ──►   q_rate  ─┤
                    p_rate  ─┼──► plane_load ──► LoadForce outputs
                    r_rate  ─┤
                    beta   ──┤
qhat_eff node ──►   qhat   ──┘
```

The load force outputs are added to the existing Output nodes alongside
SpringGain, DamperGain, Friction, BuffetAmplitude.

### Alternative: inline in axis sub-graphs

Instead of a separate sub-graph, each axis sub-graph (plane_pitch.json
etc.) could compute its own load force internally. Simpler wiring but
less reusable. Either approach works — graph-level choice.


---

## 6. Per-Aircraft Tuning

### Light GA (Cessna 172, PA-28)

Light, cable-controlled aircraft with relatively direct control feel.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 3.0 N/g | Light gradient, low max forces |
| Pitch LoadRateGain | 0.5 N/(rad/s) | Mild pitch rate cue |
| Roll LoadRateGain | 0.0 | Not needed — spring/damper sufficient |
| Yaw LoadBetaGain | 0.2 N/deg | Light rudder forces |

### Heavy GA (Bonanza, Baron, TBM)

Heavier controls, more pronounced force gradients.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 8.0 N/g | Firmer gradient |
| Pitch LoadRateGain | 1.5 N/(rad/s) | More authority feel |
| Roll LoadRateGain | 0.3 N/(rad/s) | Slight aileron heaviness |
| Yaw LoadBetaGain | 0.5 N/deg | Heavier rudder |

### Turboprop / Light jet

Higher speeds, boosted or mass-balanced controls.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 12.0 N/g | Strong gradient for higher-g capability |
| Pitch LoadRateGain | 2.0 N/(rad/s) | Crisp pitch response |
| Roll LoadRateGain | 0.5 N/(rad/s) | Roll stiffens with speed via qhat |
| Yaw LoadBetaGain | 0.8 N/deg | Firm rudder |

### Warbird / aerobatic

High g-capability, direct controls, need clear manoeuvre cues.

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 15.0 N/g | Very clear g cue for aerobatics |
| Pitch LoadRateGain | 2.0 N/(rad/s) | Crisp |
| Roll LoadRateGain | 1.0 N/(rad/s) | Aileron authority feel |
| Yaw LoadBetaGain | 0.5 N/deg | |

All values are starting points — tune by feel in-sim.


---

## 7. Vibration Sources

Fixed-wing aircraft have several distinct vibration sources that can be
rendered through the existing `Buffet` element and the planned `RotorVib`
DDS oscillator from the helicopter vibration plan.

### 7.1 Engine vibration (DDS — periodic)

The most important "alive" cue for propeller aircraft. Constant vibration
at engine RPM, always present while the engine runs.

**Rendering**: uses the `RotorVib` DDS oscillator. For fixed-wing, only
the 1/rev (engine fundamental) and 2/rev harmonics are relevant — no
blade-count-dependent harmonics.

```
fundamental_hz = prop_rpm / 60
torque_norm = torque / torque_ref
Vib1Rev = base_amp + torque_gain * torque_norm
Vib2Rev = base_amp_2   (typically smaller)
```

* `prop_rpm` — already in UDP packet (`prop_speed_rpm[0]`)
* `torque` — already in UDP packet (`POINT_drag_TRQ[0]`)
* Amplitude modulated by torque (more mechanical load = harder combustion
  pulses = more vibration). Torque is better than throttle position because
  it represents actual engine stress, not just lever position.
* Engine type determines character:
  - Flat-4 (Lycoming O-320): smooth, low base amplitude
  - Flat-6 (Lycoming IO-540): smoother still
  - Radial (R-1830 etc.): pronounced, high 1/rev
  - Rotax 912: rougher, higher base with more harmonics

**Engine failure cue**: engine stops → `fundamental_hz` drops to zero →
vibration fades via amplitude smoothing → immediate tactile "engine died"
sensation. Very valuable for training.

### 7.2 Multi-engine beat frequency (dual DDS)

Twin-engine aircraft with slightly mismatched RPMs produce a distinctive
beat (amplitude modulation) at `|RPM1 - RPM2| / 60` Hz. This is the
"wah-wah" that tells pilots their props are out of sync.

**Rendering**: two independent DDS oscillators, one per engine. The beat
emerges naturally from the interference — no special logic needed.

```
DDS 1: fundamental_hz = prop_rpm[0] / 60,  amplitude from engine 1 state
DDS 2: fundamental_hz = prop_rpm[1] / 60,  amplitude from engine 2 state
```

When RPMs match (synced props): steady vibration, no beat.
When RPMs differ by 2 Hz: slow 2 Hz amplitude modulation — realistic.
Engine failure on one side: that DDS goes to zero, vibration becomes
asymmetric (felt more on one axis than the other if per-axis amplitude
differs).

**Architecture impact**: the helicopter vibration plan defines one
`RotorVib` element per axis. For twins we need **two** `RotorVib`
instances per axis — each with its own `fundamental_hz` and amplitudes,
both accumulating into `f_vib`. The ESP32 `FlightStickFunction` adds
both as separate elements:

```cpp
RotorVib engine_vib_1;   // left engine
RotorVib engine_vib_2;   // right engine (or second DDS slot)
```

The graph produces two sets of vibration outputs:

```
Vib1Fundamental   — engine 1 RPM / 60
Vib1Amp1Rev       — engine 1 amplitude
Vib2Fundamental   — engine 2 RPM / 60
Vib2Amp1Rev       — engine 2 amplitude
```

For single-engine aircraft: `Vib2Fundamental = 0` (second DDS disabled).

For helicopters: DDS 1 = main rotor, DDS 2 = tail rotor (future, currently
out of scope). Or DDS 1 = rotor harmonics (via gateway sync), DDS 2 =
engine vibration (free-running, no sync needed).

**Gateway sync**: engine DDS does NOT need phase sync across axes — there's
no physical phase relationship between engine vibration on pitch vs roll
axes. Each axis free-runs its engine DDS independently. Only the rotor
DDS needs gateway PLL sync (for the 1/rev ellipse). This simplifies the
twin-engine implementation significantly.

### 7.3 Stall buffet (existing — incoherent noise)

Already implemented via the `buffet` graph func node. Band-limited random
noise gated by alpha approaching stall.

```
BuffetAmplitude = buffet(alpha, alpha_start, alpha_full, gain, qhat)
```

No changes needed. Rendered by the existing `Buffet` element (`f_vib`
after the damping/friction bypass is implemented).

### 7.4 Ground rumble (incoherent noise)

Vibration during taxi, takeoff roll, and landing rollout. Random,
speed-dependent, not periodic.

**Rendering**: use the existing `Buffet` element with amplitude gated by
`OnGround` and scaled by groundspeed.

```
BuffetAmplitude = ground_gain * groundspeed * OnGround
```

* `XPlane.OnGround` — already in UDP packet (0/1)
* Groundspeed — already available (`gs_mps`)
* Incoherent noise is correct for runway vibration (random bumps, not
  periodic)

Graph implementation: `Mul` nodes gating buffet amplitude. Could be
combined with stall buffet via `Max` — whichever source is larger wins:

```
BuffetAmplitude = Max(stall_buffet, ground_rumble)
```

### 7.5 Speed / flutter onset (incoherent noise)

Near Vne, control surfaces may start to buzz from aerodynamic excitation.
Similar to stall buffet but driven by airspeed instead of alpha.

```
flutter_buffet = buffet(IAS, vne_onset_kts, vne_full_kts, flutter_gain, 1.0)
```

Uses the same `buffet` func node with speed thresholds instead of alpha
thresholds. Combined with stall buffet via `Max` or `Add`.

### 7.6 Summary — rendering method per source

| Source | Element | Periodic? | Sync needed? | Signals |
| --- | --- | --- | --- | --- |
| Engine vibration | RotorVib DDS 1 | Yes (1/rev, 2/rev) | No | prop_rpm[0] |
| Second engine | RotorVib DDS 2 | Yes (1/rev) | No | prop_rpm[1] |
| Stall buffet | Buffet (existing) | No (noise) | No | alpha, qhat |
| Ground rumble | Buffet (existing) | No (noise) | No | OnGround, gs |
| Flutter onset | Buffet (existing) | No (noise) | No | IAS |

### 7.7 Architecture: dual DDS per axis

The helicopter plan defines a single `RotorVib` per axis with 5 harmonics
and gateway PLL sync. For fixed-wing we need a second independent DDS.
The cleanest approach: `FlightStickFunction` holds two `RotorVib` instances.

| Slot | Helicopter use | Fixed-wing use |
| --- | --- | --- |
| DDS 1 | Main rotor (5 harmonics, PLL-synced) | Engine 1 (1-2 harmonics, free-run) |
| DDS 2 | Future: tail rotor or engine | Engine 2 (1-2 harmonics, free-run) |

Each DDS receives its own `fundamental_hz` and amplitude set via
`FlightFfbAction`. The gateway sync frame drives DDS 1 only (helicopters);
DDS 2 always free-runs.

Protocol: both DDS frequencies and phases are carried in the gateway
`0x0F0` sync frame (see helicopter vibration plan section 3). Per-axis
`FlightFfbAction` carries amplitudes only:

```protobuf
message FlightFfbAction {
  // ... existing fields 1-6 ...
  // DDS 1 amplitudes (rotor or engine 1)
  float vib_amp_1rev = 7;
  float vib_amp_2rev = 8;
  float vib_amp_3rev = 9;
  float vib_amp_nrev = 10;
  float vib_amp_2nrev = 11;
  // DDS 2 amplitudes (engine 2)
  float vib2_amp_1rev = 12;
  float vib2_amp_2rev = 13;
}
```

Gateway sync frame (`0x0F0`, 100 Hz, 8 bytes):

```
Byte 0-1: dds1_fundamental_hz   uint16  0.001 Hz/LSB
Byte 2-3: dds1_phase            uint16  0..65535 → 0..2π
Byte 4-5: dds2_fundamental_hz   uint16  0.001 Hz/LSB
Byte 6-7: dds2_phase            uint16  0..65535 → 0..2π
```

Single-engine: DDS 1 = engine, DDS 2 freq = 0 (disabled).
Twin-engine: DDS 1 = engine 1, DDS 2 = engine 2. Beat frequency
emerges naturally from the RPM difference.


---

## 8. Per-Aircraft Tuning (load + vibration)

### Light GA single (Cessna 172, PA-28)

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 3.0 N/g | Light gradient |
| Pitch LoadRateGain | 0.5 N/(rad/s) | Mild pitch rate cue |
| Roll LoadRateGain | 0.0 | Spring/damper sufficient |
| Yaw LoadBetaGain | 0.2 N/deg | Light rudder |
| Engine Vib1Rev base | 0.3 N | Lycoming O-320, smooth 4-cyl |
| Engine Vib2Rev base | 0.1 N | Subtle 2nd harmonic |
| Ground rumble gain | 0.5 N | Light airframe |
| Stall buffet gain | existing | Already tuned per alpha |

### Light GA twin (Baron 58, PA-34)

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 6.0 N/g | Heavier than single |
| Pitch LoadRateGain | 1.0 N/(rad/s) | |
| Roll LoadRateGain | 0.3 N/(rad/s) | |
| Yaw LoadBetaGain | 0.4 N/deg | |
| Engine 1 Vib1Rev base | 0.3 N | Left engine |
| Engine 2 Vib1Rev base | 0.3 N | Right engine |
| Beat frequency | natural | DDS interference at RPM delta |

Twin-engine beat: with props synced (0 RPM delta), steady vibration.
Props 2 RPM apart → 2 Hz beat. Prop sync switch → beat disappears.
Engine failure → one DDS goes silent → asymmetric vibration + no beat.

### Turboprop (TBM 930, King Air)

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 12.0 N/g | Strong gradient |
| Pitch LoadRateGain | 2.0 N/(rad/s) | Crisp pitch |
| Roll LoadRateGain | 0.5 N/(rad/s) | Scales with qhat |
| Yaw LoadBetaGain | 0.8 N/deg | Firm rudder |
| Engine Vib1Rev base | 0.2 N | Turbine is smoother than piston |
| Ground rumble gain | 0.8 N | Heavier airframe |

### Warbird / aerobatic (Extra 300, P-51)

| Param | Value | Notes |
| --- | --- | --- |
| Pitch LoadGGain | 15.0 N/g | Clear g cue for aerobatics |
| Pitch LoadRateGain | 2.0 N/(rad/s) | Crisp |
| Roll LoadRateGain | 1.0 N/(rad/s) | Aileron authority feel |
| Yaw LoadBetaGain | 0.5 N/deg | |
| Engine Vib1Rev base | 0.8 N | Big radial = lots of vibration |
| Engine Vib2Rev base | 0.3 N | Pronounced 2nd harmonic |
| Ground rumble gain | 1.0 N | Taildragger = rough taxi |

All values are starting points — tune by feel in-sim.


---

## 9. Implementation Steps

### Phase 1 — load forces (graph only)

1. Create `plane_load.json` sub-graph (Mul, Sub, Add nodes — no new Funcs)
2. Wire `LoadForce` outputs in `plane_default.json`
3. Test with Cessna 172 — verify g-load cue in turns
4. Test with Baron 58 — verify pedal beta cue
5. Tune default params for GA categories

### Phase 2 — engine vibration (requires ESP32 RotorVib from heli plan)

1. Wire `VibFundamental = prop_rpm / 60` in plane graphs
2. Wire `Vib1Rev` amplitude from base + throttle scaling
3. Test single-engine — verify "alive" feel, engine failure cue
4. Tune per engine type (Lycoming vs Rotax vs radial)

### Phase 3 — dual DDS for twins

1. Extend `FlightStickFunction` with second `RotorVib` instance
2. Extend `FlightFfbAction` with DDS 2 fields
3. Add graph outputs for engine 2 vibration
4. Wire in twin-engine template
5. Test prop sync — verify beat frequency appears/disappears
6. Test engine failure — verify asymmetric vibration

### Phase 4 — ground rumble and flutter

1. Wire ground rumble as `Buffet` amplitude gated by `OnGround * gs`
2. Wire flutter onset as speed-threshold `buffet` func near Vne
3. Combine with stall buffet via `Max` or `Add`


---

## 10. Open Questions

### Load force

1. **Alpha cue**: should pitch `LoadForce` include an alpha component
   for stick-pusher feel at high AoA? Could add
   `alpha_gain * max(0, alpha - alpha_threshold)` as a stall warning
   force. Already have `BuffetAmplitude` for stall — may be redundant.

2. **Asymmetric engine-out**: single-engine aircraft have a strong yaw
   moment on engine failure. `N_aero` or `beta` would capture this, but
   the magnitude needs tuning. Worth it for training realism?

3. **Trim change with config**: flaps, gear, and power changes create
   trim offsets that the pilot feels as transient stick forces. Currently
   our trim system is manual (hat switch). Adding a `XPlane.Trim.Elevator`
   feedback to the graph could auto-trim, but that's a different feature.

4. **Qhat scaling on pitch g-cue**: should `LoadGGain` scale with qhat?
   In a real aircraft, the elevator is more effective at high speed, so
   less stick displacement (but not less force) is needed per g. The
   g-load cue is arguably independent of speed since it represents the
   structural load, not the control effectiveness. Start without qhat
   scaling and evaluate.

### Vibration

5. **Torque reference per engine type**: `torque_norm` needs a reference
   value (param) per aircraft. Typical values: Lycoming O-320 ~400 Nm,
   PT6A ~1800 Nm, R-2800 ~3500 Nm. Graph uses existing `torque_norm`
   func node with an aircraft-specific `TorqueRef` param.

6. **Dual DDS proto field count**: adding DDS 2 fields (fundamental +
   2 amplitudes) brings `FlightFfbAction` to 14 fields. Acceptable for
   protobuf, but check serial bandwidth at 20 Hz send rate.

7. **Per-axis engine amplitude**: in a twin, should the left engine
   vibrate the roll axis more than pitch (and vice versa for right)?
   Physically the engine mounts couple differently to each axis. Could
   be a refinement with per-axis amplitude scaling, or start with equal
   amplitude on all axes.

8. **Ground rumble vs gear type**: tailwheel aircraft are much rougher
   on the ground than tricycle gear. Worth a separate param or just tune
   the ground rumble gain per aircraft profile?
