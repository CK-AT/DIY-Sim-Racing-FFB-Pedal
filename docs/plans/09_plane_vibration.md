# Fixed-Wing Vibration

Vibration sources and rendering for fixed-wing aircraft. Uses the DDS
subsystem from plan 07. No new X-Plane signals required — all inputs
are already in the UDP packet.

**Prerequisites:** Plan 07 (DDS subsystem)


---

## 1. Vibration Sources

### 1.1 Engine vibration (DDS — periodic)

The most important "alive" cue for propeller aircraft. Constant vibration
at engine RPM, always present while the engine runs.

**Rendering**: uses the `SyncVib` DDS oscillator. For fixed-wing, only
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

### 1.2 Multi-engine beat frequency (dual DDS)

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
asymmetric.

**Gateway sync**: engine DDS does NOT need phase sync across axes — there's
no physical phase relationship between engine vibration on pitch vs roll
axes. Each axis free-runs its engine DDS independently. Only the rotor
DDS needs gateway PLL sync (for the 1/rev ellipse). This simplifies the
twin-engine implementation significantly.

### 1.3 Stall buffet (existing — incoherent noise)

Already implemented via the `buffet` graph func node. Band-limited random
noise gated by alpha approaching stall.

```
BuffetAmplitude = buffet(alpha, alpha_start, alpha_full, gain, qhat)
```

No changes needed. Rendered by the existing `Buffet` element (`f_vib`
after the damping/friction bypass from plan 07 is implemented).

### 1.4 Ground rumble (incoherent noise)

Vibration during taxi, takeoff roll, and landing rollout. Random,
speed-dependent, not periodic.

**Rendering**: use the existing `Buffet` element with amplitude gated by
`OnGround` and scaled by groundspeed.

```
BuffetAmplitude = Min(ground_gain * groundspeed * OnGround, ground_max)
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

### 1.5 Speed / flutter onset (incoherent noise)

Near Vne, control surfaces may start to buzz from aerodynamic excitation.
Similar to stall buffet but driven by airspeed instead of alpha.

```
flutter_buffet = buffet(IAS, vne_onset_kts, vne_full_kts, flutter_gain, 1.0)
```

Uses the same `buffet` func node with speed thresholds instead of alpha
thresholds. Combined with stall buffet via `Max` or `Add`.

### 1.6 Summary — rendering method per source

| Source | Element | Periodic? | Sync needed? | Signals |
| --- | --- | --- | --- | --- |
| Engine vibration | SyncVib DDS 1 | Yes (1/rev, 2/rev) | No | prop_rpm[0] |
| Second engine | SyncVib DDS 2 | Yes (1/rev) | No | prop_rpm[1] |
| Stall buffet | Buffet (existing) | No (noise) | No | alpha, qhat |
| Ground rumble | Buffet (existing) | No (noise) | No | OnGround, gs |
| Flutter onset | Buffet (existing) | No (noise) | No | IAS |


---

## 2. Slot Assignment Profiles

### `plane_single` — single-engine fixed-wing

DDS 1 fundamental: prop RPM / 60.
DDS 2: disabled (fundamental = 0).

| DDS | Slot | Ratio | Source | Envelope driver |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1.0 | Engine 1/rev | base + `torque_norm` |
| 1 | 2 | 2.0 | Engine 2/rev | base constant |
| 1 | 3-5 | — | Unused | amplitude = 0 |

`rotation_sign = 0` (no axis split — engine vibration is isotropic).

Graph template: `plane_engine_vib.json`.

### `plane_twin` — twin-engine fixed-wing

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


---

## 3. Per-Aircraft Tuning

### Light GA single (Cessna 172, PA-28)

| Param | Value | Notes |
| --- | --- | --- |
| Engine Vib1Rev base | 0.3 N | Lycoming O-320, smooth 4-cyl |
| Engine Vib2Rev base | 0.1 N | Subtle 2nd harmonic |
| Ground rumble gain | 0.5 N | Light airframe |
| Stall buffet gain | existing | Already tuned per alpha |

### Light GA twin (Baron 58, PA-34)

| Param | Value | Notes |
| --- | --- | --- |
| Engine 1 Vib1Rev base | 0.3 N | Left engine |
| Engine 2 Vib1Rev base | 0.3 N | Right engine |
| Beat frequency | natural | DDS interference at RPM delta |

Twin-engine beat: with props synced (0 RPM delta), steady vibration.
Props 2 RPM apart → 2 Hz beat. Prop sync switch → beat disappears.
Engine failure → one DDS goes silent → asymmetric vibration + no beat.

### Turboprop (TBM 930, King Air)

| Param | Value | Notes |
| --- | --- | --- |
| Engine Vib1Rev base | 0.2 N | Turbine is smoother than piston |
| Ground rumble gain | 0.8 N | Heavier airframe |

### Warbird / aerobatic (Extra 300, P-51)

| Param | Value | Notes |
| --- | --- | --- |
| Engine Vib1Rev base | 0.8 N | Big radial = lots of vibration |
| Engine Vib2Rev base | 0.3 N | Pronounced 2nd harmonic |
| Ground rumble gain | 1.0 N | Taildragger = rough taxi |

All values are starting points — tune by feel in-sim.


---

## 4. Implementation

### Phase 1 — engine vibration (requires DDS from plan 07)

1. Wire `VibFundamental = prop_rpm / 60` in plane graphs
2. Wire `Vib1Rev` amplitude from base + torque scaling
3. Test single-engine — verify "alive" feel, engine failure cue
4. Tune per engine type (Lycoming vs Rotax vs radial)

### Phase 2 — dual DDS for twins

1. Add graph outputs for engine 2 vibration
2. Wire in twin-engine template
3. Test prop sync — verify beat frequency appears/disappears
4. Test engine failure — verify asymmetric vibration

### Phase 3 — ground rumble and flutter

1. Wire ground rumble as `Buffet` amplitude gated by `OnGround * gs`
2. Wire flutter onset as speed-threshold `buffet` func near Vne
3. Combine with stall buffet via `Max` or `Add`


---

## 5. Open Questions

1. **Torque reference per engine type**: `torque_norm` needs a reference
   value (param) per aircraft. Typical values: Lycoming O-320 ~400 Nm,
   PT6A ~1800 Nm, R-2800 ~3500 Nm. Graph uses existing `torque_norm`
   func node with an aircraft-specific `TorqueRef` param.

2. **Per-axis engine amplitude**: in a twin, should the left engine
   vibrate the roll axis more than pitch (and vice versa for right)?
   Physically the engine mounts couple differently to each axis. Could
   be a refinement with per-axis amplitude scaling, or start with equal
   amplitude on all axes.

3. **Ground rumble vs gear type**: tailwheel aircraft are much rougher
   on the ground than tricycle gear. Worth a separate param or just tune
   the ground rumble gain per aircraft profile?
