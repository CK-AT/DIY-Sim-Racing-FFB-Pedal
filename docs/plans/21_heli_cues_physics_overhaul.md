# Heli Cues Physics Overhaul

Follow-up to the physics review of
[heli_unboosted_msfs.json](../../SimHubPlugin/graphs/templates/heli_unboosted_msfs.json)
and its includes. The architecture is sound — advance-ratio chain,
ETL Gaussian, RBS blade-loading proxy, asymmetric VRS lag, harmonic
ratio plumbing (1/N/2N + TR gear ratios) and rotation-sign handling
all check out. But the review found one real tuning bug, two
wired-but-dead inputs, one harmonic-channel inconsistency, a VRS
model that fires in autorotation, and a list of cue-quality gaps.
This plan turns those findings into ordered work items. Phases 4–5
add cues the review identified as missing entirely (maneuvering
load, weathercock pedals, ground resonance, …).

**Status:** implemented through commit 7 (W1–W22, W25) on
`ck_heli_cues_rework`; W23 (AGL plumbing, D4) and W24 (engine Vib2,
D5) deferred. Hardware retune pass pending.
**Decisions taken:** D1 = bypass Nm round-trip, D2 = implement VRS
1/rev shake, D3 = accept shared-include flow-through.

**Implementation deviations from the spec below:**

- **W1 went further:** `heli_scale.json` is removed from the MSFS
  template entirely (not just the torque link) — `rpm_norm` is a
  direct divide by a new template-level `Aircraft.Rotor.SpeedNom`
  param. Otherwise dead `TorqueNom`/`RpmBlend` sliders would remain
  in the MSFS UI. heli_scale itself untouched (X-Plane/boosted/SAS
  unaffected, validated).
- **W21 defaults:** `TurbBuffetGain` ships at 1.0 on cyclic and
  collective (quiet in calm air since the G high-pass idles at ~0)
  and 0.0 on pedals.
- **X-Plane wiring state:** W19/W20 fully wired (`XPlane.OnGround`
  exists). W15/W16/W17/W21/W22 include-inputs are left unwired on
  X-Plane (template has no G/TAS/beta/TR-pitch inputs) — they
  default to 0 and no-op, same precedent as its unwired
  `etl_bump`/`rbs` force inputs.
- A structural validator
  ([validate_graphs.py](../../SimHubPlugin/graphs/validate_graphs.py))
  now checks link/port integrity incl. include-resolved ports and
  bus pairing; all 22 graph JSONs pass.

**Branch base:** `ck_heli_cues_rework` (continues the VRS full-range
rework from c31c3171)

---

## Blast radius

The unboosted includes are shared. Changes hit more than the MSFS
template:

| File | Used by |
|---|---|
| `_embedded/heli_cyclic_unboosted.json`, `heli_collective_unboosted.json`, `heli_pedals_unboosted.json`, `heli_vibration_{cyclic,pedal,collective}.json` | `heli_unboosted_msfs.json` **and** `heli_unboosted.json` (X-Plane) |
| `_embedded/common/heli_scale.json` | all four heli templates (`heli_unboosted_msfs`, `heli_unboosted`, `heli_sas`, `heli_boosted`) |
| `_embedded/msfs_derivations.json` | `heli_unboosted_msfs.json` only |

Any include change must be sanity-checked against
[heli_unboosted.json](../../SimHubPlugin/graphs/templates/heli_unboosted.json)
(X-Plane feeds the same input ports from datarefs instead of the
derivations include). `heli_scale.json` changes additionally affect
the boosted/SAS templates.

---

## Phase 1 — correctness fixes

### W1. Torque double-normalization (the real bug)

[msfs_derivations.json](../../SimHubPlugin/graphs/_embedded/msfs_derivations.json)
converts `Eng.TorquePct/100 × Aircraft.MaxTorqueNm` (default **800**)
into Nm; [heli_scale.json](../../SimHubPlugin/graphs/_embedded/common/heli_scale.json)
then divides by `Aircraft.Rotor.TorqueNom` (default **1000**). At
100% engine torque, `torque_norm = 0.8` — the collective load
(labelled "N @ trq_ref") is silently 20% weak, and the user has two
sliders that must be kept equal with no hint that they're coupled.

**Fix (see D1):** make the MSFS template bypass the Nm round-trip —
feed `Eng.TorquePct/100` straight into the `torque_norm` path so
100% torque ⇒ `torque_norm = 1.0`, and drop `Aircraft.MaxTorqueNm`
from the template params. `heli_scale.json` itself stays unchanged
(X-Plane delivers real Nm and genuinely needs `TorqueNom`).

### W2. Wired-but-dead VRS inputs in the vibration includes

The top-level graph routes the VRS bus into all three vib includes,
and both
[heli_vibration_cyclic.json](../../SimHubPlugin/graphs/_embedded/heli_vibration_cyclic.json)
(`in_vrs`, node only) and
[heli_vibration_collective.json](../../SimHubPlugin/graphs/_embedded/heli_vibration_collective.json)
(`in_vrs`, node only) declare the input — but **no link leaves either
node**. VRS shake currently exists only via the `BuffetAmplitude`
channel in the force includes.

**Fix (see D2):** finish the hookup rather than delete it. VRS is a
strong low-frequency airframe shudder; add
`vrs × VrsVibGain × rpm_norm` into `Vib1Ampl1` (1/rev) in both
includes, with a new muteable gain param per group
(`Cyclic.VrsVibGain`, `Collective.VrsVibGain`, default ~0.3 mm).
The stochastic buffet stays as-is; the two channels are
complementary (periodic shake + random buffet).

### W3. Collective RBS heave on the wrong harmonic

[heli_vibration_collective.json](../../SimHubPlugin/graphs/_embedded/heli_vibration_collective.json)
adds `rbs × RbsHeaveGain` into `Vib1Ampl3` (**2N/rev**, ~67 Hz at
defaults) while the cyclic puts RBS shake on 1/rev + N/rev. Stall
heave is a low-frequency event; at 2N it reads as a high buzz.

**Fix:** move the RBS heave term from `add_vib3_total` into the
N/rev sum (`add_nrev_total`, which becomes a 4-input add like the
cyclic's `add_nrev_slap`). `Vib1Ampl3` reverts to pure
`base_2nrev × rpm_norm`.

### W4. VRS fires in low-speed autorotation

The VRS core in
[msfs_derivations.json](../../SimHubPlugin/graphs/_embedded/msfs_derivations.json)
is `2.5 × fwd_factor × 0.5 × descent_norm` — no power term. A steep
auto at 30 kt / 2000 fpm produces a strong VRS cue, but a windmilling
rotor has no vortex ring.

**Fix:** multiply the core by `clamp_collective` (already computed
for RBS) before the lag. Powered descent keeps the cue; bottomed
collective kills it. Collective position is a better gate than
torque here because autos are flown at flat pitch regardless of
residual engine torque indication.

---

## Phase 2 — cue quality

### W5. Pedal spring re-centering (flat-pitch null)

There is no force path from main-rotor torque to the pedals. What
unboosted pedals actually transmit is the TR blades' centrifugal
twisting moment — a spring toward the **flat-pitch** pedal
position with stiffness ∝ Ω². The "pedal force tracks torque"
correlation pilots report is indirect: high power ⇒ you *hold*
more TR pitch ⇒ more spring deflection. No torque telemetry
needed; the previously-drafted `LoadForce ∝ torque` cue is
dropped (it would keep pushing during pedal turns toward flat
pitch, where real pedals get lighter).

**Fix:** keep the spring output in
[heli_pedals_unboosted.json](../../SimHubPlugin/graphs/_embedded/heli_pedals_unboosted.json)
but shift its center via a scoped `TrimOffset` output (the pedal
channel shares the stick axes' FFB outputs):
`TrimOffset = SpringOffset × RotationSign`. Stiffness becomes
`rpm_norm²` via W10. New param `FlightPedals.SpringOffset` (mm,
default ~3; axis sign convention settled on hardware).

**Tuning rule:** set the offset so pedal force ≈ 0 in trimmed,
ball-centered cruise — this mirrors what real designs do with
counterweights/trim springs, and the fin unloading the TR in
cruise puts the physical flat-pitch null near cruise trim anyway.
Emergent behavior, all real: sustained power-pedal force in hover,
asymmetric pedal weights, light feet in cruise, slack pedals in
autorotation and during spool-up, force lightening mid pedal-turn.

### W6. Slap should be descent-gated

Slap in the derivations is monotonic with TAS (linear above
`SlapOnsetKts`, capped). Real blade slap/BVI peaks in shallow
powered descents at 50–80 kt and in flares. At the 0.03 default
gain the cruise contribution is ~0.01–0.03 mm vs the 0.3 mm N/rev
base — inaudible, always-on noise.

**Fix:** multiply the slap product by a descent factor before the
cap. Reuse the existing descent chain with a much lower saturation
(BVI saturates around 500–800 fpm, not `DescentMaxFpm`): add
`min(descent_fpm / 600, 1)` as a third factor in `mul_slap_k`'s
chain (600 fpm as a named const; promote to a param only if tuning
shows the need). Raise the default `KSlap` so the cue is actually
felt in a descending pass (retune target: slap ≈ 0.05–0.1 mm at
60 kt / 500 fpm).

### W7. Clamp before the VRS lag

The VRS core saturates at 1.25 but every consumer clamps at 1.0,
and `lag_asym` runs on the **unclamped** value. Exiting deep VRS the
output spends `2·ln(1.25) ≈ 0.45 s` decaying invisibly from 1.25 to
1.0 before the felt cue starts to fade — hidden exit latency on top
of the intended 2 s tau.

**Fix:** insert a `clamp 0..1` between `mul_vrs_core` and
`lag_asym_vrs` in the derivations. The 1.25 headroom keeps its
purpose (full VRS at 80% of `DescentMaxFpm`) but stops poisoning
the lag state. Consumer-side clamps stay (other templates feed the
ports too).

### W8. VRS attenuation should cover ETL/RBS loads

In [heli_cyclic_unboosted.json](../../SimHubPlugin/graphs/_embedded/heli_cyclic_unboosted.json)
the `1 − VrsAttenGain × vrs_eff` factor multiplies only the flapping
load; ETL and RBS load terms are added afterwards. ETL (peak 18 kt)
and VRS overlap in the low-speed descent regime, so a crisp ETL bump
punches through an otherwise-mushy stick.

**Fix:** move `mul_load_atten` after `add_total_load` (attenuate the
sum, then OWL). RBS at VRS speeds is ~zero anyway (mu ≈ 0), so the
only behavior change is the intended one.

### W9. Let low G relax RBS

`max(G, 1)` in the RBS demand means pushovers never reduce stall
margin demand, although unloading the rotor genuinely does — and
"lower G to exit RBS" is the textbook recovery.

**Fix:** replace `max(G, 1)` with `clamp(G, 0, Gmax)` (`max_g_factor`
becomes a clamp; reuse `const_zero`, cap at the existing slider-less
constant 3.0 to keep turbulence spikes bounded).

### W10. Ω² scaling consistency

1/rev mass imbalance correctly uses `rpm_norm²` (centrifugal), but
aerodynamic terms scale linearly: track drift (cyclic vib), TR
blade-passing amplitudes (pedal vib), and the pedal spring. Aero
forces go with Ω². Only audible during spool-up/down, where the
linear versions feel too strong at low rpm.

**Fix:** switch track drift, both TR amplitude products, and the
pedal spring to `rpm_norm²` (the `mul_rpm_sq` node already exists in
both vib includes; pedal include gains one). Defaults unchanged —
at nominal rpm the values are identical.

### W11. 1/rev amplitude saturation near Vne

`gain_1rev × |blade_alph|` grows unbounded with speed
(blade_alph ≈ 17·mu ⇒ ~6° ⇒ ~3 mm at the 0.5 default gain). The
slider caps the gain, not the product.

**Fix:** clamp the product at a new `Cyclic.Vib1Max` param
(default 2.0 mm) before it enters `add_1rev_abc`. Cheap insurance
against firmware-side amplitude saturation artifacts.

---

## Phase 3 — cosmetic / hygiene (no behavior change)

- **W12. Rename bus send/recv ports** in
  [heli_unboosted_msfs.json](../../SimHubPlugin/graphs/templates/heli_unboosted_msfs.json)
  to match their `BusName`s. The wiring is verified correct today,
  but the port literally named "VRS" carries `ETLBump`, "ETLBump"
  carries `BladeAlphaPitch`, etc. — a mis-patch waiting to happen.
- **W13. Explicit const for OWL clamp**: `owl_clamp.a` in the cyclic
  include is unconnected and relies on the implicit-0 default to
  implement `min(load, 0)`. Wire a `0` const.
- **W14. Drop the cancelling double-negate** in `mul_alph_roll`
  (derivations): both `a` and `b` negate, which cancels. Remove both
  flags; output is identical.

---

## Phase 4 — missing cues, no new sim signals

All inputs already exist in the graph; these touch only files
already being edited in phases 1–2.

### W15. Stick force per G (cyclic maneuvering load)

In an unboosted helicopter, pulling G stiffens the cyclic — blade
pitching moments grow with disc loading. `G_Nrml` is already an
input but feeds only RBS. Day-to-day this is a bigger cue than RBS:
it makes flares, turns and cyclic climbs feel loaded.

**Fix:** derivations compute `g_load = G − 1` (signed — pushovers
lighten the stick) and broadcast it on a new `GLoad` bus. The cyclic
include gains a `g_load_gain_eff` input (same pattern as
`etl_load_gain_eff`) and adds `g_load × gain × rpm_norm` into the
total load — `add_total_load` grows to a 4-input add. Top level
feeds `Cyclic.GLoadGain` (N/G, muteable, default ~1.0) to the pitch
instance.

**W15b (optional, ships silent).** Roll gets a second-order G
coupling instead of a bare `0` const: coning grows with G, and a
coned rotor in forward flight develops lateral flapping
(`b1 ≈ k · a0 · mu`, `a0 ∝ G`) that returns as a lateral cyclic
force — the G-driven sibling of the existing sideslip cue
(`BladeAlphaRoll = k·beta·mu`, same mechanism, different
excitation). Top level computes
`GLoadRollGain × g_load × mu × RotationSign` and feeds it to the
roll instance's `g_load_gain_eff`-path; derivations broadcast `mu`
on a new `Mu` bus (W17 can reuse it for the buzz onset instead of a
private signal). Properties the implementation must keep: zero in
hover regardless of G (the `mu` factor), flips with rotor direction
(RotationSign family, like the ETL/RBS roll gains), and small —
`Cyclic.GLoadRollGain` defaults to **0** (muteable; try ~0.3 on
hardware, expect ~20–30% of the pitch gradient).

### W16. TR inflow modulation in sideslip (small)

The fin's weathercock load never reaches the pedals — fin loads go
into the airframe, not the control run. What *is* real: sideslip
changes the axial inflow through the TR disc, shifting blade
feathering moments (plus delta-3 flapping feedback on teetering
TRs). A small, steady force modulation, felt mostly as a trim-force
change when the ball is off-center — in real machines sideslip
shows up more as pedal *position* than force.

**Fix:** derivations compute
`tr_inflow = beta_deg × min((TAS / 60kt)², 1)` and broadcast on a
`TrInflow` bus. The pedal include gains a scoped `LoadForce`
output: `LoadForce = InflowGain × tr_inflow × RotationSign ×
rpm_norm²`. New param `FlightPedals.InflowGain` (N/deg, muteable,
**small** default ~0.05). Sign caveat: the true direction depends
on delta-3 and pitch-horn geometry — keep the gain signed and
settle the sign on hardware.

### W17. High-mu compressibility buzz

Above mu ≈ 0.35 the advancing blade picks up a growing N/rev buzz
toward Vne. Today the only speed cue is the static flapback
gradient.

**Fix:** derivations compute
`mu_buzz = clamp((mu − MuBuzzOnset) / MuBuzzWidth, 0, 1)` (onset
0.32, width 0.08, both as Aircraft params) and broadcast on a
`MuBuzz` bus. Cyclic vib include adds `mu_buzz × MuBuzzGain` into
the N/rev sum. `add_nrev_slap` already uses all four ports — chain
one extra 2-input add. New param `Cyclic.MuBuzzGain` (mm,
muteable, default 0.3). If W15b lands first, raw `mu` is already on
the `Mu` bus — compute the onset ramp at top level from that
instead of adding a second mu-derived bus signal.

### W18. TR 1/rev imbalance on pedals

Pedal vibration has TR blade-passing + 2nd harmonic but no TR
once-per-rev (track/balance) — and `Vib1Ampl5` / `HarmRatio5` sit
unused in
[heli_vibration_pedal.json](../../SimHubPlugin/graphs/_embedded/heli_vibration_pedal.json).

**Fix:** `HarmRatio5 = tr_gear` (ratio alone, not × blades),
`Vib1Ampl5 = gain_tr_1rev × rpm_norm²`. New param
`Pedal.gain_tr_1rev` (mm, default 0.05). Uses the rpm² node from
W10. Note: the *aero* share of TR 1/rev (edgewise advancing/
retreating asymmetry) genuinely grows with airspeed while the
imbalance share doesn't — ship one gain first, split in an
optional `× (1 + TAS/V_ref)` term only if hover-vs-cruise tuning
conflicts.

### W25. Collective flat-pitch spring (same critique as W5)

The existing collective `LoadForce = −LoadGain × torque_norm` has
the same flaw the dropped pedal torque cue had: what an unboosted
collective actually transmits is the main-rotor blades' feathering
moments — centrifugal twisting moment toward flat pitch plus a
thrust-dependent aero share. Collective position *is* blade pitch,
so the faithful model is a position spring, not a torque load.
(Reality check: release the friction in an R22 and the lever
falls.) Failure signature of the current model: lowering collective
on approach while torque is still up keeps pulling, where the real
lever lightens.

**Fix:** [heli_collective_unboosted.json](../../SimHubPlugin/graphs/_embedded/heli_collective_unboosted.json)
gains scoped `SpringGain` + `TrimOffset` outputs (shared FFB
infrastructure): spring centered at the low-pitch stop
(`TrimOffset` places the center; `FlightStickCollective.SpringGain`
× rpm_norm² for stiffness). Demote the existing torque `LoadForce`
to the small thrust-dependent residual (default ~25% of today's
value; muteable). Emergent, all real: lever force builds during
spool-up, goes light at flat pitch, "falls" with friction off,
lightens entering autorotation.

**Note on trim:** hat trim is *real* unboosted equipment — the
MD500 and Enstrom machines have electric trim actuators that
reposition the cyclic spring anchor at a slow beep rate, which is
exactly what the include's `TrimStep` (mm/s) accumulator
implements. The only deliberate exception to the real-cues rule is
the **FTR button**: instant spring re-referencing is a
force-trim-system feature from boosted machines (Bell 206 style);
the friction-only end of the spectrum (R22) has neither. FTR stays
for rig usability, but don't tune force cues to match an
FTR-equipped feel.

---

## Phase 5 — missing cues needing new wiring or signals

`MSFS.OnGround` and `XPlane.OnGround` already exist in
[GraphSignalCatalogData.cs:105](../../SimHubPlugin/GraphSignalCatalogData.cs#L105),
and `SIM ON GROUND` is already in
[MsfsSimVarTable.cs:56](../../SimHubPlugin/Msfs/MsfsSimVarTable.cs#L56)
— W19/W20 need **no C# plumbing**, only graph wiring. Only W23
needs a new SimVar.

### W19. Ground resonance / on-ground shake

With weight on wheels and the rotor turning, the airframe transmits
a strong 1/rev–N/rev that vanishes at lift-off — its disappearance
*is* the lift-off cue. Best feel-per-effort item in this plan.

**Fix:** new top-level Input `MSFS.OnGround` → `OnGround` bus → all
three vib includes get an `on_ground` input. Multiply the 1/rev and
N/rev sums by `1 + on_ground × GndResGain` (one shared
`Aircraft.GndResGain` param, default 0.6, muteable). rpm scaling is
already inherent in the amplitude terms, so spool-up on the ground
builds shake naturally.

### W20. Touchdown thump

**Fix:** in the top-level graph: `edge_detect` on `OnGround` gives a
one-frame rising-edge pulse; feed `pulse × TouchdownGain` into
`lag_asym` (tau_up ≈ 0.01 s, tau_down ≈ 0.3 s) to stretch it into a
felt decay, and add it onto the collective + cyclic
`BuffetAmplitude` paths. New param `Aircraft.TouchdownGain` (N,
muteable, default 2.0). No new primitives needed.

### W21. Turbulence buffet

Gusts shake the disc; the pilot feels it in the cyclic before the
airframe visibly moves. The `BuffetAmplitude` channel exists but
has no atmospheric driver, and there is no wind signal in the
catalog — so derive it from G: high-pass via the existing lag
primitive, `turb = |G − lag_asym(G, 0.5, 0.5)|`, clamp, broadcast
on a `Turb` bus, add `turb × TurbBuffetGain` into cyclic +
collective buffet. New params `Cyclic.TurbBuffetGain` /
`Collective.TurbBuffetGain` (N, muteable). Note: this also picks up
maneuvering G transients — acceptable (real discs shake in abrupt
maneuvers too); tune the lag tau on hardware.

### W22. TR-regime pedal buffet (real cue — promoted)

The most distinctive real pedal cue after the spring gradient:
irregular kicking whenever the TR inflow goes dirty — main-rotor
wake ingestion in quartering/crosswind hover, TR vortex ring state
in downwind / critical-side-crosswind hover (the felt precursor to
LTE), and TR blade stall at high pitch demand. The pedal
`BuffetAmplitude` output exists (all flight axes share the same
FFB infrastructure), and all gate signals are already in the
catalog — `MSFS.TailRotor.BladePitchPct` included. Graph wiring
only.

**Fix:** compute the gate at top level:

- `lowspeed = clamp(1 − TAS/30kt, 0, 1)`
- `crit_beta = clamp(beta × RotationSign / 15°, 0, 1)` — one-sided,
  wind from the TR's critical quadrant only
- `demand = clamp((tr_pitch_pct − 70) / 30, 0, 1)` — blade-stall
  proxy (high DA / low rpm / max-power hover)

`BuffetAmplitude = TrBuffetGain × lowspeed × clamp(crit_beta + demand, 0, 1)`.
New param `FlightPedals.TrBuffetGain` (N, muteable, default ~1.0).
Optionally add W21's `turb × FlightPedals.TurbBuffetGain`
(default 0) onto the same output.

### W23. Ground effect (only item needing C# plumbing)

Inside IGE (height ≲ rotor diameter) torque demand drops and the
disc smooths out — what makes hover height readable without
looking.

**Fix:** add `PLANE ALT ABOVE GROUND MINUS CG` (feet) to
`MsfsSimVarTable` and `MSFS.Height.AGL` to the signal catalog (D4).
Derivations: `ige = clamp(1 − agl_m / RotorDiameterM, 0, 1)`
(diameter param already exists; mind the ft→m conversion).
Consumers: scale collective `LoadForce` by `1 − IgeLoadRelief × ige`
and the collective/cyclic N/rev base by `1 − IgeVibRelief × ige`
(defaults ~0.2 / ~0.3). X-Plane equivalent dataref exists; wire it
in `heli_unboosted.json` in the same commit or leave the port at 0
(harmless — ige = 1 − big number clamps to 0… **no**: unwired AGL
defaults to 0 ⇒ ige = 1 ⇒ permanent ground effect. Feed the
X-Plane template a large const if its dataref isn't wired yet).

### W24 (optional). Engine-frequency vibration via Vib2

`Shared.Vib2Fund` + per-axis `Vib2Ampl1/2` already exist in the
catalog — a second oscillator is available without firmware work.
Piston-heli engine shake (clutch engagement, idle with rotor
disengaged) would drive `Vib2Fund = EngRPM / 60`. Needs an engine
RPM signal in the MSFS catalog (not currently present) → folded
under D5. Skip unless a piston-heli profile demands it.

---

## Open decisions

- **D1. W1 approach.** Recommended: bypass Nm in the MSFS template
  (percentage is already normalized; the Nm round-trip adds two
  coupled sliders for nothing). Alternative: keep the Nm path and
  default `MaxTorqueNm = TorqueNom = 1000` — preserves the "real
  units" reading on the torque signal if anything else ever wants
  Nm. Decide before touching the template.
- **D2. W2 direction.** Recommended: implement the VRS 1/rev shake
  (the wiring intent was clearly there). Alternative: delete the
  dead `in_vrs` nodes + top-level bus wiring and rely on
  `BuffetAmplitude` alone. Either resolves the inconsistency.
- **D3. Scope split with X-Plane.** W2/W3/W5/W8–W11 and all of
  phases 4–5 change shared includes and therefore change the
  X-Plane template's behavior too. Recommended: accept that (it's
  the same physics), but the X-Plane template needs the matching
  top-level wiring in the same commits — most new include inputs
  default to 0 and silently no-op when unwired, **except** W23's
  AGL input, where unwired = 0 means *permanent* ground effect (see
  W23 note).
- **D4. AGL signal plumbing (W23).** Sim-var choice
  (`PLANE ALT ABOVE GROUND MINUS CG` vs `RADIO HEIGHT`), catalog
  name (`MSFS.Height.AGL`), and unit convention (feet at the
  SimVar, meters on the graph?). Only Phase 5 item that touches C#.
- **D5. Engine Vib2 (W24).** Whether to add an engine-RPM SimVar
  and ship the second-oscillator engine shake at all. Recommended:
  defer — no current profile needs it; the plan item exists so the
  Vib2 channel isn't forgotten.

---

## Suggested commit grouping

1. Phase 1 W1 + W4 + W7 (derivations/template only, MSFS-scoped
   except heli_scale untouched) — the "VRS + torque correctness"
   commit.
2. Phase 1 W2 + W3 (vibration includes, both templates) — needs D2.
3. W5 (pedal spring re-centering, both templates' wiring) — needs
   D3.
4. W6 + W8–W11 (cue quality, retune defaults afterwards on
   hardware).
5. Phase 3 cosmetics — separate commit, pure rename/no-op, easy to
   review as such.
6. Phase 4 in three commits: W15 + W17 (cyclic, new buses),
   W16 + W18 (pedals — W16 stacks on W5), and W25 (collective
   re-spec — behavior change, retune `LoadGain` on hardware).
7. Phase 5: W19 + W20 (OnGround wiring, no plumbing), then
   W21 + W22 (turbulence + TR-regime pedal buffet), then W23 alone
   (the C# plumbing commit — needs D4). W24 deferred per D5.

---

## Verification

Per commit:

- **Editor round-trip**: open each affected template in the graph
  editor, confirm no unresolved ports / missing params, save,
  reload.
- **Evaluator spot checks** (GraphTest evaluator, fixed inputs):
  - W1: 100% `Eng.TorquePct` ⇒ `torque_norm = 1.0`.
  - W4: 30 kt / 2000 fpm / collective 0 ⇒ VRS ≈ 0; same with
    collective 0.8 ⇒ VRS > 0.5.
  - W7: step descent 1500→0 fpm, VRS output must start decaying
    within one frame (no 0.45 s plateau at 1.0).
  - W3/W10: dump `Vib1Ampl1..3` + harm ratios at nominal rpm before
    and after — only the intended channel moves.
  - W5: pedal `TrimOffset` flips with RotationSign; `rpm_norm = 0`
    ⇒ spring force 0 (slack pedals); zero-force point sits at
    center + offset, not at center.
  - W15: G = 1 ⇒ zero G-load contribution; G = 2 ⇒ +GLoadGain on
    pitch, roll unchanged at default gains; G = 0.5 ⇒ sign flips.
  - W15b: with GLoadRollGain > 0: G = 2 in hover (mu = 0) ⇒ roll
    still zero; G = 2 at mu = 0.3 ⇒ roll term present, flips with
    RotationSign.
  - W16: beta = ±10° at 60 kt ⇒ small pedal `LoadForce` (a few N
    at most), flips with both beta sign and RotationSign; zero at
    rpm_norm = 0.
  - W22: buffet > 0 only when TAS < 30 kt AND (critical-side beta
    OR TR pitch > 70%); TAS = 50 kt ⇒ 0 regardless of beta/demand;
    beta on the safe side contributes nothing (one-sided clamp).
  - W25: collective zero-force point at the low-pitch stop;
    `rpm_norm = 0` ⇒ lever slack; raising collective at constant
    torque ⇒ force rises (position-driven, not torque-driven);
    lowering collective at high torque ⇒ force *drops*.
  - W17: mu = 0.30 ⇒ no buzz; mu = 0.40 ⇒ full `MuBuzzGain` on
    N/rev only.
  - W19/W20: OnGround 0→1 step ⇒ thump decays over ~0.3 s and
    shake boost engages; 1→0 ⇒ boost drops within one frame, no
    thump.
  - W23: AGL = 0 ⇒ ige = 1; AGL ≥ rotor diameter ⇒ ige = 0; X-Plane
    template with port unwired must NOT sit at ige = 1.
- **On-hardware feel pass** after commit 4: hover → ETL transition →
  cruise → steep powered descent into VRS → recovery; auto entry
  (collective flat) must stay shake-free (W4). After commit 7:
  ground run-up (pedals and collective slack→loaded, W5/W25) →
  lift-off (shake drop, W19) → downwind / critical-side crosswind
  hover (pedal kicking, W22) → IGE hover vs OGE hover (W23) →
  cruise: pedal force ≈ 0 ball-centered, small shift in sideslip
  (W5/W16) → high-speed run (W17) → autorotation entry (pedals and
  collective go light, W5/W25) → landing (W20).
