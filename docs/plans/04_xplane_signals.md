# X-Plane Telemetry Signal Extensions

New X-Plane datarefs for helicopter and fixed-wing FFB. These signals
feed vibration envelopes (plans 08-09) and helicopter load forces (plan 05).

**Status:** Implemented (UDP v4). `BladeAlpha` and `DiscAlpha` deferred.


---

## 1. New Rotor Datarefs

All vibration envelope drivers are available as X-Plane datarefs. Verified
by cross-referencing DataRefs.txt against live flight data (MD 500E dataref
logger, ~9000 samples across hover through 170 kt and back).

### Confirmed datarefs

| Graph signal | X-Plane dataref | Type | Range (MD 500E) | Purpose |
| --- | --- | --- | --- | --- |
| `XPlane.Rotor.BladeAlphPitch` | `sim/flightmodel/cyclic/cyclic_elev_blad_alph[N]` | float[16] | -0.2 (hover) to -6.2 (170 kt) | **1/rev pitch amplitude + longitudinal load force** — dominant, scales with IAS |
| `XPlane.Rotor.BladeAlphRoll` | `sim/flightmodel/cyclic/cyclic_ailn_blad_alph[N]` | float[16] | -0.3 to +0.1 | 1/rev roll amplitude + lateral load force — weak |
| `XPlane.Rotor.Slap` | `sim/flightmodel2/engines/rotor_blade_slap_rat[N]` | float[16] | 0 (hover) to 0.27 (high speed) | **2/rev high-speed envelope** |
| `XPlane.Rotor.VRS` | `sim/flightmodel/engine/vortex_ring_state[N]` | float[16][10] | 0.50 (hover) to 0.25 (60+ kt) | **2/rev ETL envelope** — transition zone 0.50->0.25 IS the ETL |
| `XPlane.Rotor.Propwash` | `sim/flightmodel2/engines/propwash_mtr_sec[N]` | float[16] | 19 (hover) to 2.5 (170 kt) | Downwash velocity — ground effect proxy |

### Deferred datarefs (add in a future UDP version)

| Graph signal | X-Plane dataref | Type | Range (MD 500E) | Purpose |
| --- | --- | --- | --- | --- |
| `XPlane.Rotor.BladeAlpha` | `sim/flightmodel2/engines/rotor_blade_alpha_deg[N]` | float[16] | 2.3 to 4.7 | Retreating blade stall indicator (3/rev) |
| `XPlane.Rotor.DiscAlpha` | `sim/flightmodel2/engines/rotor_disc_alpha_deg[N]` | float[16] | -73 (hover) to -0.2 (90 kt) | Disc AoA — context for blade slap |

### NOT available as datarefs

* `pitch,_flap` / `_roll,_flap` (blade flapping angles) — not exposed.
  **Replaced by** `cyclic_elev_blad_alph` / `cyclic_ailn_blad_alph` which
  track the same envelope shape.
* `swirl,maxkt` (wake swirl velocity) — not exposed.
  **Replaced by** `vortex_ring_state` which provides a clean ETL indicator.

### Derived signals (computed in plugin)

None. DDS fundamental frequency is computed inside the graph as
`MainRotor.Speed / 60` and published as the `Shared.Vib1Fund` output;
no separate `Rotor.FundamentalHz` input signal is needed.


---

## 2. Key Observations from Flight Data

**`blade_alph_pitch` is the dominant rotor signal.** Scales from -0.2
(hover) to -6.2 (170 kt). This is the elevator-axis (longitudinal)
decomposition of blade AoA — X-Plane has already resolved it into the
cyclic axis frame. No 90-degree precession mapping needed. Drives both
**pitch-axis 1/rev vibration** and **longitudinal load force** (aft
stick force building with airspeed).

**`vortex_ring_state` is the ETL indicator.** Value 0.50 in hover (full
recirculation), decays through 0.47 (20 kt) → 0.36 (40 kt) → 0.25
(60+ kt, clean forward flight). The transition zone IS the ETL. Compute
ETL factor as `(vrs - 0.25) / 0.25` → 1.0 at hover, 0.0 at 60+ kt.

**`slap_rat` builds monotonically with speed** — 0 at hover, 0.02 at
80 kt, 0.05 at 170 kt. Driven by blade-vortex interaction when
`disc_alpha` is near zero (vortices not clearing the disc).

**`Q_rotor` / `R_rotor` are too small** (±0.03 rad) and noisy to use as
vibration envelopes. They represent rotor moments, not flapping directly.

**All vibration-relevant signals naturally zero on the ground** (blade_alph
decays when no aerodynamic asymmetry). No `OnGround` gate needed.


---

## 3. Observed Flight Data (MD 500E)

Averaged per 10-kt bin, airborne, RPM > 400:

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

### blade_alph_pitch vs g-load

`blade_alph_pitch` is primarily an **airspeed** signal, not a **g-load**
signal:

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
driver (see plan 05 for the two-component model that adds a g-load term).


---

## 4. Implementation

### UDP packet v4

Added 5 new `float[4]` arrays to `FfbDataPacket` (80 bytes, total 212):

* `cyclic_elev_blad_alph[N]`
* `cyclic_ailn_blad_alph[N]`
* `rotor_blade_slap_rat[N]`
* `vortex_ring_state[N]` — element `[0]` per rotor from `float[16][10]`
* `propwash_mtr_sec[N]`

VRS uses stride-10 indexing: reads `[0], [10], [20], [30]` individually.

### Plugin signal registration

Registered 5 graph input signals in `GraphSignalCatalogData.InputNames`:

```text
XPlane.Rotor.BladeAlphPitch
XPlane.Rotor.BladeAlphRoll
XPlane.Rotor.Slap
XPlane.Rotor.VRS
XPlane.Rotor.Propwash
```

All indexed by `rotorIndex` (same as existing torque/speed signals).

### Files modified

* `XPlanePlugin/DiyFfbDataProvider.cpp` — datarefs, packet struct, version 4
* `SimHubPlugin/DiyFfbPlugin.cs` — `XPlaneUdpPacket`, `ParseXPlanePacket`, version/size
* `SimHubPlugin/GraphSignalCatalogData.cs` — `InputNames`
* `SimHubPlugin/GraphSignals.cs` — `BuildXPlaneInputs`
