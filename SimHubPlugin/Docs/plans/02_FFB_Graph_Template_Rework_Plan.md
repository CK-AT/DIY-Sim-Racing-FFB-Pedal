# FFB Graph Template Rework Plan

## Objective

Rework `SimHubPlugin/graphs/templates/` to fully implement the FFB behavior defined in `FFB_Design_Future.md`, using Include graphs for shared calculations.

## Architecture Overview

```
Templates (plane_default, heli_default)
  └── Axis graphs (plane_pitch, heli_cyclic_pitch, etc.)
      └── Building block: heli_scale (helicopters only)
```

### Key Design Decisions

1. **No qhat_calc include** - `qhat_eff` is a single function, called directly in each plane axis
2. **heli_scale include** - Contains torque_norm, rpm_norm, assist_loss + damp_scale lerp; worth sharing
3. **Buffet on all plane axes** - Each axis has own `BuffetGain`, shares `Aircraft.Buffet.AlphaStart/AlphaFull`
4. **Helicopter parameter grouping**:
   - `FlightStickCollective.*` - collective-specific parameters
   - `Cyclic.*` - shared by cyclic pitch AND roll (uniform behavior), appears in Aircraft tab
   - `FlightPedals.*` - pedal-specific parameters

---

## Target FFB Formulas

### Fixed-Wing (Plane) - All Axes

| Output | Formula |
|--------|---------|
| Spring | `k_spring * qhat_eff` |
| Damper | `k_damper * qhat_eff` |
| Friction | `k_friction_base + k_friction_q * qhat_eff` |
| Load | `k_load * (aero_trq / max_aero_trq)` |
| Buffet | `buffet(alpha, start, full, gain, qhat_eff)` |

### Helicopter - Collective

| Output | Formula |
|--------|---------|
| Spring | 0 (none) |
| Damper | `k_damper * damp_scale` |
| Friction | `k_friction_base + k_friction_torque * torque_norm + k_friction_lowrpm * assist_loss` |
| Load | `k_load * torque_norm` |

### Helicopter - Cyclic/Pedals

| Output | Formula |
|--------|---------|
| Spring | `k_center` (constant) |
| Damper | `k_damper * damp_scale` |
| Friction | `k_friction_base + k_friction_torque * torque_norm + k_friction_lowrpm * assist_loss` |
| Load | `k_aero * (aero_trq / max_aero_trq)` (small additive) |

### Helper Formulas (in heli_scale)

```
torque_norm = torque / torque_ref
rpm_norm = rpm / rpm_ref
assist_loss = clamp(1 - rpm_norm, 0, 1)
damp_scale = lerp(torque_norm, torque_norm + assist_loss, rpm_blend)
```

---

## File Structure After Implementation

```
SimHubPlugin/graphs/
├── templates/
│   ├── plane_default.json      # Includes: plane_pitch, plane_roll, plane_yaw
│   └── heli_default.json       # Includes: heli_collective, heli_cyclic_pitch, heli_cyclic_roll, heli_pedals
└── _embedded/
    ├── common/
    │   └── heli_scale.json     # NEW: torque/RPM scaling building block
    ├── plane_pitch.json        # UPDATE: V2, load norm, friction base, buffet
    ├── plane_roll.json         # NEW: roll axis
    ├── plane_yaw.json          # NEW: yaw axis
    ├── heli_collective.json    # UPDATE: include heli_scale, use damp_scale
    ├── heli_cyclic_pitch.json  # NEW: includes heli_scale, uses Cyclic.* params
    ├── heli_cyclic_roll.json   # NEW: includes heli_scale, uses Cyclic.* params
    └── heli_pedals.json        # NEW: pedals axis
```

---

## Implementation Tasks

### Phase 1: Create Building Block

#### Task 1.1: Create `_embedded/common/heli_scale.json`

Create directory `_embedded/common/` and the heli_scale.json file.

**Named Input Ports** (Kind: Input with named ports for Include mapping):
- `in_torque` - Main rotor torque
- `in_torque_ref` - Reference torque (TorqueNom)
- `in_rpm` - Main rotor RPM
- `in_rpm_ref` - Reference RPM (SpeedNom)
- `in_rpm_blend` - Blend factor for damp_scale

**Named Output Ports** (Kind: Output with named ports):
- `out_torque_norm` - Normalized torque
- `out_assist_loss` - Assist loss factor
- `out_damp_scale` - Blended damping scale

**Internal Nodes**:
1. Func node `torque_norm`: inputs (in_torque, in_torque_ref) → out
2. Func node `rpm_norm`: inputs (in_rpm, in_rpm_ref) → out
3. Func node `assist_loss`: input (rpm_norm.out) → out
4. Op node `add`: (torque_norm.out + assist_loss.out) → sum
5. Op node `lerp`: (torque_norm.out, sum, in_rpm_blend) → damp_scale

**Note**: Use Version 1 format with named ports for Include compatibility.

---

### Phase 2: Update Plane Axis Graphs

#### Task 2.1: Update `plane_pitch.json`

Convert from V1 to V2 format and add missing features.

**Input Nodes** (SignalGroup: XPlane):
- `ias` - Speed.IAS
- `alpha` - Angle.Alpha (NEW)
- `aero_trq` - AeroTorque.Pitch

**Param Nodes** (SignalGroup as shown):
- `Aircraft.Vref` - Reference speed
- `Aircraft.AeroTorque.PitchMax` - Max pitch moment (NEW)
- `Aircraft.Buffet.AlphaStart` - Buffet onset (NEW)
- `Aircraft.Buffet.AlphaFull` - Full buffet (NEW)
- `FlightStickPitch.SpringGain`
- `FlightStickPitch.DamperGain`
- `FlightStickPitch.FrictionBase` (NEW)
- `FlightStickPitch.FrictionGain`
- `FlightStickPitch.LoadGain`
- `FlightStickPitch.BuffetGain` (NEW)

**Processing Nodes**:
1. Func `qhat`: qhat_eff(ias, vref) → qhat
2. Op `spring`: mul(qhat, SpringGain) → spring
3. Op `damper`: mul(qhat, DamperGain) → damper
4. Op `fric_scaled`: mul(qhat, FrictionGain) → scaled friction
5. Op `friction`: add(FrictionBase, fric_scaled) → total friction
6. Op `load_norm`: div(aero_trq, PitchMax) → normalized load (NEW)
7. Op `load`: mul(load_norm, LoadGain) → load
8. Func `buffet`: buffet(alpha, AlphaStart, AlphaFull, BuffetGain, qhat) → buffet (NEW)

**Output Nodes** (SignalGroup: FlightStickPitch):
- SpringGain
- DamperGain
- Friction
- LoadForce
- BuffetAmplitude (NEW)

**Param Definitions** (add to Params array):
```json
{
  "Name": "Aircraft.AeroTorque.PitchMax",
  "DefaultValue": 1000.0,
  "Min": 100.0,
  "Max": 10000.0,
  "Ui": { "Widget": "slider", "Label": "Max Pitch Torque", "Group": "Aircraft", "Units": "Nm", "LogScale": true }
}
```

#### Task 2.2: Create `plane_roll.json`

Copy plane_pitch.json and make these changes:
- `AeroTorque.Pitch` → `AeroTorque.Roll`
- `Aircraft.AeroTorque.PitchMax` → `Aircraft.AeroTorque.RollMax`
- `FlightStickPitch.*` → `FlightStickRoll.*`
- Output SignalGroup: `FlightStickRoll`

#### Task 2.3: Create `plane_yaw.json`

Copy plane_roll.json and make these changes:
- `AeroTorque.Roll` → `AeroTorque.Yaw`
- `Aircraft.AeroTorque.RollMax` → `Aircraft.AeroTorque.YawMax`
- `FlightStickRoll.*` → `FlightPedals.*`
- Output SignalGroup: `FlightPedals`

---

### Phase 3: Update Helicopter Axis Graphs

#### Task 3.1: Update `heli_collective.json`

Modify existing file to include heli_scale and use damp_scale.

**Changes**:
1. Add Include node for `common/heli_scale.json`
2. Wire Include inputs from existing torque/rpm/ref nodes
3. Replace direct torque_norm usage with Include outputs
4. Use `out_damp_scale` for damper calculation instead of raw torque_norm
5. Add FrictionBase param and update friction calculation

**Include Node**:
```json
{
  "Id": "scale_include",
  "Title": "Heli Scale",
  "Kind": "Include",
  "IncludePath": "common/heli_scale.json",
  "InputMap": {
    "in_torque": "trq_mr",
    "in_torque_ref": "trq_ref",
    "in_rpm": "rpm",
    "in_rpm_ref": "rpm_ref",
    "in_rpm_blend": "rpm_blend"
  },
  "OutputMap": {
    "out_torque_norm": "inc_torque_norm",
    "out_assist_loss": "inc_assist_loss",
    "out_damp_scale": "inc_damp_scale"
  }
}
```

**New Param**:
- `Aircraft.RpmBlend` - damp_scale blend factor (default 0.5)
- `FlightStickCollective.FrictionBase` - static friction

**Updated Damper Calculation**:
- Change from: `damper = trq_norm * k_damper`
- Change to: `damper = inc_damp_scale * k_damper`

#### Task 3.2: Create `heli_cyclic_pitch.json`

New file for helicopter cyclic pitch axis.

**Input Nodes** (SignalGroup: XPlane):
- `torque` - MainRotor.Torque
- `rpm` - MainRotor.Speed
- `aero_trq` - AeroTorque.Pitch

**Param Nodes**:
- `Aircraft.Rotor.TorqueNom`
- `Aircraft.Rotor.SpeedNom`
- `Aircraft.RpmBlend`
- `Aircraft.AeroTorque.PitchMax`
- `Cyclic.SpringGain` (NOTE: Cyclic group, not FlightStickPitch)
- `Cyclic.DamperGain`
- `Cyclic.FrictionBase`
- `Cyclic.FrictionTorque`
- `Cyclic.FrictionLowRpm`
- `Cyclic.LoadGain`

**Include Node**: Same as heli_collective for heli_scale

**Processing**:
1. Include heli_scale → get torque_norm, assist_loss, damp_scale
2. Spring = Cyclic.SpringGain (direct passthrough)
3. Damper = Cyclic.DamperGain * damp_scale
4. Friction = FrictionBase + (FrictionTorque * torque_norm) + (FrictionLowRpm * assist_loss)
5. Load = LoadGain * (aero_trq / PitchMax)

**Output Nodes** (SignalGroup: FlightStickPitch):
- SpringGain
- DamperGain
- Friction
- LoadForce

#### Task 3.3: Create `heli_cyclic_roll.json`

Copy heli_cyclic_pitch.json and make these changes:
- `AeroTorque.Pitch` → `AeroTorque.Roll`
- `Aircraft.AeroTorque.PitchMax` → `Aircraft.AeroTorque.RollMax`
- Keep `Cyclic.*` params (shared with pitch!)
- Output SignalGroup: `FlightStickRoll`

#### Task 3.4: Create `heli_pedals.json`

Copy heli_cyclic_pitch.json and make these changes:
- `AeroTorque.Pitch` → `AeroTorque.Yaw`
- `Aircraft.AeroTorque.PitchMax` → `Aircraft.AeroTorque.YawMax`
- `Cyclic.*` → `FlightPedals.*` (own param group)
- Output SignalGroup: `FlightPedals`

---

### Phase 4: Update Templates

#### Task 4.1: Update `plane_default.json`

Replace current content with Include nodes for all 3 axes.

**Include Nodes**:
```json
{
  "Id": "pitch_include",
  "Title": "Pitch FFB",
  "Kind": "Include",
  "IncludePath": "../_embedded/plane_pitch.json"
},
{
  "Id": "roll_include",
  "Title": "Roll FFB",
  "Kind": "Include",
  "IncludePath": "../_embedded/plane_roll.json"
},
{
  "Id": "yaw_include",
  "Title": "Yaw FFB",
  "Kind": "Include",
  "IncludePath": "../_embedded/plane_yaw.json"
}
```

**Shared Params** (define once in template):
- `Aircraft.Vref`
- `Aircraft.AeroTorque.PitchMax`
- `Aircraft.AeroTorque.RollMax`
- `Aircraft.AeroTorque.YawMax`
- `Aircraft.Buffet.AlphaStart`
- `Aircraft.Buffet.AlphaFull`

#### Task 4.2: Update `heli_default.json`

Replace current content with Include nodes for all 4 axes.

**Include Nodes**:
```json
{
  "Id": "collective_include",
  "Title": "Collective FFB",
  "Kind": "Include",
  "IncludePath": "../_embedded/heli_collective.json"
},
{
  "Id": "cyclic_pitch_include",
  "Title": "Cyclic Pitch FFB",
  "Kind": "Include",
  "IncludePath": "../_embedded/heli_cyclic_pitch.json"
},
{
  "Id": "cyclic_roll_include",
  "Title": "Cyclic Roll FFB",
  "Kind": "Include",
  "IncludePath": "../_embedded/heli_cyclic_roll.json"
},
{
  "Id": "pedals_include",
  "Title": "Pedals FFB",
  "Kind": "Include",
  "IncludePath": "../_embedded/heli_pedals.json"
}
```

**Shared Params**:
- `Aircraft.Rotor.TorqueNom`
- `Aircraft.Rotor.SpeedNom`
- `Aircraft.RpmBlend`
- `Aircraft.AeroTorque.PitchMax`
- `Aircraft.AeroTorque.RollMax`
- `Aircraft.AeroTorque.YawMax`

---

## Parameter Reference

### Plane - Aircraft Group

| Parameter | Default | Min | Max | Units | LogScale |
|-----------|---------|-----|-----|-------|----------|
| `Aircraft.Vref` | 100 | 50 | 300 | kts | false |
| `Aircraft.AeroTorque.PitchMax` | 1000 | 100 | 10000 | Nm | true |
| `Aircraft.AeroTorque.RollMax` | 1000 | 100 | 10000 | Nm | true |
| `Aircraft.AeroTorque.YawMax` | 500 | 100 | 5000 | Nm | true |
| `Aircraft.Buffet.AlphaStart` | 12 | 0 | 30 | deg | false |
| `Aircraft.Buffet.AlphaFull` | 18 | 0 | 45 | deg | false |

### Plane - Per-Axis (FlightStickPitch, FlightStickRoll, FlightPedals)

| Parameter | Default | Min | Max | Units |
|-----------|---------|-----|-----|-------|
| `*.SpringGain` | 1.0 | 0 | 5 | N/mm @ v_ref |
| `*.DamperGain` | 1.0 | 0 | 5 | N*s/mm @ v_ref |
| `*.FrictionBase` | 0.1 | 0 | 2 | N |
| `*.FrictionGain` | 0.3 | 0 | 5 | N @ v_ref |
| `*.LoadGain` | 0.5 | 0 | 5 | N @ aero_trq_ref |
| `*.BuffetGain` | 0.5 | 0 | 5 | N @ v_ref |

### Helicopter - Aircraft Group

| Parameter | Default | Min | Max | Units | LogScale |
|-----------|---------|-----|-----|-------|----------|
| `Aircraft.Rotor.TorqueNom` | 1000 | 100 | 10000 | Nm | true |
| `Aircraft.Rotor.SpeedNom` | 400 | 100 | 1000 | 1/min | false |
| `Aircraft.RpmBlend` | 0.5 | 0 | 1 | ratio | false |

### Helicopter - FlightStickCollective

| Parameter | Default | Min | Max | Units |
|-----------|---------|-----|-----|-------|
| `FlightStickCollective.DamperGain` | 0.5 | 0 | 5 | N*s/mm @ trq_ref |
| `FlightStickCollective.FrictionBase` | 0.1 | 0 | 2 | N |
| `FlightStickCollective.FrictionTorque` | 0.3 | 0 | 5 | N @ trq_ref |
| `FlightStickCollective.FrictionLowRpm` | 0.5 | 0 | 5 | multiplier |
| `FlightStickCollective.LoadGain` | 0.8 | 0 | 5 | N @ trq_ref |

### Helicopter - Cyclic (shared by pitch AND roll)

| Parameter | Default | Min | Max | Units |
|-----------|---------|-----|-----|-------|
| `Cyclic.SpringGain` | 1.0 | 0 | 5 | N/mm @ trq_ref |
| `Cyclic.DamperGain` | 0.5 | 0 | 5 | N*s/mm @ trq_ref |
| `Cyclic.FrictionBase` | 0.1 | 0 | 2 | N |
| `Cyclic.FrictionTorque` | 0.3 | 0 | 5 | N @ trq_ref |
| `Cyclic.FrictionLowRpm` | 0.5 | 0 | 5 | multiplier |
| `Cyclic.LoadGain` | 0.2 | 0 | 5 | N @ aero_trq_ref |

### Helicopter - FlightPedals

| Parameter | Default | Min | Max | Units |
|-----------|---------|-----|-----|-------|
| `FlightPedals.SpringGain` | 1.0 | 0 | 5 | N/mm @ trq_ref |
| `FlightPedals.DamperGain` | 0.5 | 0 | 5 | N*s/mm @ trq_ref |
| `FlightPedals.FrictionBase` | 0.1 | 0 | 2 | N |
| `FlightPedals.FrictionTorque` | 0.3 | 0 | 5 | N @ trq_ref |
| `FlightPedals.FrictionLowRpm` | 0.5 | 0 | 5 | multiplier |
| `FlightPedals.LoadGain` | 0.2 | 0 | 5 | N @ aero_trq_ref |

---

## Verification Checklist

- [ ] heli_scale.json evaluates correctly with test inputs
- [ ] plane_pitch.json loads and previews correctly
- [ ] plane_roll.json loads and previews correctly
- [ ] plane_yaw.json loads and previews correctly
- [ ] heli_collective.json includes resolve and evaluate
- [ ] heli_cyclic_pitch.json loads and previews correctly
- [ ] heli_cyclic_roll.json loads and previews correctly
- [ ] heli_pedals.json loads and previews correctly
- [ ] plane_default.json composes all 3 plane axes
- [ ] heli_default.json composes all 4 heli axes
- [ ] Parameter cascade works (include default → graph override → profile override)
- [ ] Update FFB_Graph_Progress.md with completed items

---

## Reference Files

- Target behavior spec: `SimHubPlugin/Docs/FFB_Design_Future.md`
- Current behavior spec: `SimHubPlugin/Docs/FFB_Design_Current.md`
- Graph node types: `SimHubPlugin/Docs/Graph_Node_Types_Design.md`
- Signal catalog: `SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md`
- Existing plane_pitch.json: `SimHubPlugin/graphs/_embedded/plane_pitch.json`
- Existing heli_collective.json: `SimHubPlugin/graphs/_embedded/heli_collective.json`
