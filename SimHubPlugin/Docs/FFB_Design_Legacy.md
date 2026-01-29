# FFB Design Summary (Legacy Reference)

> **SUPERSEDED**: This document describes the original hardcoded FFB formula design.
> The FFB Graph System now implements these formulas as configurable graphs.
> See [FFB_Graph_Design.md](FFB_Graph_Design.md) for the current architecture.
>
> Legacy runtime code was removed on 2026-01-29. This document remains as a historical reference
> for the physics rationale and formula derivations only.

This document captures the agreed FFB terms per aircraft type and per function.

## Shared Concepts

- **System references (per aircraft / CarId):**
  - **Plane:**
    - `Vref (kts)` for qhat_eff scaling.
  - **Heli:**
    - `Nominal RPM` for low RPM effects.
    - `Max Main Rotor Torque` for damper and friction scaling.
  - **Both:** `Max Aero Torque` for load, per function.
- **Normalization:** spring/damping/friction/load gains are tuned as
  - **value @ <max_aero_torque> Nm (aero)**, using a tracked `abs_max_aero_torque` per function (tracking only active when in reference flight mode, user editable)
  - **value @ <max_rotor_torque> Nm (main rotor)**, using a tracked `abs_max_mr_torque` per helicopter (tracking only active when in reference flight mode, user editable)
  - **value @ <v_ref> kts**, using Vref per plane and thus qhat_eff
- **Trim:** trim and weathervane are treated as offset terms (mm).
- **Safety:** ESP32 reverts to defaults if no FFB updates arrive within 200 ms.
- **Settings Management:**
  - Stage changed settings per aircraft and prompt the user to explicitly save/discard them on aircraft change.

## Definitions / Inputs

| Term | Source | Units | Notes |
| --- | --- | --- | --- |
| `qhat_eff` | SimHub-derived | unitless | Uses IAS and Vref. |
| `mr_torque` | X-Plane dataref | Nm | Main rotor torque (selected rotor or auto). |
| `rpm` | X-Plane dataref | RPM | Main rotor speed (selected rotor or auto). |
| `M_aero` / `L_aero` / `N_aero` | X-Plane datarefs | Nm | Aerodynamic moments (pitch/roll/yaw). |
| `trim` | X-Plane dataref | unitless | Normalized; converted to mm offset. |
| `buffet(alpha)` | SimHub | unitless | Alpha-based effect shaping. |

## Conventions and Safety

- **Sign conventions:** use `f_load = -(k * trq_aero_norm)` so positive aero moments produce restoring load forces.
- **Unit conversions:** RPM values are used directly (not rad/s); trim datarefs are normalized to `[-1, 1]` and converted to mm offsets using the configured travel range.
- **Rotor auto-selection:** when set to “auto,” choose the rotor with the lowest sustained RPM (main rotor). Only evaluate when `on_ground == false`, using a 3 s window; fall back to index 0 if no valid rotor is detected.
- **Output safety:** apply a total force clamp (load terms only) to prevent runaway if inputs spike or telemetry is stale.
- **Telemetry freshness:** ignore samples older than 200 ms for max tracking and rotor auto-selection.

## Fixed-Wing Aircraft (Plane)

### Pitch (FlightStickPitch)
- **Spring:** `k_q * qhat_eff`
- **Damper:** `k_rate * qhat_eff`
- **Friction:** base + `k_friction_q * qhat_eff`
- **Load:** **secondary** from aero moment `M_aero`, normalized by max torque `k_aero * trq_aero_norm`
- **Buffet:** `buffet(alpha) * qhat_eff`
- **Weathervane:** optional (alpha-based trim offset)

### Roll (FlightStickRoll)
- **Spring:** `k_q * qhat_eff`
- **Damper:** `k_rate * qhat_eff`
- **Friction:** base + `k_friction_q * qhat_eff`
- **Load:** **secondary** from aero moment `L_aero`, normalized by max torque `k_aero * trq_aero_norm`
- **Buffet:** `buffet(alpha) * qhat_eff`
- **Weathervane:** none

### Yaw (FlightPedals)
- **Spring:** `k_q * qhat_eff`
- **Damper:** `k_rate * qhat_eff`
- **Friction:** base + `k_friction_q * qhat_eff`
- **Load:** **secondary** from aero moment `N_aero`, normalized by max torque `k_aero * trq_aero_norm`
- **Weathervane:** optional (beta-based trim offset)

## Helicopters (Heli)

### Collective (FlightStickCollective)
- **Spring:** none (0)
- **Damper:** `k_rate * damp_scale`
- **Friction:** base + `k_friction_torque * torque_norm_mr` + `k_friction_low_rpm * low_rpm_factor`
- **Load:** **primary** from main rotor torque, normalized by max torque `k_load_torque * torque_norm_mr`
- **Trim:** from prop_ratio offset
- **Low-RPM behavior:** friction ramp below nominal RPM to emulate reduced hydraulic assist

### Cyclic Pitch/Roll (FlightStickPitch/Roll)
- **Spring:** `k_center` (trim-affected centering)
- **Damper:** `k_rate * damp_scale`
- **Friction:** base + `k_friction_torque * torque_norm_mr` + `k_friction_low_rpm * low_rpm_factor`
- **Load:** **small additive** aero-moment term (M/L) `k_aero * trq_aero_norm`
- **Weathervane:** optional

### Yaw / Pedals (FlightPedals)
- **Spring:** `k_center` (trim-affected centering)
- **Damper:** `k_rate * damp_scale`
- **Friction:** base + `k_friction_torque * torque_norm_mr` + `k_friction_low_rpm * low_rpm_factor`
- **Load:** **small additive** aero-moment term (N) `k_aero * trq_aero_norm`
- **Weathervane:** optional

## Blending (Heli damping)

- Normalize (allow 10% overtorque/overspeed before clamping):
  - `torque_norm_mr = clamp(mr_torque / max_mr_torque, 0..1.1)`
  - `rpm_norm = clamp(rpm / nominal_rpm, 0..1.1)`
- Assist loss from RPM:
  - `assist_loss = 1 - rpm_norm`
- Blend:
  - `damp_scale = lerp(torque_norm_mr, torque_norm_mr + assist_loss, k_rpm_blend)`
  - `k_rpm_blend` is a tunable parameter in [0..1] (0 = torque-only, 1 = torque + full RPM assist loss)

## Notes

- **trq_aero_norm:** normalized by the tracked max aero torque for the function/aircraft.
- **trq_aero_norm clamp:** not clamped by design to preserve rare peak loads; tuning uses the tracked max for normalization instead.
- **torque_norm_mr:** normalized by the tracked max main rotor torque for the helicopter.
- **low_rpm_factor:** only active below nominal RPM; use a smooth ramp to avoid discontinuities.
- **Fixed-wing friction scaling:** keep `k_friction_q` small to preserve trim authority and avoid latch.
- **Assist loss clamp:** keep `assist_loss` clamped to `0..1` even if `rpm_norm` exceeds 1.0.
- **Trim + k_center:** centering targets the trim-adjusted center (weathervane offsets remain separate).
- **Max tracking:** max torque values are updated only during reference-flight mode and can be reset/edited to avoid drift.

## Rationales (Friction/Damper Scaling)
Generally, for servo assisted flight controls, higher loads cause greater breakaway friction (force needed to actuate a servo's pilot valve) and higher damping due to the limitations in hydraulic fluid flow at higher (pressure) demand. Non-assisted flight controls experience similar effects due to linkage stiction and aerodynamic hinge moments.

- **Heli friction vs torque:** rotor torque represents blade loading and linkage forces; scaling friction with torque captures higher breakaway force under load and avoids creep without relying on damping.
- **Heli low-RPM friction:** reduced hydraulic assist at low RPM raises stick forces and stiction; a low-RPM friction ramp emulates this without destabilizing damping.
- **Heli damping blend:** torque provides load-related damping while RPM reflects assist loss; blending avoids over-damping at high torque and preserves feel as RPM drops.
- **Fixed-wing friction vs qhat_eff:** increased dynamic pressure raises control surface loads; a small qhat_eff friction term models static breakaway without overpowering trim.
- **Fixed-wing damping vs qhat_eff:** damping scales with airspeed-dependent hinge moment sensitivity; qhat_eff provides a simple, stable proxy.
