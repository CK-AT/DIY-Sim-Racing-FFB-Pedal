# FFB Design Summary (Current)

This document captures the currently implemented FFB terms per aircraft type and per function. See `SimHubPlugin/Docs/XPlane_FFB.md` for pipeline, tuning, and UI details.

## Shared Concepts

- **System references (per aircraft / CarId):**
  - **Plane:** `Vref (kts)` for qhat_eff scaling.
  - **Heli:** `Nominal RPM` for rotor scaling.
- **Torque normalization:** load gains are tuned as **value @ max torque**, using a tracked `abs_max_torque` per function.
- **Trim:** trim and weathervane are treated as offset terms (mm).
- **Safety:** ESP32 reverts to defaults if no FFB updates arrive within 200 ms.

## Fixed-Wing Aircraft (Plane)

### Pitch (FlightStickPitch)
- **Spring:** `k_q * qhat_eff`
- **Damper:** `k_rate * qhat_eff`
- **Friction:** base
- **Load:** **secondary** from aero moment `M_aero`, normalized by max torque
- **Buffet:** `buffet(alpha) * qhat_eff`
- **Weathervane:** optional (alpha-based trim offset)

### Roll (FlightStickRoll)
- **Spring:** `k_q * qhat_eff`
- **Damper:** `k_rate * qhat_eff`
- **Friction:** base
- **Load:** **secondary** from aero moment `L_aero`, normalized by max torque
- **Buffet:** `buffet(alpha) * qhat_eff`
- **Weathervane:** none

### Yaw (FlightPedals)
- **Spring:** `k_q * qhat_eff`
- **Damper:** `k_rate * qhat_eff`
- **Friction:** base
- **Load:** **secondary** from aero moment `N_aero`, normalized by max torque
- **Weathervane:** optional (beta-based trim offset)

## Helicopters (Heli)

### Collective (FlightStickCollective)
- **Spring:** none (0)
- **Damper:** `k_rate * (RPM / Nominal_RPM)`
- **Friction:** base
- **Load:** **primary** from main rotor torque, normalized by max torque
- **Trim:** from prop_ratio offset

### Cyclic Pitch/Roll (FlightStickPitch/Roll)
- **Spring:** `k_q * qhat_eff` (optional; may be reduced)
- **Damper:** `k_rate * qhat_eff` (or torque-scaled if desired)
- **Friction:** base
- **Load:** **small additive** aero-moment term (M/L) only
- **Weathervane:** optional

### Yaw / Pedals (FlightPedals)
- **Spring:** `k_q * qhat_eff` (optional; may be reduced)
- **Damper:** `k_rate * qhat_eff` (or torque-scaled if desired)
- **Friction:** base
- **Load:** **small additive** aero-moment term (N) only
- **Weathervane:** optional
