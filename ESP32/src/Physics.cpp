#include <Physics.h>

void SimElement::update(const SimState &state, SimAccumulators &accum) {
}

void CompoundElement::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    for (auto element : _elements) {
        element->update(state, accum);
    }
}

void Spring::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    accum.f_sum = accum.f_sum - ((state.x - _offset) * _k);
}

void Damper::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    float k = state.v < 0.0f ? _k_neg : _k_pos;
    accum.k_damp_sum += max(k, 0.0f);
}

void Buffet::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled || _amplitude <= 0.0f) return;

    uint32_t now = micros();
    float dt = 0.001f;
    if (_last_update_us != 0) {
        dt = (now - _last_update_us) / 1000000.0f;
        if (dt <= 0.0f) {
            dt = 0.001f;
        }
    }
    _last_update_us = now;

    // Xorshift32 RNG for repeatable noise.
    _rng_state ^= _rng_state << 13;
    _rng_state ^= _rng_state >> 17;
    _rng_state ^= _rng_state << 5;
    float noise = (int32_t)(_rng_state & 0x7FFFFF) / 4194303.5f - 1.0f;

    const float tau_fast = 1.0f / (2.0f * PI * 25.0f);
    const float tau_slow = 1.0f / (2.0f * PI * 5.0f);
    float alpha_fast = dt / (tau_fast + dt);
    float alpha_slow = dt / (tau_slow + dt);
    _fast_state += alpha_fast * (noise - _fast_state);
    _slow_state += alpha_slow * (noise - _slow_state);
    float band_noise = _fast_state - _slow_state;

    accum.f_vib += band_noise * _amplitude;
}

void Friction::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    accum.f_static_sum += _f_static;
    accum.f_kin_sum += _f_kin;
    accum.v_eps_max = max(accum.v_eps_max, _v_eps);
}

void ConstForce::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    accum.f_sum += _f;
}

void ForceMap::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    float x = state.x;
    if (x <= _x_vect[0]) {
        accum.f_sum -= _f_vect[0];
        _last_idx = 0;
    } else if (x >= _x_vect.back()) {
        accum.f_sum -= _f_vect.back();
        _last_idx = _x_vect.size() - 1;
    } else {
        while ((_last_idx >= 0) && (_last_idx <= (_x_vect.size() - 1))) {
            if (x < _x_vect[_last_idx]) {
                _last_idx--;
            } else if (x > _x_vect[_last_idx + 1]) {
                _last_idx++;
            } else {
                float k = (_f_vect[_last_idx + 1] - _f_vect[_last_idx]) / (_x_vect[_last_idx + 1] - _x_vect[_last_idx]);
                float d = _f_vect[_last_idx] - (k * _x_vect[_last_idx]);
                accum.f_sum -= ((k * x) + d);
                return;
            }
        }
    }
}

void DampingMap::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    float x = state.x;
    float v = state.v;
    std::vector<float> *k_vect;
    if (v < 0.0) {
        k_vect = &_k_vect_neg;
    } else {
        k_vect = &_k_vect_pos;
    }
    if (x <= _x_vect[0]) {
        accum.k_damp_sum += max(k_vect->front(), 0.0f);
        _last_idx = 0;
    } else if (x >= _x_vect.back()) {
        accum.k_damp_sum += max(k_vect->back(), 0.0f);
        _last_idx = _x_vect.size() - 1;
    } else {
        while ((_last_idx >= 0) && (_last_idx <= (_x_vect.size() - 1))) {
            if (x < _x_vect[_last_idx]) {
                _last_idx--;
            } else if (x > _x_vect[_last_idx + 1]) {
                _last_idx++;
            } else {
                float k = (k_vect->at(_last_idx + 1) - k_vect->at(_last_idx)) / (_x_vect[_last_idx + 1] - _x_vect[_last_idx]);
                float d = k_vect->at(_last_idx) - (k * _x_vect[_last_idx]);
                accum.k_damp_sum += max((k * x) + d, 0.0f);
                return;
            }
        }
    }
}

void Cam::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    float z = (state.x - _center) / _half_width;
    if (z <= -1.0f || z >= 1.0f) return;
    accum.f_sum += _f_max * fastmath::fast_sinf(float(PI) * z);
}

void Sim::update(float &dt, float &f_in, bool final_f) {
    _dt_ms = dt;
    _v = (_x - _x_prev) * 1000.0 / dt;

    SimState state;
    state.x = _x;
    state.v = _v;
    state.a = _a;
    state.dt_ms = dt;
    state.m = _m;
    state.x_min = _x_min;
    state.x_max = _x_max;

    SimAccumulators accum;
    accum.f_sum = f_in;

    if (!final_f) {
        for (auto element : _elements) {
            element->update(state, accum);
        }
    }

    if (accum.has_limits_override) {
        _x_min_tgt = accum.x_min_override;
        _x_max_tgt = accum.x_max_override;
        if (accum.limits_immediate) {
            _x_min = _x_min_tgt;
            _x_max = _x_max_tgt;
        }
    }

    _x_min += constrain(_x_min_tgt - _x_min, -20.0 * dt / 1000.0, 20.0 * dt / 1000.0);
    _x_max += constrain(_x_max_tgt - _x_max, -20.0 * dt / 1000.0, 20.0 * dt / 1000.0);

    // Axis-level safety damping floor (AxisConfig.min_damping). Applied
    // unconditionally regardless of which function is active or what frames
    // are flowing — the safety net for resonance-prone setups.
    if (_min_damping > 0.0f) {
        accum.k_damp_sum = max(accum.k_damp_sum, _min_damping);
    }
    float k_limit = 0.0f;
    if (dt > 0.0f && _m > 0.0f) {
        k_limit = 1.9f * _m / dt;
    }
    // Stability cap comes after the floor so a misconfigured floor cannot
    // violate the integrator stability bound.
    if (k_limit > 0.0f) {
        accum.k_damp_sum = min(accum.k_damp_sum, k_limit);
    }
    if (accum.k_damp_sum > 0.0f) {
        accum.f_sum -= _v * accum.k_damp_sum;
    }

    if (accum.f_static_sum > 0.0f || accum.f_kin_sum > 0.0f) {
        float f_static = max(accum.f_static_sum, 0.0f);
        float f_kin = accum.f_kin_sum > 0.0f ? accum.f_kin_sum : f_static;
        float v_eps = max(accum.v_eps_max, 0.01f);
        float v_abs = fabsf(_v);
        if (v_abs < v_eps) {
            if (fabsf(accum.f_sum) <= f_static) {
                accum.f_sum = 0.0f;
                _v = 0.0f;
                _x_prev = _x;
            } else {
                float dir = accum.f_sum > 0.0f ? 1.0f : -1.0f;
                accum.f_sum -= dir * f_kin;
            }
        } else {
            float dir = _v > 0.0f ? 1.0f : -1.0f;
            accum.f_sum -= dir * f_kin;
        }
    }

    // Inject Buffet's vibration force after damping and friction so the chaotic
    // band-limited noise is not attenuated. Oscillation guard's damping is
    // folded into k_damp_sum above and has already been applied — vibration
    // bypasses it too. SyncVib does not use this path (see plan 12 — it
    // writes accum.x_vib instead and is applied as a position delta on the
    // servo command path in Main.cpp).
    accum.f_sum += accum.f_vib;

    _a = accum.f_sum / _m * 1000.0;
    // Keep x_raw double: 2.0*_x evaluates in double, and a float x_raw would
    // truncate the result back to float ULP, re-introducing the stiction the
    // double position state is meant to remove (see Physics.h _x comment).
    double x_raw = (2.0 * _x) - _x_prev + (((_a * dt * dt) / 1000.0) / 1000.0);
    _x_prev = _x;
    _x = constrain(x_raw, (double)_x_min, (double)_x_max);
    _f_sum = accum.f_sum;
    // SyncVib position delta — the servo path in Main.cpp adds this on top
    // of x_contact_point so the integrator's mass/spring/damper feel is
    // unchanged. accum.x_vib is fresh-zero each call, so the final_f branch
    // (where the element loop is skipped) naturally carries 0.
    _x_vib = accum.x_vib;
}

#ifdef UNIT_TEST
float Sim::compute_force_sum(float f_in) {
    SimState state;
    state.x = _x;
    state.v = _v;
    state.a = _a;
    state.dt_ms = _dt_ms;
    state.m = _m;
    state.x_min = _x_min;
    state.x_max = _x_max;

    SimAccumulators accum;
    accum.f_sum = f_in;
    for (auto element : _elements) {
        element->update(state, accum);
    }
    float k_limit = 0.0f;
    if (_dt_ms > 0.0f && _m > 0.0f) {
        k_limit = 1.9f * _m / _dt_ms;
    }
    if (k_limit > 0.0f) {
        accum.k_damp_sum = min(accum.k_damp_sum, k_limit);
    }
    if (accum.k_damp_sum > 0.0f) {
        accum.f_sum -= _v * accum.k_damp_sum;
    }
    if (accum.f_static_sum > 0.0f || accum.f_kin_sum > 0.0f) {
        float f_static = max(accum.f_static_sum, 0.0f);
        float f_kin = accum.f_kin_sum > 0.0f ? accum.f_kin_sum : f_static;
        float v_eps = max(accum.v_eps_max, 0.01f);
        float v_abs = fabsf(_v);
        if (v_abs < v_eps) {
            if (fabsf(accum.f_sum) <= f_static) {
                accum.f_sum = 0.0f;
            } else {
                float dir = accum.f_sum > 0.0f ? 1.0f : -1.0f;
                accum.f_sum -= dir * f_kin;
            }
        } else {
            float dir = _v > 0.0f ? 1.0f : -1.0f;
            accum.f_sum -= dir * f_kin;
        }
    }
    accum.f_sum += accum.f_vib;
    _x_vib = accum.x_vib;
    return accum.f_sum;
}
#endif

void SyncVib::set_config(float phase_offset, const float *ratios, uint8_t num_slots) {
    _phase_offset = phase_offset;
    if (num_slots > MAX_SLOTS) num_slots = MAX_SLOTS;
    _num_slots = num_slots;
    for (uint8_t i = 0; i < num_slots; i++) {
        _ratios[i] = (ratios != nullptr) ? ratios[i] : 0.0f;
    }
    for (uint8_t i = num_slots; i < MAX_SLOTS; i++) {
        _ratios[i] = 0.0f;
    }
}

void SyncVib::set_amplitudes(const float *targets, uint8_t count) {
    if (count > MAX_SLOTS) count = MAX_SLOTS;
    for (uint8_t i = 0; i < count; i++) {
        _target[i] = (targets != nullptr) ? targets[i] : 0.0f;
    }
    for (uint8_t i = count; i < MAX_SLOTS; i++) {
        _target[i] = 0.0f;
    }
}

void SyncVib::on_sync(float gateway_phase, float gateway_hz) {
    _fundamental_hz = gateway_hz;
    _phase_error = wrap_pm_pi(gateway_phase - _phase);
    _error_integral += _phase_error * 0.01f;  // dt_sync ~10 ms
    // Anti-windup
    float clamp = PLL_INT_MAX / PLL_KI;
    if (_error_integral > clamp) _error_integral = clamp;
    else if (_error_integral < -clamp) _error_integral = -clamp;
}

void SyncVib::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    float dt_s = state.dt_ms * 0.001f;
    // Clamp dt: the first cycle after (re)enable can follow a config-apply stall
    // with a large gap; an unbounded dt would turn into a giant phase step.
    // (!(dt_s > 0) also rejects NaN/negative.)
    if (!(dt_s > 0.0f)) dt_s = 0.0f;
    else if (dt_s > 0.05f) dt_s = 0.05f;

    // PLL-corrected frequency (PLL terms are zero in phase 1 if on_sync isn't called)
    float f_adj = _fundamental_hz + PLL_KP * _phase_error + PLL_KI * _error_integral;
    if (!isfinite(f_adj) || f_adj < 0.0f) f_adj = 0.0f;

    // Advance phase, wrap to [0, 2*pi). fmodf is O(1) and cannot spin — the old
    // iterative subtraction looped forever on a non-finite or large _phase.
    _phase += 2.0f * (float)M_PI * f_adj * dt_s;
    if (!isfinite(_phase)) _phase = 0.0f;
    _phase = fmodf(_phase, 2.0f * (float)M_PI);
    if (_phase < 0.0f) _phase += 2.0f * (float)M_PI;

    // Smooth amplitudes (first-order LPF, tau = 50 ms)
    float a = dt_s / (0.05f + dt_s);
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
        _amp[i] += (_target[i] - _amp[i]) * a;
    }

    // Sum all active slots — output is a position delta (mm) on the servo
    // command path, applied in Main.cpp after calc_final_position. Never
    // enters the integrator; damping changes do not attenuate amplitude.
    float f = 0.0f;
    for (uint8_t i = 0; i < _num_slots; i++) {
        f += _amp[i] * fastmath::fast_sinf(_ratios[i] * _phase + _phase_offset);
    }

    accum.x_vib += f;
}

float SyncVib::wrap_pm_pi(float x) {
    if (!isfinite(x)) return 0.0f;
    x = fmodf(x + (float)M_PI, 2.0f * (float)M_PI);
    if (x < 0.0f) x += 2.0f * (float)M_PI;
    return x - (float)M_PI;
}

void SyncVib::reset(void) {
    _phase = 0.0f;
    _fundamental_hz = 0.0f;
    _phase_error = 0.0f;
    _error_integral = 0.0f;
    // Ramp amplitudes back in from silence; ratios/phase_offset (config) persist.
    for (uint8_t i = 0; i < MAX_SLOTS; i++) _amp[i] = 0.0f;
}
