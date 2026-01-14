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

void OscillationGuard::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    uint32_t now = micros();
    float v = state.v;
    float x = state.x;

    int8_t v_sign = 0;
    if (v > _min_velocity) {
        v_sign = 1;
    } else if (v < -_min_velocity) {
        v_sign = -1;
    }

    if ((v_sign != 0) && (_last_v_sign != 0) && (v_sign != _last_v_sign)) {
        if (_last_flip_us != 0) {
            uint32_t half_period_us = now - _last_flip_us;
            float amplitude = fabsf(x - _last_peak_x);
            if ((half_period_us >= _min_half_period_us) && (half_period_us <= _max_half_period_us) &&
                (amplitude >= _min_amplitude)) {
                if (_oscillation_hits < _required_hits) {
                    _oscillation_hits++;
                }
                if (_oscillation_hits >= _required_hits) {
                    _last_oscillation_us = now;
                }
            } else {
                _oscillation_hits = 0;
            }
        }
        _last_peak_x = x;
        _last_flip_us = now;
    }

    if (v_sign != 0) {
        _last_v_sign = v_sign;
    }

    bool oscillating = _last_oscillation_us && ((now - _last_oscillation_us) <= _hold_time_us);
    float target = oscillating ? _k_max : 0.0f;

    if (_last_update_us == 0 || _ramp_time_us == 0) {
        _damping_gain = target;
    } else {
        float step = (float)(now - _last_update_us) / (float)_ramp_time_us * _k_max;
        if (target > _damping_gain) {
            _damping_gain = min(_damping_gain + step, target);
        } else if (target < _damping_gain) {
            _damping_gain = max(_damping_gain - step, target);
        }
    }
    _last_update_us = now;

    if (_damping_gain > 0.0f) {
        accum.k_damp_sum += _damping_gain;
    }
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

    accum.f_sum += band_noise * _amplitude;
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

    float k_limit = 0.0f;
    if (dt > 0.0f && _m > 0.0f) {
        k_limit = 1.9f * _m / dt;
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

    _a = accum.f_sum / _m * 1000.0;
    float x_raw = (2.0 * _x) - _x_prev + (((_a * dt * dt) / 1000.0) / 1000.0);
    _x_prev = _x;
    _x = constrain(x_raw, _x_min, _x_max);
    _f_sum = accum.f_sum;
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
    return accum.f_sum;
}
#endif
