#include "OscillationGuard.h"

#include "Arduino.h"

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
