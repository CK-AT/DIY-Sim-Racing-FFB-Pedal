#include "StaticBalancer.h"

#include "Arduino.h"
#include "LogOutput.h"

void StaticBalancer::update_config(const AxisConfig_StaticBalanceConfig *axis_cfg,
                                   const FunctionConfig_StaticBalanceTuning *tuning) {
    const float *coeffs = nullptr;
    uint16_t coeffs_count = 0;
    float x_center = 0.0f;
    float x_half_range = 0.0f;
    if (axis_cfg) {
        coeffs = axis_cfg->coeffs;
        coeffs_count = static_cast<uint16_t>(sizeof(axis_cfg->coeffs) / sizeof(axis_cfg->coeffs[0]));
        x_center = axis_cfg->x_center;
        x_half_range = axis_cfg->x_half_range;
    }
    set_poly(coeffs, coeffs_count, x_center, x_half_range);

    bool enabled = false;
    float gain = 1.0f;
    if (tuning) {
        enabled = tuning->enabled;
        gain = tuning->gain;
    }
    set_tuning(enabled, gain);
}

void StaticBalancer::start_calibration(float x_min, float x_max, float step_mm, uint32_t settle_ms, CommChannel comm_channel) {
    if (_calibration.state != CalState::Idle && _calibration.state != CalState::Complete) {
        return;
    }
    if (x_max < x_min) {
        float tmp = x_min;
        x_min = x_max;
        x_max = tmp;
    }
    float range = x_max - x_min;
    if (range <= 0.0f) {
        LogOutput::printf("StaticBalancer: invalid range (%.3f..%.3f)", x_min, x_max);
        return;
    }
    if (step_mm <= 0.0f) {
        step_mm = range / static_cast<float>(CalibrationState::k_max_samples - 1);
    }
    uint16_t count = static_cast<uint16_t>((range) / step_mm) + 1;
    if (count > CalibrationState::k_max_samples) {
        count = CalibrationState::k_max_samples;
        if (count > 1) {
            step_mm = range / static_cast<float>(count - 1);
        }
    }
    _calibration.sample_count = count;
    _calibration.sample_index = 0;
    _calibration.sample_accum_count = 0;
    _calibration.x_min = x_min;
    _calibration.x_max = x_max;
    _calibration.step = step_mm;
    _calibration.target_x = x_min;
    _calibration.last_step_ms = 0;
    _calibration.settle_ms = settle_ms;
    _calibration.prev_x_min = x_min;
    _calibration.prev_x_max = x_max;
    _calibration.sample_accum = 0.0f;
    _calibration.comm_channel = comm_channel;
    _calibration.state = CalState::Moving;
    enable();
    LogOutput::printf("StaticBalancer: start %.3f..%.3f step=%.3f count=%u settle=%lu",
                      x_min, x_max, step_mm, static_cast<unsigned>(count), static_cast<unsigned long>(settle_ms));
}

bool StaticBalancer::calibration_done(StaticBalanceResultData &result) {
    if (_calibration.state != CalState::Complete) {
        return false;
    }
    result.count = _calibration.sample_count;
    result.x_min = _calibration.x_min;
    result.x_max = _calibration.x_max;
    result.step = _calibration.step;
    result.samples = _calibration.samples;
    result.comm_channel = _calibration.comm_channel;
    _calibration.state = CalState::Idle;
    return true;
}

void StaticBalancer::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    if (_calibration.state == CalState::Idle || _calibration.state == CalState::Complete) return;

    accum.set_limits(_calibration.target_x, _calibration.target_x);

    uint32_t now_ms = millis();
    const float k_reach_eps_mm = 0.05f;
    if (_calibration.state == CalState::Moving) {
        if (fabsf(state.x - _calibration.target_x) > k_reach_eps_mm) {
            return;
        }
        _calibration.last_step_ms = now_ms;
        _calibration.state = CalState::Settling;
        return;
    }
    if (_calibration.state == CalState::Settling) {
        if ((now_ms - _calibration.last_step_ms) < _calibration.settle_ms) {
            return;
        }
        _calibration.state = CalState::Sampling;
    }

    if (_calibration.sample_index < _calibration.sample_count) {
        _calibration.sample_accum += -_calibration.base_force;
        _calibration.sample_accum_count++;
        if (_calibration.sample_accum_count < CalibrationState::k_samples_per_step) {
            return;
        }
        _calibration.samples[_calibration.sample_index] =
            _calibration.sample_accum / static_cast<float>(_calibration.sample_accum_count);
        _calibration.sample_index++;
        _calibration.sample_accum = 0.0f;
        _calibration.sample_accum_count = 0;
    }

    _calibration.target_x = _calibration.x_min + (_calibration.step * static_cast<float>(_calibration.sample_index));
    _calibration.last_step_ms = now_ms;

    if (_calibration.sample_index >= _calibration.sample_count) {
        accum.set_limits(_calibration.prev_x_min, _calibration.prev_x_max);
        _calibration.state = CalState::Complete;
        disable();
        LogOutput::printf("StaticBalancer: complete count=%u", static_cast<unsigned>(_calibration.sample_count));
        return;
    }

    _calibration.state = CalState::Moving;
}

float StaticBalancer::get_force(float x_mm, float base_force) {
    if (_calibration.state != CalState::Idle && _calibration.state != CalState::Complete) {
        _calibration.base_force = base_force;
        return base_force;
    }
    if (!_tuning_enabled) {
        return base_force;
    }
    if (_count == 0 || _coeffs == nullptr || _x_half_range <= 0.0f) {
        return base_force;
    }
    float x_norm = (x_mm - _x_center) / _x_half_range;
    if (x_norm < -1.0f) {
        x_norm = -1.0f;
    } else if (x_norm > 1.0f) {
        x_norm = 1.0f;
    }

    float f_offset = _coeffs[_count - 1];
    for (int32_t idx = static_cast<int32_t>(_count) - 2; idx >= 0; --idx) {
        f_offset = (f_offset * x_norm) + _coeffs[idx];
    }
    return base_force + (_gain * f_offset);
}
