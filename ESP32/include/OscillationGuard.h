#pragma once

#include "Physics.h"

class OscillationGuard : public SimElement {
    public:
        OscillationGuard(float k_max = 0.5f,
                         float min_amplitude = 0.2f,
                         float min_velocity = 0.5f,
                         uint32_t min_half_period_us = 5000,
                         uint32_t max_half_period_us = 250000,
                         uint32_t hold_time_us = 150000,
                         uint32_t ramp_time_us = 80000,
                         uint8_t required_hits = 2)
            : _k_max(k_max),
              _min_amplitude(min_amplitude),
              _min_velocity(min_velocity),
              _min_half_period_us(min_half_period_us),
              _max_half_period_us(max_half_period_us),
              _hold_time_us(hold_time_us),
              _ramp_time_us(ramp_time_us),
              _required_hits(required_hits) {}
        void update(const SimState &state, SimAccumulators &accum) override;
        void set_damping_gain(float k_max) {
            _k_max = max(k_max, 0.0f);
        }
        void set_detection(float min_amplitude, float min_velocity, uint32_t min_half_period_us, uint32_t max_half_period_us) {
            _min_amplitude = min_amplitude;
            _min_velocity = min_velocity;
            _min_half_period_us = min_half_period_us;
            _max_half_period_us = max_half_period_us;
        }
        void set_timing(uint32_t hold_time_us, uint32_t ramp_time_us) {
            _hold_time_us = hold_time_us;
            _ramp_time_us = ramp_time_us;
        }
        void set_required_hits(uint8_t required_hits) {
            _required_hits = required_hits < 1 ? 1 : required_hits;
        }

    private:
        float _k_max;
        float _min_amplitude;
        float _min_velocity;
        uint32_t _min_half_period_us;
        uint32_t _max_half_period_us;
        uint32_t _hold_time_us;
        uint32_t _ramp_time_us;
        uint8_t _required_hits;
        uint8_t _oscillation_hits = 0;
        int8_t _last_v_sign = 0;
        float _last_peak_x = 0.0f;
        float _damping_gain = 0.0f;
        uint32_t _last_flip_us = 0;
        uint32_t _last_oscillation_us = 0;
        uint32_t _last_update_us = 0;
};
