#pragma once

#include "diy_ffb_protocol.pb.h"
#include "Physics.h"

struct StaticBalanceResultData {
    uint16_t count = 0;
    float x_min = 0.0f;
    float x_max = 0.0f;
    float step = 0.0f;
    const float *samples = nullptr;
};

class StaticBalancer : public SimElement {
    public:
        StaticBalancer() {
            disable();
        }
        void set_poly(const float *coeffs, uint16_t count, float x_center, float x_half_range) {
            _coeffs = coeffs;
            _count = count;
            _x_center = x_center;
            _x_half_range = x_half_range;
        }
        void update_config(const AxisConfig_StaticBalanceConfig *axis_cfg,
                           const FunctionConfig_StaticBalanceTuning *tuning);
        void update(const SimState &state, SimAccumulators &accum) override;
        void set_tuning(bool enabled, float gain) {
            _tuning_enabled = enabled;
            _gain = gain;
        }
        float get_force(float x_mm, float base_force);
        void start_calibration(float x_min, float x_max, float step_mm = 1.0f, uint32_t settle_ms = 30);
        bool calibration_done(StaticBalanceResultData &result);
        bool is_calibrating(void) const {
            return _calibration.state != CalState::Idle;
        }

    private:
        enum class CalState : uint8_t {
            Idle,
            Moving,
            Settling,
            Sampling,
            Complete
        };
        const float *_coeffs = nullptr;
        uint16_t _count = 0;
        float _x_center = 0.0f;
        float _x_half_range = 0.0f;
        bool _tuning_enabled = false;
        float _gain = 1.0f;
        struct CalibrationState {
            CalState state = CalState::Idle;
            uint16_t sample_count = 0;
            uint16_t sample_index = 0;
            float x_min = 0.0f;
            float x_max = 0.0f;
            float step = 0.0f;
            float target_x = 0.0f;
            float base_force = 0.0f;
            uint32_t last_step_ms = 0;
            uint32_t settle_ms = 30;
            float prev_x_min = 0.0f;
            float prev_x_max = 0.0f;
            static constexpr uint16_t k_max_samples = 32;
            float samples[k_max_samples] = {};
        } _calibration = {};
};
