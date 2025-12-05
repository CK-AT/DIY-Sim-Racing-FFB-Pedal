#pragma once

#include <Kalman.h>

static constexpr int k_n_obs = 1;    // 1 filter input:   observed value
static constexpr int k_n_state = 2;  // 2 filter outputs: change & velocity
static constexpr int k_n_com = 1;    // Number of commands, u vector

class KalmanFilter {
    private:
        KALMAN<k_n_state, k_n_obs, k_n_com> _kalman;
        unsigned long _time_last_observation;

    public:
        KalmanFilter(float variance_estimate);

        float filtered_value(float observation, float command, uint8_t model_noise_scaling_u8);
        float change_velocity();
};
