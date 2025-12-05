#pragma once

#include <Kalman.h>

static constexpr int k_n_obs_second_order = 1;    // 1 filter input:   observed value
static constexpr int k_n_state_second_order = 3;  // 2 filter outputs: change, velocity & acceleration
static constexpr int k_n_com_second_order = 1;    // Number of commands, u vector

class KalmanFilterSecondOrder {
    private:
        KALMAN<k_n_state_second_order, k_n_obs_second_order, k_n_com_second_order> _kalman;
        unsigned long _time_last_observation;

    public:
        KalmanFilterSecondOrder(float variance_estimate);

        float filtered_value(float observation, float command, uint8_t model_noise_scaling_u8);
        float change_velocity();
        float change_accel();
};
