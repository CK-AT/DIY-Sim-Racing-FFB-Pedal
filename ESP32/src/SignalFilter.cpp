#include "SignalFilter.h"

// v = s / t
// a = v/t
// a = s / t^2
// a = 300 / delta_t^2
// adjust model noise here s = 0.5 * a * delta_t^2 --> a = 2 * s / delta_t^2
// static const float KF_MODEL_NOISE_FORCE_ACCELERATION = ( 2.0f * 1000.0f / 0.05f/ 0.05f );
static const float KF_MODEL_NOISE_FORCE_ACCELERATION = (8.0f * 4.0f / 0.1f / 0.1f);

KalmanFilter::KalmanFilter(float variance_estimate) : _time_last_observation(micros()) {
    // evolution matrix. Size is <Nstate,Nstate>
    _kalman.F = {(float)1.0, 0.0, 0.0, (float)1.0};

    // command matrix.  Size is <Nstate,Ncom>
    _kalman.B = {1.0, 0.0};

    // measurement matrix. Size is <Nobs,Nstate>
    _kalman.H = {1.0, 0.0};

    // model covariance matrix. Size is <Nstate,Nstate>
    _kalman.Q = {1.0, 0.0, 0.0, 1.0};

    // measurement covariance matrix. Size is <Nobs,Nobs>
    _kalman.R = {variance_estimate};
}

float KalmanFilter::filtered_value(float observation, float command, uint8_t model_noise_scaling_u8) {
    // obtain time
    unsigned long current_time = micros();
    unsigned long elapsed_time = current_time - _time_last_observation;
    float model_noise_scaling = model_noise_scaling_u8;
    model_noise_scaling /= 255.0;

    if (model_noise_scaling < 0.001) {
        model_noise_scaling = 0.001;
    }
    if (elapsed_time < 1) {
        elapsed_time = 1;
    }
    _time_last_observation = current_time;

    // update state transition and system covariance matrices
    float delta_t = ((float)elapsed_time) / 1000000.0f;  /// 1000000.0f; // convert to seconds
    float delta_t_pow2 = delta_t * delta_t;
    float delta_t_pow3 = delta_t_pow2 * delta_t;
    float delta_t_pow4 = delta_t_pow2 * delta_t_pow2;

    _kalman.F = {(float)1.0, delta_t, 0.0, (float)1.0};

    _kalman.B = {1.0, 0.0};

    float k_q_11 = model_noise_scaling * KF_MODEL_NOISE_FORCE_ACCELERATION * (float)0.5f * delta_t_pow3;
    _kalman.Q = {model_noise_scaling * KF_MODEL_NOISE_FORCE_ACCELERATION * (float)0.25f * delta_t_pow4, k_q_11, k_q_11,
            model_noise_scaling * KF_MODEL_NOISE_FORCE_ACCELERATION * delta_t_pow2};

    // APPLY KALMAN FILTER
    _kalman.update({observation}, {command});
    return _kalman.x(0, 0);
}

float KalmanFilter::change_velocity() {
    return _kalman.x(0, 1) / 1.0f;
}
