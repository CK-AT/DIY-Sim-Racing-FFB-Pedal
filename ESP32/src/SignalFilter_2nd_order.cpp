#include "SignalFilter_2nd_order.h"

// v = s / t
// a = v/t
// a = s / t^2
// a = 300 / delta_t^2
// adjust model noise here s = 1/6 * j * delta_t^3 --> j = 6 * s / delta_t^3
// static const float KF_MODEL_NOISE_FORCE_ACCELERATION = ( 2.0f * 1000.0f / 0.05f/ 0.05f );
static const float KF_MODEL_NOISE_FORCE_JERK = 2000 * (2.0f * 4.0f / 0.1f / 0.1f);

// static const float KF_MODEL_NOISE_FORCE_ACCELERATION = 180 * 1e6;//( 2.0f * 4.0f / 0.1f/ 0.1f );

KalmanFilterSecondOrder::KalmanFilterSecondOrder(float variance_estimate) : _time_last_observation(micros()) {
    // evolution matrix. Size is <Nstate,Nstate>
    _kalman.F = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    // command matrix.  Size is <Nstate,Ncom>
    _kalman.B = {1.0, 0.0, 0.0};

    // measurement matrix. Size is <Nobs,Nstate>
    _kalman.H = {1.0, 0.0, 0.0};

    // model covariance matrix. Size is <Nstate,Nstate>
    _kalman.Q = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    // measurement covariance matrix. Size is <Nobs,Nobs>
    _kalman.R = {variance_estimate};
}

float KalmanFilterSecondOrder::filtered_value(float observation, float command, uint8_t model_noise_scaling_u8) {
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
    float delta_t = (float)elapsed_time / 1000000.0f;  // convert to seconds
    float delta_t_pow2 = delta_t * delta_t;
    float delta_t_pow3 = delta_t_pow2 * delta_t;
    float delta_t_pow4 = delta_t_pow2 * delta_t_pow2;

    _kalman.F = {1.0, delta_t, 0.5f * delta_t * delta_t, 0.0, 1.0, delta_t, 0.0, 0.0, 1.0};

    _kalman.B = {1.0, 0.0, 0.0};

    float q11 = KF_MODEL_NOISE_FORCE_JERK * (1. / 6. * delta_t * delta_t * delta_t) * (1. / 6. * delta_t * delta_t * delta_t);
    float q12 = KF_MODEL_NOISE_FORCE_JERK * (1. / 6. * delta_t * delta_t * delta_t) * (1. / 2. * delta_t * delta_t);
    float q13 = KF_MODEL_NOISE_FORCE_JERK * (1. / 6. * delta_t * delta_t * delta_t) * (delta_t);

    float q21 = q12;
    float q22 = KF_MODEL_NOISE_FORCE_JERK * (1. / 2. * delta_t * delta_t) * (1. / 2. * delta_t * delta_t);
    float q23 = KF_MODEL_NOISE_FORCE_JERK * (1. / 2. * delta_t * delta_t) * (delta_t);

    float q31 = q13;
    float q32 = q23;
    float q33 = KF_MODEL_NOISE_FORCE_JERK * (delta_t) * (delta_t);

    _kalman.Q = {q11, q12, q13, q21, q22, q23, q31, q32, q33};

    // APPLY KALMAN FILTER
    _kalman.update({observation}, {command});
    return _kalman.x(0, 0);
}

float KalmanFilterSecondOrder::change_velocity() {
    return _kalman.x(0, 1);
}

float KalmanFilterSecondOrder::change_accel() {
    return _kalman.x(0, 2);
}
