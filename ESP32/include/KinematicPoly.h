#pragma once

#include <stddef.h>
#include <stdint.h>

// Polynomial evaluators for the kinematic conversion polynomials. The wire
// format keeps double for offline-fit fidelity, but on-device evaluation
// runs in float on the ESP32's single-precision FPU. ConfigManager caches
// float copies of the coefficients at config-update time and uses
// horner5_f() in the FFB hot path. The double-precision version is kept
// for the self-check at config commit and for the fallback path on the
// rare axis whose coefficients fail the float-accuracy check.

namespace kinematic_poly {

// Evaluates c[0] + c[1]*x + c[2]*x^2 + ... + c[N-1]*x^(N-1) in double
// precision and casts the result to float. Used at config-update time
// (self-check) and as the fallback hot path.
inline float calc_poly_d(float in, const double *coeffs, size_t num_coeffs) {
    double result = coeffs[0];
    double temp = in;
    for (uint8_t i = 1; i < num_coeffs; i++) {
        result += temp * coeffs[i];
        temp *= in;
    }
    return static_cast<float>(result);
}

// Single-precision Horner-form evaluation of a degree-4 polynomial:
//   c[0] + c[1]*x + c[2]*x^2 + c[3]*x^3 + c[4]*x^4
// = ((((c[4]*x + c[3])*x + c[2])*x + c[1])*x + c[0])
inline float horner5_f(float x, const float *c) {
    float r = c[4];
    r = r * x + c[3];
    r = r * x + c[2];
    r = r * x + c[1];
    r = r * x + c[0];
    return r;
}

}  // namespace kinematic_poly
