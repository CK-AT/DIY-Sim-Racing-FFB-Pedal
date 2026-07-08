#include <math.h>

#include <unity.h>

#include "KinematicPoly.h"

using kinematic_poly::calc_poly_d;
using kinematic_poly::horner5_f;

namespace {

constexpr size_t POLY_N = 5;

// Default coefficients from ConfigManager::set_axis_config_defaults().
const double kDefaultForce[POLY_N] = {
    6.20184902e-01,
    -1.71372506e-03,
    1.07828479e-05,
    2.71382634e-09,
    7.34203389e-11,
};
const double kDefaultSled[POLY_N] = {
    5.50622588e+01,
    5.55488175e-01,
    7.59065219e-04,
    -2.81513616e-06,
    -1.11220289e-08,
};

// Range from defaults: contact_point_pos_min_abs = -1200, max_abs = 900 in
// 0.1mm units. Hot path uses mm, so divide by 10.
constexpr float kDefaultXMin = -120.0f;
constexpr float kDefaultXMax = 90.0f;

// Range from the largest axis in full_default_config.json: ±103 mm. This is
// the worst-case input magnitude analysed in the plan.
constexpr float kWorstXAbs = 103.0f;

void copy_to_float(const double *src, float *dst, size_t n) {
    for (size_t i = 0; i < n; i++) dst[i] = static_cast<float>(src[i]);
}

void sweep(
    float x_min,
    float x_max,
    int n_points,
    const double *coeffs_d,
    const float *coeffs_f,
    float &max_err
) {
    max_err = 0.0f;
    for (int i = 0; i < n_points; i++) {
        float t = float(i) / float(n_points - 1);
        float x = x_min + t * (x_max - x_min);
        float yd = calc_poly_d(x, coeffs_d, POLY_N);
        float yf = horner5_f(x, coeffs_f);
        float e = fabsf(yd - yf);
        if (e > max_err) max_err = e;
    }
}

}  // namespace

void setUp(void) {
}

void tearDown(void) {
}

// Equivalence sweep across the default-config range for the force-factor
// polynomial.
void test_force_factor_equivalence(void) {
    float coeffs_f[POLY_N];
    copy_to_float(kDefaultForce, coeffs_f, POLY_N);
    float max_err = 0.0f;
    sweep(kDefaultXMin, kDefaultXMax, 33, kDefaultForce, coeffs_f, max_err);
    // Plan threshold for force factor: < 1e-5 (unitless).
    TEST_ASSERT_LESS_THAN_FLOAT(1e-5f, max_err);
}

// Equivalence sweep across the default-config range for the sled-position
// polynomial.
void test_sled_position_equivalence(void) {
    float coeffs_f[POLY_N];
    copy_to_float(kDefaultSled, coeffs_f, POLY_N);
    float max_err = 0.0f;
    sweep(kDefaultXMin, kDefaultXMax, 33, kDefaultSled, coeffs_f, max_err);
    // Plan threshold for sled position: < 1e-3 mm.
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, max_err);
}

// Equivalence at the worst-case magnitude analysed in the plan (|x| = 103 mm
// across all five real axes from full_default_config.json).
void test_worst_case_magnitude(void) {
    float coeffs_f_force[POLY_N];
    float coeffs_f_sled[POLY_N];
    copy_to_float(kDefaultForce, coeffs_f_force, POLY_N);
    copy_to_float(kDefaultSled, coeffs_f_sled, POLY_N);

    float max_force_err = 0.0f;
    sweep(-kWorstXAbs, kWorstXAbs, 33, kDefaultForce, coeffs_f_force, max_force_err);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-5f, max_force_err);

    float max_sled_err = 0.0f;
    sweep(-kWorstXAbs, kWorstXAbs, 33, kDefaultSled, coeffs_f_sled, max_sled_err);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, max_sled_err);
}

// Sanity at zero: horner5_f(0, c) returns c[0] cast to float.
void test_horner_at_zero(void) {
    float coeffs_f[POLY_N];
    copy_to_float(kDefaultSled, coeffs_f, POLY_N);
    float y = horner5_f(0.0f, coeffs_f);
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(kDefaultSled[0]), y);
}

// Reference vector: hand-computed (x, expected) using the default sled
// polynomial. Computed in double to ~10 sig figs:
//   x = 50 mm
//   sled(50) = 55.0622588
//            + 0.555488175 * 50
//            + 7.59065219e-4 * 2500
//            - 2.81513616e-6 * 125000
//            - 1.11220289e-8 * 6250000
//          = 84.313162
void test_reference_vector_sled(void) {
    float coeffs_f[POLY_N];
    copy_to_float(kDefaultSled, coeffs_f, POLY_N);
    constexpr float x = 50.0f;
    constexpr float expected = 84.313162f;
    float y = horner5_f(x, coeffs_f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, y);
}

// A pathological coefficient set that forces float to lose accuracy. c[4]
// is large enough that c[4]*x^4 at |x|=100 carries an error well above the
// 1e-3 mm sled threshold. Confirms the self-check threshold can actually
// trip — sanity check on the fallback gate.
void test_pathological_coefficients_diverge(void) {
    const double bad_sled[POLY_N] = {0.0, 0.0, 0.0, 0.0, 1e-3};  // c4 = 1e-3
    float coeffs_f[POLY_N];
    copy_to_float(bad_sled, coeffs_f, POLY_N);
    float max_err = 0.0f;
    sweep(-100.0f, 100.0f, 33, bad_sled, coeffs_f, max_err);
    // At |x|=100, x^4 = 1e8 — float ULP at that magnitude is ~16, so
    // c4 * x^4 carries an absolute error of ~1e-3 * 16 = 1.6e-2. Well
    // above the 1e-3 mm threshold the self-check uses.
    TEST_ASSERT_GREATER_THAN_FLOAT(1e-3f, max_err);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_force_factor_equivalence);
    RUN_TEST(test_sled_position_equivalence);
    RUN_TEST(test_worst_case_magnitude);
    RUN_TEST(test_horner_at_zero);
    RUN_TEST(test_reference_vector_sled);
    RUN_TEST(test_pathological_coefficients_diverge);
    return UNITY_END();
}
