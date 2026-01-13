#include <algorithm>
#include <cstdio>
#include <cmath>

#include <unity.h>

#include "Physics.h"

namespace {
void SetSimPosition(Sim &sim, float position) {
    float original_min = sim.get_x_min();
    float original_max = sim.get_x_max();
    sim.set_x_min(position, true);
    sim.set_x_max(position, true);
    float dt = 1.0f / 16.0f;
    float f_in = 0.0f;
    sim.update(dt, f_in, true);
    sim.update(dt, f_in, true);
    sim.set_x_min(original_min, true);
    sim.set_x_max(original_max, true);
}

float ComputeEnergy(Sim &sim, float k, float offset) {
    float x = sim.get_x();
    float v = sim.get_v() / 1000.0f;
    float dx = x - offset;
    float kinetic = 0.5f * sim.get_m() * v * v;
    float potential = 0.5f * k * dx * dx / 1000.0f;
    return kinetic + potential;
}

void StepSemiImplicit(Sim &sim, float &x, float &v, float dt_ms, float f_in) {
    sim.set_dt_ms(dt_ms);
    sim.set_state(x, x - (v * dt_ms / 1000.0f));
    float f_sum = sim.compute_force_sum(f_in);
    float a = f_sum / sim.get_m() * 1000.0f;
    v += a * dt_ms / 1000.0f;
    x += v * dt_ms / 1000.0f;
    sim.set_state(x, x - (v * dt_ms / 1000.0f));
}

void StepVelocityVerlet(Sim &sim, float &x, float &v, float dt_ms, float f_in) {
    sim.set_dt_ms(dt_ms);
    sim.set_state(x, x - (v * dt_ms / 1000.0f));
    float f_sum = sim.compute_force_sum(f_in);
    float a = f_sum / sim.get_m() * 1000.0f;
    float dt_s = dt_ms / 1000.0f;
    x += v * dt_s + 0.5f * a * dt_s * dt_s;
    sim.set_state(x, x - (v * dt_ms / 1000.0f));
    float f_sum_next = sim.compute_force_sum(f_in);
    float a_next = f_sum_next / sim.get_m() * 1000.0f;
    v += 0.5f * (a + a_next) * dt_s;
    sim.set_state(x, x - (v * dt_ms / 1000.0f));
}
}  // namespace

void setUp(void) {
}

void tearDown(void) {
}

void test_energy_conservation_spring_only(void) {
    constexpr float k = 2.0f;
    constexpr float m = 0.5f;
    constexpr float offset = 0.0f;
    Sim sim(m, -100.0f, 100.0f);
    Spring spring(offset, k);
    sim.add_element(&spring);
    SetSimPosition(sim, 20.0f);

    float dt = 1.0f;
    float f_in = 0.0f;
    float baseline = 0.0f;
    float max_drift = 0.0f;

    for (int step = 0; step < 20000; ++step) {
        sim.update(dt, f_in);
        if (step == 10) {
            baseline = ComputeEnergy(sim, k, offset);
        } else if (step > 10) {
            float e = ComputeEnergy(sim, k, offset);
            float drift = fabsf(e - baseline) / baseline;
            if (drift > max_drift) {
                max_drift = drift;
            }
        }
    }

    char msg[128];
    std::snprintf(msg, sizeof(msg), "max_drift=%.4f", max_drift);
    TEST_ASSERT_TRUE_MESSAGE(max_drift < 0.25f, msg);
}

void test_damper_clamped_to_stable_limit(void) {
    constexpr float m = 0.05f;
    float dt = 1.0f;
    constexpr float k = 10.0f;
    Sim sim(m, -100.0f, 100.0f);
    Damper damper(k);
    sim.add_element(&damper);
    SetSimPosition(sim, 0.0f);

    float f_in = 10.0f;
    for (int step = 0; step < 5; ++step) {
        sim.update(dt, f_in);
    }

    f_in = 0.0f;
    sim.update(dt, f_in);

    float v = sim.get_v();
    TEST_ASSERT_TRUE(std::isfinite(v));
    if (fabsf(v) > 1e-3f) {
        float k_effective = fabsf(sim.get_f_sum() / v);
        float k_limit = 2.0f * m / dt;
        TEST_ASSERT_TRUE(k_effective <= k_limit * 1.02f);
    }
}

void test_integrator_comparison_spring_only(void) {
    constexpr float k = 2.0f;
    constexpr float m = 0.5f;
    constexpr float offset = 0.0f;
    constexpr float dt = 1.0f / 1000.0f;
    Sim sim_verlet(m, -100.0f, 100.0f);
    Sim sim_semi(m, -100.0f, 100.0f);
    Sim sim_vv(m, -100.0f, 100.0f);
    Spring spring(offset, k);
    sim_verlet.add_element(&spring);
    sim_semi.add_element(&spring);
    sim_vv.add_element(&spring);

    SetSimPosition(sim_verlet, 20.0f);
    SetSimPosition(sim_semi, 20.0f);
    SetSimPosition(sim_vv, 20.0f);

    float x = sim_semi.get_x();
    float v = 0.0f;
    float x_vv = sim_vv.get_x();
    float v_vv = 0.0f;
    float f_in = 0.0f;

    float drift_verlet = 0.0f;
    float drift_semi = 0.0f;
    float drift_vv = 0.0f;
    float baseline_verlet = 0.0f;
    float baseline_semi = 0.0f;
    float baseline_vv = 0.0f;

    for (int step = 0; step < 20000000; ++step) {
        float dt_local = dt;
        sim_verlet.update(dt_local, f_in);
        StepSemiImplicit(sim_semi, x, v, dt, f_in);
        StepVelocityVerlet(sim_vv, x_vv, v_vv, dt, f_in);

        if (step == 10) {
            baseline_verlet = ComputeEnergy(sim_verlet, k, offset);
            baseline_semi = ComputeEnergy(sim_semi, k, offset);
            baseline_vv = ComputeEnergy(sim_vv, k, offset);
        } else if (step > 10) {
            float e_verlet = ComputeEnergy(sim_verlet, k, offset);
            float e_semi = ComputeEnergy(sim_semi, k, offset);
            float e_vv = ComputeEnergy(sim_vv, k, offset);
            drift_verlet = std::max(drift_verlet, fabsf(e_verlet - baseline_verlet) / baseline_verlet);
            drift_semi = std::max(drift_semi, fabsf(e_semi - baseline_semi) / baseline_semi);
            drift_vv = std::max(drift_vv, fabsf(e_vv - baseline_vv) / baseline_vv);
        }
    }

    char msg[160];
    std::snprintf(msg, sizeof(msg), "drift_verlet=%.7f drift_semi=%.7f drift_vv=%.7f", drift_verlet, drift_semi, drift_vv);
    TEST_ASSERT_TRUE_MESSAGE(drift_verlet < 0.00000003f, msg);
}

void test_integrator_comparison_stiff_spring(void) {
    constexpr float k = 20.0f;
    constexpr float m = 0.5f;
    constexpr float offset = 0.0f;
    constexpr float dt = 1.0f / 1000.0f;
    Sim sim_verlet(m, -100.0f, 100.0f);
    Sim sim_semi(m, -100.0f, 100.0f);
    Sim sim_vv(m, -100.0f, 100.0f);
    Spring spring(offset, k);
    sim_verlet.add_element(&spring);
    sim_semi.add_element(&spring);
    sim_vv.add_element(&spring);

    SetSimPosition(sim_verlet, 80.0f);
    SetSimPosition(sim_semi, 80.0f);
    SetSimPosition(sim_vv, 80.0f);

    float x = sim_semi.get_x();
    float v = 0.0f;
    float x_vv = sim_vv.get_x();
    float v_vv = 0.0f;
    float f_in = 0.0f;

    float drift_verlet = 0.0f;
    float drift_semi = 0.0f;
    float drift_vv = 0.0f;
    float baseline_verlet = 0.0f;
    float baseline_semi = 0.0f;
    float baseline_vv = 0.0f;

    for (int step = 0; step < 20000000; ++step) {
        float dt_local = dt;
        sim_verlet.update(dt_local, f_in);
        StepSemiImplicit(sim_semi, x, v, dt, f_in);
        StepVelocityVerlet(sim_vv, x_vv, v_vv, dt, f_in);

        if (step == 10) {
            baseline_verlet = ComputeEnergy(sim_verlet, k, offset);
            baseline_semi = ComputeEnergy(sim_semi, k, offset);
            baseline_vv = ComputeEnergy(sim_vv, k, offset);
        } else if (step > 10) {
            float e_verlet = ComputeEnergy(sim_verlet, k, offset);
            float e_semi = ComputeEnergy(sim_semi, k, offset);
            float e_vv = ComputeEnergy(sim_vv, k, offset);
            drift_verlet = std::max(drift_verlet, fabsf(e_verlet - baseline_verlet) / baseline_verlet);
            drift_semi = std::max(drift_semi, fabsf(e_semi - baseline_semi) / baseline_semi);
            drift_vv = std::max(drift_vv, fabsf(e_vv - baseline_vv) / baseline_vv);
        }
    }

    char msg[160];
    std::snprintf(msg, sizeof(msg), "stiff drift_verlet=%.7f drift_semi=%.7f drift_vv=%.7f", drift_verlet, drift_semi, drift_vv);
    TEST_ASSERT_TRUE_MESSAGE(drift_verlet < 0.0000001f, msg);
}

void test_friction_no_input_no_drift(void) {
    Sim sim(1.0f, -100.0f, 100.0f);
    Friction friction(2.0f);
    sim.add_element(&friction);
    SetSimPosition(sim, 0.0f);

    float dt = 1.0f;
    float f_in = 0.0f;
    for (int step = 0; step < 1000; ++step) {
        sim.update(dt, f_in);
    }

    TEST_ASSERT_TRUE(fabsf(sim.get_x()) < 0.001f);
    TEST_ASSERT_TRUE(fabsf(sim.get_v()) < 0.01f);
}

void test_friction_static_holds_under_threshold(void) {
    Sim sim(1.0f, -100.0f, 100.0f);
    Friction friction(2.0f);
    sim.add_element(&friction);
    SetSimPosition(sim, 0.0f);

    float dt = 1.0f;
    float f_in = 1.0f;
    for (int step = 0; step < 1000; ++step) {
        sim.update(dt, f_in);
    }

    float x = sim.get_x();
    float v = sim.get_v();
    char msg[128];
    std::snprintf(msg, sizeof(msg), "x=%.4f v=%.4f", x, v);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(x) < 0.01f, msg);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(v) < 0.05f, msg);
}

void test_friction_kinetic_moves_above_threshold(void) {
    Sim sim(1.0f, -100.0f, 100.0f);
    Friction friction(2.0f);
    sim.add_element(&friction);
    SetSimPosition(sim, 0.0f);

    float dt = 1.0f;
    float f_in = 2.2f;
    for (int step = 0; step < 500; ++step) {
        sim.update(dt, f_in);
    }

    TEST_ASSERT_TRUE(sim.get_x() > 0.5f);
}

void test_damper_and_friction_settle_velocity(void) {
    Sim sim(1.0f, -100.0f, 100.0f);
    Damper damper(0.5f);
    Friction friction(1.0f);
    sim.add_element(&damper);
    sim.add_element(&friction);
    SetSimPosition(sim, 0.0f);

    float dt = 1.0f;
    float f_in = 5.0f;
    for (int step = 0; step < 150; ++step) {
        sim.update(dt, f_in);
    }

    f_in = 0.0f;
    for (int step = 0; step < 300; ++step) {
        sim.update(dt, f_in);
    }

    TEST_ASSERT_TRUE(fabsf(sim.get_v()) < 5.0f);
}

void test_integrator_comparison_stiff_damped_friction(void) {
    constexpr float k = 50.0f;
    constexpr float m = 0.5f;
    constexpr float offset = 0.0f;
    constexpr float dt = 1.0f / 1000.0f;
    Sim sim_verlet(m, -100.0f, 100.0f);
    Sim sim_semi(m, -100.0f, 100.0f);
    Sim sim_vv(m, -100.0f, 100.0f);
    Spring spring(offset, k);
    Damper damper(1.0f);
    Friction friction(1.0f);
    sim_verlet.add_element(&spring);
    sim_verlet.add_element(&damper);
    sim_verlet.add_element(&friction);
    sim_semi.add_element(&spring);
    sim_semi.add_element(&damper);
    sim_semi.add_element(&friction);
    sim_vv.add_element(&spring);
    sim_vv.add_element(&damper);
    sim_vv.add_element(&friction);

    SetSimPosition(sim_verlet, 80.0f);
    SetSimPosition(sim_semi, 80.0f);
    SetSimPosition(sim_vv, 80.0f);

    float x = sim_semi.get_x();
    float v = 0.0f;
    float x_vv = sim_vv.get_x();
    float v_vv = 0.0f;
    float f_in = 0.0f;

    float final_x_verlet = 0.0f;
    float final_v_verlet = 0.0f;
    float final_x_semi = 0.0f;
    float final_v_semi = 0.0f;
    float final_x_vv = 0.0f;
    float final_v_vv = 0.0f;

    for (int step = 0; step < 20000000; ++step) {
        float dt_local = dt;
        sim_verlet.update(dt_local, f_in);
        StepSemiImplicit(sim_semi, x, v, dt, f_in);
        StepVelocityVerlet(sim_vv, x_vv, v_vv, dt, f_in);
    }

    final_x_verlet = sim_verlet.get_x();
    final_v_verlet = sim_verlet.get_v();
    final_x_semi = x;
    final_v_semi = v * 1000.0f;
    final_x_vv = x_vv;
    final_v_vv = v_vv * 1000.0f;

    char msg[200];
    std::snprintf(msg, sizeof(msg),
                  "damped stiff: verlet x=%.2f v=%.2f semi x=%.2f v=%.2f vv x=%.2f v=%.2f",
                  final_x_verlet, final_v_verlet, final_x_semi, final_v_semi, final_x_vv, final_v_vv);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(final_x_verlet) < 5.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(final_x_semi) < 5.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(final_x_vv) < 5.0f, msg);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_energy_conservation_spring_only);
    RUN_TEST(test_damper_clamped_to_stable_limit);
    RUN_TEST(test_integrator_comparison_spring_only);
    RUN_TEST(test_integrator_comparison_stiff_spring);
    RUN_TEST(test_friction_no_input_no_drift);
    RUN_TEST(test_friction_static_holds_under_threshold);
    RUN_TEST(test_friction_kinetic_moves_above_threshold);
    RUN_TEST(test_damper_and_friction_settle_velocity);
    // RUN_TEST(test_integrator_comparison_stiff_damped_friction);
    return UNITY_END();
}
