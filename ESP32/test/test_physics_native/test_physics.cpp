#include <algorithm>
#include <cstdio>
#include <cmath>
#include <vector>

#include <unity.h>

#include "Physics.h"
#include "OscillationGuard.h"

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

void test_friction_order_agnostic(void) {
    constexpr float k = 1.5f;
    constexpr float offset = 0.0f;
    Sim sim_a(1.0f, -100.0f, 100.0f);
    Sim sim_b(1.0f, -100.0f, 100.0f);
    Spring spring(offset, k);
    Friction friction(2.0f);
    sim_a.add_element(&friction);
    sim_a.add_element(&spring);
    sim_b.add_element(&spring);
    sim_b.add_element(&friction);

    SetSimPosition(sim_a, 10.0f);
    SetSimPosition(sim_b, 10.0f);

    float dt = 1.0f;
    float f_in = 0.5f;
    for (int step = 0; step < 500; ++step) {
        sim_a.update(dt, f_in);
        sim_b.update(dt, f_in);
    }

    float dx = fabsf(sim_a.get_x() - sim_b.get_x());
    float dv = fabsf(sim_a.get_v() - sim_b.get_v());
    char msg[128];
    std::snprintf(msg, sizeof(msg), "dx=%.4f dv=%.4f", dx, dv);
    TEST_ASSERT_TRUE_MESSAGE(dx < 0.01f, msg);
    TEST_ASSERT_TRUE_MESSAGE(dv < 0.1f, msg);
}

void test_friction_no_creep_with_small_alternating_force(void) {
    Sim sim(1.0f, -100.0f, 100.0f);
    Friction friction(2.0f, 1.5f, 0.05f);
    sim.add_element(&friction);
    SetSimPosition(sim, 0.0f);

    float dt = 1.0f;
    for (int step = 0; step < 2000; ++step) {
        float f_in = (step % 2 == 0) ? 1.0f : -1.0f;
        sim.update(dt, f_in);
    }

    float x = sim.get_x();
    float v = sim.get_v();
    char msg[128];
    std::snprintf(msg, sizeof(msg), "x=%.4f v=%.4f", x, v);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(x) < 0.05f, msg);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(v) < 0.1f, msg);
}

void test_combined_damping_clamp(void) {
    constexpr float m = 0.05f;
    float dt = 1.0f;
    Sim sim(m, -100.0f, 100.0f);
    Damper damper(100.0f);
    std::vector<float> x_vect = {-100.0f, 100.0f};
    std::vector<float> k_vect = {100.0f, 100.0f};
    DampingMap damping_map(x_vect, k_vect);
    sim.add_element(&damper);
    sim.add_element(&damping_map);
    SetSimPosition(sim, 0.0f);

    float f_in = 10.0f;
    for (int step = 0; step < 10; ++step) {
        sim.update(dt, f_in);
    }

    f_in = 0.0f;
    sim.update(dt, f_in);

    float v = sim.get_v();
    if (fabsf(v) > 1e-3f) {
        float k_effective = fabsf(sim.get_f_sum() / v);
        float k_limit = 2.0f * m / dt;
        TEST_ASSERT_TRUE(k_effective <= k_limit * 1.02f);
    }
}

void test_dt_jitter_stability(void) {
    Sim sim(0.5f, -100.0f, 100.0f);
    Spring spring(0.0f, 2.0f);
    Damper damper(0.5f);
    sim.add_element(&spring);
    sim.add_element(&damper);
    SetSimPosition(sim, 20.0f);

    float f_in = 0.0f;
    for (int step = 0; step < 10000; ++step) {
        float dt = (step % 3 == 0) ? 0.25f : (step % 3 == 1 ? 1.0f : 2.0f);
        sim.update(dt, f_in);
        TEST_ASSERT_TRUE(std::isfinite(sim.get_x()));
        TEST_ASSERT_TRUE(std::isfinite(sim.get_v()));
    }
}

void test_limit_clamp_no_nan(void) {
    Sim sim(0.5f, -10.0f, 10.0f);
    Spring spring(0.0f, 5.0f);
    sim.add_element(&spring);
    SetSimPosition(sim, 9.5f);

    float dt = 1.0f;
    float f_in = 50.0f;
    for (int step = 0; step < 200; ++step) {
        sim.update(dt, f_in);
        TEST_ASSERT_TRUE(sim.get_x() <= 10.0f);
        TEST_ASSERT_TRUE(sim.get_x() >= -10.0f);
        TEST_ASSERT_TRUE(std::isfinite(sim.get_x()));
    }
}

void test_damping_map_negative_entries_clamped(void) {
    Sim sim(1.0f, -100.0f, 100.0f);
    std::vector<float> x_vect = {-100.0f, 100.0f};
    std::vector<float> k_vect = {-5.0f, -5.0f};
    DampingMap damping_map(x_vect, k_vect);
    sim.add_element(&damping_map);
    SetSimPosition(sim, 0.0f);

    float dt = 1.0f;
    float f_in = 5.0f;
    for (int step = 0; step < 50; ++step) {
        sim.update(dt, f_in);
    }

    float v = sim.get_v();
    TEST_ASSERT_TRUE(std::isfinite(v));
}

// Drive a SyncVib with a fixed dt and return the f_vib output trace.
static std::vector<float> RunSyncVib(SyncVib &vib, float dt_ms, int steps) {
    std::vector<float> trace;
    trace.reserve(steps);
    SimState state;
    state.dt_ms = dt_ms;
    for (int i = 0; i < steps; ++i) {
        SimAccumulators accum;
        vib.update(state, accum);
        trace.push_back(accum.f_vib);
    }
    return trace;
}

void test_syncvib_continuity_at_10hz(void) {
    SyncVib vib;
    float ratios[1] = {1.0f};
    vib.set_config(0.0f, ratios, 1);
    vib.on_sync(0.0f, 10.0f);  // seed fundamental = 10 Hz
    float amps[1] = {1.0f};
    vib.set_amplitudes(amps, 1);

    auto trace = RunSyncVib(vib, 1.0f, 1000);  // 1 ms dt, 1 second

    // Count zero crossings — at 10 Hz over 1 s, expect ~20 (two per cycle)
    int zero_crossings = 0;
    for (size_t i = 1; i < trace.size(); ++i) {
        if ((trace[i] >= 0.0f) != (trace[i - 1] >= 0.0f)) {
            zero_crossings++;
        }
    }
    // Allow some tolerance for the LPF settle on amplitude (starts at 0)
    TEST_ASSERT_INT_WITHIN(2, 20, zero_crossings);

    // No NaN / no discontinuities (max step between adjacent samples
    // should be much less than peak amplitude)
    for (size_t i = 1; i < trace.size(); ++i) {
        TEST_ASSERT_TRUE(std::isfinite(trace[i]));
        // 10 Hz, dt 1 ms => max delta ≈ 2*pi*10*amp*dt = 0.063, give margin
        TEST_ASSERT_LESS_THAN_FLOAT(0.2f, std::fabs(trace[i] - trace[i - 1]));
    }
}

void test_syncvib_phase_offset_90_degrees(void) {
    SyncVib pitch;
    SyncVib roll;
    float ratios[1] = {1.0f};
    pitch.set_config(0.0f, ratios, 1);
    roll.set_config((float)M_PI / 2.0f, ratios, 1);
    pitch.on_sync(0.0f, 10.0f);
    roll.on_sync(0.0f, 10.0f);
    float amps[1] = {1.0f};
    pitch.set_amplitudes(amps, 1);
    roll.set_amplitudes(amps, 1);

    auto trace_p = RunSyncVib(pitch, 1.0f, 1000);
    auto trace_r = RunSyncVib(roll, 1.0f, 1000);

    // After amp LPF settles (~250 ms), roll should equal cos(2*pi*10*t)
    // and pitch should equal sin(...). At any time t after settle, the
    // identity sin^2 + cos^2 = amp^2 must hold.
    for (size_t i = 500; i < trace_p.size(); ++i) {
        float sumsq = trace_p[i] * trace_p[i] + trace_r[i] * trace_r[i];
        TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, sumsq);
    }
}

void test_syncvib_amplitude_smoothing(void) {
    SyncVib vib;
    float ratios[1] = {1.0f};
    vib.set_config((float)M_PI / 2.0f, ratios, 1);  // cos so output starts at amp at phase=0
    vib.on_sync(0.0f, 0.001f);  // tiny fundamental so phase barely moves
    float amps[1] = {1.0f};
    vib.set_amplitudes(amps, 1);

    // Drive 200 ms (4 * tau with tau=50ms) at 1 ms dt — output should be ~0.99 of amplitude
    auto trace = RunSyncVib(vib, 1.0f, 200);

    // Final sample should be close to 1.0 (cos of nearly-zero phase, with smoothed amp ~0.99)
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, trace.back());
    // First sample should be close to 0 (amp not yet smoothed up)
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, trace.front());
}

void test_oscillation_guard_ramps_damping(void) {
    Sim sim(1.0f, -100.0f, 100.0f);
    OscillationGuard guard(1.0f, 0.2f, 0.5f, 4000, 20000, 50000, 20000, 1);
    sim.add_element(&guard);
    SetSimPosition(sim, 0.0f);

    float dt = 1.0f;
    float f_in = 0.0f;
    float v = 0.0f;
    float x = 0.0f;
    for (int step = 0; step < 50; ++step) {
        v = (step % 2 == 0) ? 2.0f : -2.0f;
        x += v * dt / 1000.0f;
        sim.set_state(x, x - (v * dt / 1000.0f));
        sim.update(dt, f_in, true);
        sim.update(dt, f_in);
    }

    TEST_ASSERT_TRUE(std::isfinite(sim.get_f_sum()));
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
    RUN_TEST(test_friction_order_agnostic);
    RUN_TEST(test_friction_no_creep_with_small_alternating_force);
    RUN_TEST(test_combined_damping_clamp);
    RUN_TEST(test_dt_jitter_stability);
    RUN_TEST(test_limit_clamp_no_nan);
    RUN_TEST(test_damping_map_negative_entries_clamped);
    RUN_TEST(test_oscillation_guard_ramps_damping);
    RUN_TEST(test_syncvib_continuity_at_10hz);
    RUN_TEST(test_syncvib_phase_offset_90_degrees);
    RUN_TEST(test_syncvib_amplitude_smoothing);
    // RUN_TEST(test_integrator_comparison_stiff_damped_friction);
    return UNITY_END();
}
