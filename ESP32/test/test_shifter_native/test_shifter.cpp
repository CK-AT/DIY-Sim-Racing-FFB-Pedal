#include <cmath>

#include <unity.h>

#include "CommManager.h"
#include "ShifterDetect.h"
#include "ShifterFunction.h"

void setUp(void) {
}

void tearDown(void) {
}

namespace {
constexpr float k_tenth_mm = 0.1f;

float to_mm(int32_t value_tenth_mm) {
    return static_cast<float>(value_tenth_mm) * k_tenth_mm;
}

void SetSimPosition(Sim &sim, float position) {
    sim.set_x_min(position, true);
    sim.set_x_max(position, true);
    float dt = 1.0f;
    float f_in = 0.0f;
    sim.update(dt, f_in, true);
    sim.update(dt, f_in, true);
    sim.update(dt, f_in, true);
}

void BuildDetectConfig(AuxFunctionConfig &config) {
    config.which_specific = AuxFunctionConfig_shifter_detect_tag;
    config.linked_axes[0] = AxisID_AXIS_ID_1;
    config.linked_axes[1] = AxisID_AXIS_ID_2;
    config.specific.shifter_detect.gear_slots_count = 1;
    config.specific.shifter_detect.gear_slots[0].center_x = 0;
    config.specific.shifter_detect.gear_slots[0].center_y = 0;
    config.specific.shifter_detect.gear_slots[0].half_width = 50;
    config.specific.shifter_detect.gear_slots[0].half_height = 50;
    config.specific.shifter_detect.gear_slots[0].gear = ShifterGear_SHIFTER_GEAR_NEUTRAL;
    config.specific.shifter_detect.hysteresis = 20;
}

void BuildDemoDetectConfig(AuxFunctionConfig &config) {
    config.which_specific = AuxFunctionConfig_shifter_detect_tag;
    config.linked_axes[0] = AxisID_AXIS_ID_1;
    config.linked_axes[1] = AxisID_AXIS_ID_2;
    config.linked_axes[2] = AxisID_AXIS_UNDEFINED;
    config.linked_axes[3] = AxisID_AXIS_UNDEFINED;
    config.specific.shifter_detect.gear_slots_count = 7;
    config.specific.shifter_detect.gear_slots[0].center_x = -400;
    config.specific.shifter_detect.gear_slots[0].center_y = 450;
    config.specific.shifter_detect.gear_slots[0].half_width = 80;
    config.specific.shifter_detect.gear_slots[0].half_height = 80;
    config.specific.shifter_detect.gear_slots[0].gear = ShifterGear_SHIFTER_GEAR_1;
    config.specific.shifter_detect.gear_slots[1].center_x = -400;
    config.specific.shifter_detect.gear_slots[1].center_y = -450;
    config.specific.shifter_detect.gear_slots[1].half_width = 80;
    config.specific.shifter_detect.gear_slots[1].half_height = 80;
    config.specific.shifter_detect.gear_slots[1].gear = ShifterGear_SHIFTER_GEAR_2;
    config.specific.shifter_detect.gear_slots[2].center_x = 0;
    config.specific.shifter_detect.gear_slots[2].center_y = 450;
    config.specific.shifter_detect.gear_slots[2].half_width = 80;
    config.specific.shifter_detect.gear_slots[2].half_height = 80;
    config.specific.shifter_detect.gear_slots[2].gear = ShifterGear_SHIFTER_GEAR_3;
    config.specific.shifter_detect.gear_slots[3].center_x = 0;
    config.specific.shifter_detect.gear_slots[3].center_y = -450;
    config.specific.shifter_detect.gear_slots[3].half_width = 80;
    config.specific.shifter_detect.gear_slots[3].half_height = 80;
    config.specific.shifter_detect.gear_slots[3].gear = ShifterGear_SHIFTER_GEAR_4;
    config.specific.shifter_detect.gear_slots[4].center_x = 400;
    config.specific.shifter_detect.gear_slots[4].center_y = 450;
    config.specific.shifter_detect.gear_slots[4].half_width = 80;
    config.specific.shifter_detect.gear_slots[4].half_height = 80;
    config.specific.shifter_detect.gear_slots[4].gear = ShifterGear_SHIFTER_GEAR_5;
    config.specific.shifter_detect.gear_slots[5].center_x = 400;
    config.specific.shifter_detect.gear_slots[5].center_y = -450;
    config.specific.shifter_detect.gear_slots[5].half_width = 80;
    config.specific.shifter_detect.gear_slots[5].half_height = 80;
    config.specific.shifter_detect.gear_slots[5].gear = ShifterGear_SHIFTER_GEAR_6;
    config.specific.shifter_detect.gear_slots[6].center_x = -700;
    config.specific.shifter_detect.gear_slots[6].center_y = -450;
    config.specific.shifter_detect.gear_slots[6].half_width = 70;
    config.specific.shifter_detect.gear_slots[6].half_height = 80;
    config.specific.shifter_detect.gear_slots[6].gear = ShifterGear_SHIFTER_GEAR_REVERSE;
    config.specific.shifter_detect.hysteresis = 20;
}

void BuildGateConfig(ShifterConfig &config) {
    config.pos_x_min = -20;
    config.pos_x_max = 20;
    config.pos_y_min = -20;
    config.pos_y_max = 20;
    config.damping = 0.0f;
    // Use max_force as the centering spring rate for the tested axis
    config.max_force = 5.0f;
    config.grid_step = 10;
    config.sequential = false;
    config.gate_segments_count = 1;
    config.gate_segments[0].x0 = 0;
    config.gate_segments[0].y0 = -200;
    config.gate_segments[0].x1 = 0;
    config.gate_segments[0].y1 = 200;
    config.gate_segments[0].half_width = 50;
    config.gate_segments[0].spring_center = 1.0f;
    config.gate_segments[0].spring_wall = 5.0f;
    config.detents_count = 0;
}

void BuildDemoGateConfig(ShifterConfig &config) {
    config.pos_x_min = -80;
    config.pos_x_max = 80;
    config.pos_y_min = -60;
    config.pos_y_max = 60;
    config.damping = 0.5f;
    config.max_force = 50.0f;
    config.grid_step = 100;
    config.sequential = false;
    config.gate_segments_count = 5;
    config.gate_segments[0].x0 = -800;
    config.gate_segments[0].y0 = 0;
    config.gate_segments[0].x1 = 600;
    config.gate_segments[0].y1 = 0;
    config.gate_segments[0].half_width = 60;
    config.gate_segments[0].spring_center = 1.0f;
    config.gate_segments[0].spring_wall = 8.0f;
    config.gate_segments[1].x0 = -400;
    config.gate_segments[1].y0 = -550;
    config.gate_segments[1].x1 = -400;
    config.gate_segments[1].y1 = 550;
    config.gate_segments[1].half_width = 60;
    config.gate_segments[1].spring_center = 1.0f;
    config.gate_segments[1].spring_wall = 8.0f;
    config.gate_segments[2].x0 = 0;
    config.gate_segments[2].y0 = -550;
    config.gate_segments[2].x1 = 0;
    config.gate_segments[2].y1 = 550;
    config.gate_segments[2].half_width = 60;
    config.gate_segments[2].spring_center = 1.0f;
    config.gate_segments[2].spring_wall = 8.0f;
    config.gate_segments[3].x0 = 400;
    config.gate_segments[3].y0 = -550;
    config.gate_segments[3].x1 = 400;
    config.gate_segments[3].y1 = 550;
    config.gate_segments[3].half_width = 60;
    config.gate_segments[3].spring_center = 1.0f;
    config.gate_segments[3].spring_wall = 8.0f;
    config.gate_segments[4].x0 = -700;
    config.gate_segments[4].y0 = -550;
    config.gate_segments[4].x1 = -700;
    config.gate_segments[4].y1 = 550;
    config.gate_segments[4].half_width = 40;
    config.gate_segments[4].spring_center = 1.0f;
    config.gate_segments[4].spring_wall = 10.0f;
    config.detents_count = 7;
    config.detents[0].x = -400;
    config.detents[0].y = 450;
    config.detents[0].radius = 60;
    config.detents[0].spring = 6.0f;
    config.detents[1].x = -400;
    config.detents[1].y = -450;
    config.detents[1].radius = 60;
    config.detents[1].spring = 6.0f;
    config.detents[2].x = 0;
    config.detents[2].y = 450;
    config.detents[2].radius = 60;
    config.detents[2].spring = 6.0f;
    config.detents[3].x = 0;
    config.detents[3].y = -450;
    config.detents[3].radius = 60;
    config.detents[3].spring = 6.0f;
    config.detents[4].x = 400;
    config.detents[4].y = 450;
    config.detents[4].radius = 60;
    config.detents[4].spring = 6.0f;
    config.detents[5].x = 400;
    config.detents[5].y = -450;
    config.detents[5].radius = 60;
    config.detents[5].spring = 6.0f;
    config.detents[6].x = -700;
    config.detents[6].y = -450;
    config.detents[6].radius = 60;
    config.detents[6].spring = 7.0f;
}
}  // namespace

void test_shifter_detect_hysteresis(void) {
    CommManager comm_manager;
    ShifterDetect detect;
    AuxFunctionConfig config = {};
    BuildDetectConfig(config);

    comm_manager.set_position(AxisID_AXIS_ID_1, 0.0f);
    comm_manager.set_position(AxisID_AXIS_ID_2, 0.0f);
    detect.process(comm_manager, config);
    TEST_ASSERT_TRUE(comm_manager.get_controller_button_value(0));

    comm_manager.clear_buttons();
    comm_manager.set_position(AxisID_AXIS_ID_1, 6.0f);
    detect.process(comm_manager, config);
    TEST_ASSERT_TRUE(comm_manager.get_controller_button_value(0));
}

void test_shifter_function_gate_force_direction(void) {
    CommManager comm_manager(AxisID_AXIS_ID_1);
    comm_manager.set_position(AxisID_AXIS_ID_2, 0.0f);

    ShifterConfig config = ShifterConfig_init_default;
    BuildGateConfig(config);

    AxisID linked_axes[4] = {AxisID_AXIS_ID_1, AxisID_AXIS_ID_2, AxisID_AXIS_UNDEFINED, AxisID_AXIS_UNDEFINED};
    AuxFunctionConfig detect_config = {};
    BuildDetectConfig(detect_config);
    ShifterFunction shifter;
    shifter.update_config(config, detect_config.specific.shifter_detect, comm_manager, linked_axes);
    shifter.enable();

    Sim sim(1.0f, -20.0f, 20.0f);
    SetSimPosition(sim, 10.0f);
    SimState state = {};
    SimAccumulators accum = {};
    state.x = sim.get_x();
    state.v = sim.get_v();
    state.a = sim.get_a();
    state.dt_ms = 1.0f;
    state.m = sim.get_m();
    state.x_min = sim.get_x_min();
    state.x_max = sim.get_x_max();
    accum.f_sum = 0.0f;
    shifter.update(state, accum);
    TEST_ASSERT_TRUE(accum.f_sum < -0.1f);

    SetSimPosition(sim, -10.0f);
    state.x = sim.get_x();
    state.v = sim.get_v();
    accum.f_sum = 0.0f;
    shifter.update(state, accum);
    TEST_ASSERT_TRUE(accum.f_sum > 0.1f);
}

void test_shifter_function_corridor_center_force_zero(void) {
    CommManager comm_manager(AxisID_AXIS_ID_1);
    comm_manager.set_position(AxisID_AXIS_ID_2, 0.0f);

    ShifterConfig config = ShifterConfig_init_default;
    BuildGateConfig(config);

    AxisID linked_axes[4] = {AxisID_AXIS_ID_1, AxisID_AXIS_ID_2, AxisID_AXIS_UNDEFINED, AxisID_AXIS_UNDEFINED};
    AuxFunctionConfig detect_config = {};
    BuildDetectConfig(detect_config);
    ShifterFunction shifter;
    shifter.update_config(config, detect_config.specific.shifter_detect, comm_manager, linked_axes);
    shifter.enable();

    Sim sim(1.0f, -20.0f, 20.0f);
    SetSimPosition(sim, 0.0f);
    SimState state = {};
    SimAccumulators accum = {};
    state.x = sim.get_x();
    state.v = sim.get_v();
    state.a = sim.get_a();
    state.dt_ms = 1.0f;
    state.m = sim.get_m();
    state.x_min = sim.get_x_min();
    state.x_max = sim.get_x_max();
    accum.f_sum = 0.0f;
    shifter.update(state, accum);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, accum.f_sum);
}

void test_shifter_detect_demo_slots(void) {
    CommManager comm_manager;
    AuxFunctionConfig config = {};
    BuildDemoDetectConfig(config);

    struct Slot {
        int32_t x_tenth;
        int32_t y_tenth;
        ShifterGear gear;
    };

    const Slot slots[] = {
        {-400, 450, ShifterGear_SHIFTER_GEAR_1},
        {-400, -450, ShifterGear_SHIFTER_GEAR_2},
        {0, 450, ShifterGear_SHIFTER_GEAR_3},
        {0, -450, ShifterGear_SHIFTER_GEAR_4},
        {400, 450, ShifterGear_SHIFTER_GEAR_5},
        {400, -450, ShifterGear_SHIFTER_GEAR_6},
        {-700, -450, ShifterGear_SHIFTER_GEAR_REVERSE},
    };

    for (const auto &slot : slots) {
        ShifterDetect detect;
        comm_manager.clear_buttons();
        comm_manager.set_position(AxisID_AXIS_ID_1, to_mm(slot.x_tenth));
        comm_manager.set_position(AxisID_AXIS_ID_2, to_mm(slot.y_tenth));
        detect.process(comm_manager, config);

        for (uint8_t idx = 0; idx < CommManager::JOYSTICK_BUTTON_COUNT; idx++) {
            bool expected = (idx == static_cast<uint8_t>(slot.gear));
            TEST_ASSERT_EQUAL(expected, comm_manager.get_controller_button_value(idx));
        }
    }

    {
        ShifterDetect detect;
        comm_manager.clear_buttons();
        comm_manager.set_position(AxisID_AXIS_ID_1, 0.0f);
        comm_manager.set_position(AxisID_AXIS_ID_2, 0.0f);
        detect.process(comm_manager, config);
        for (uint8_t idx = 0; idx < CommManager::JOYSTICK_BUTTON_COUNT; idx++) {
            bool expected = (idx == 0);
            TEST_ASSERT_EQUAL(expected, comm_manager.get_controller_button_value(idx));
        }
    }
}

void test_shifter_function_demo_gate_x_force_direction(void) {
    CommManager comm_manager(AxisID_AXIS_ID_1);
    comm_manager.set_position(AxisID_AXIS_ID_2, 30.0f);

    ShifterConfig config = ShifterConfig_init_default;
    BuildDemoGateConfig(config);

    AxisID linked_axes[4] = {AxisID_AXIS_ID_1, AxisID_AXIS_ID_2, AxisID_AXIS_UNDEFINED, AxisID_AXIS_UNDEFINED};
    AuxFunctionConfig detect_config = {};
    BuildDemoDetectConfig(detect_config);
    ShifterFunction shifter;
    shifter.update_config(config, detect_config.specific.shifter_detect, comm_manager, linked_axes);
    shifter.enable();

    Sim sim(1.0f, float(config.pos_x_min), float(config.pos_x_max));
    SetSimPosition(sim, 10.0f);
    SimState state = {};
    SimAccumulators accum = {};
    state.x = sim.get_x();
    state.v = sim.get_v();
    state.a = sim.get_a();
    state.dt_ms = 1.0f;
    state.m = sim.get_m();
    state.x_min = sim.get_x_min();
    state.x_max = sim.get_x_max();
    accum.f_sum = 0.0f;
    shifter.update(state, accum);
    TEST_ASSERT_TRUE(accum.f_sum < -0.1f);

    SetSimPosition(sim, -10.0f);
    state.x = sim.get_x();
    state.v = sim.get_v();
    accum.f_sum = 0.0f;
    shifter.update(state, accum);
    TEST_ASSERT_TRUE(accum.f_sum > 0.1f);
}

void test_shifter_function_demo_neutral_centering(void) {
    CommManager comm_manager(AxisID_AXIS_ID_1);
    comm_manager.set_position(AxisID_AXIS_ID_2, 0.0f);

    ShifterConfig config = ShifterConfig_init_default;
    BuildDemoGateConfig(config);

    AxisID linked_axes[4] = {AxisID_AXIS_ID_1, AxisID_AXIS_ID_2, AxisID_AXIS_UNDEFINED, AxisID_AXIS_UNDEFINED};
    AuxFunctionConfig detect_config = {};
    BuildDemoDetectConfig(detect_config);
    ShifterFunction shifter;
    shifter.update_config(config, detect_config.specific.shifter_detect, comm_manager, linked_axes);
    shifter.enable();

    Sim sim(1.0f, float(config.pos_x_min), float(config.pos_x_max));
    SetSimPosition(sim, 10.0f);
    SimState state = {};
    SimAccumulators accum = {};
    state.x = sim.get_x();
    state.v = sim.get_v();
    state.a = sim.get_a();
    state.dt_ms = 1.0f;
    state.m = sim.get_m();
    state.x_min = sim.get_x_min();
    state.x_max = sim.get_x_max();
    accum.f_sum = 0.0f;
    shifter.update(state, accum);
    TEST_ASSERT_TRUE(accum.f_sum < -0.05f);

    SetSimPosition(sim, -10.0f);
    state.x = sim.get_x();
    state.v = sim.get_v();
    accum.f_sum = 0.0f;
    shifter.update(state, accum);
    TEST_ASSERT_TRUE(accum.f_sum > 0.05f);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_shifter_detect_hysteresis);
    RUN_TEST(test_shifter_function_gate_force_direction);
    RUN_TEST(test_shifter_function_corridor_center_force_zero);
    RUN_TEST(test_shifter_detect_demo_slots);
    RUN_TEST(test_shifter_function_demo_gate_x_force_direction);
    RUN_TEST(test_shifter_function_demo_neutral_centering);
    return UNITY_END();
}
