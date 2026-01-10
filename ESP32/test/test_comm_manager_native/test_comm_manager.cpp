#include <unity.h>

#include "CommManager.h"

void setUp(void) {
}

void tearDown(void) {
}

void test_comm_manager_button_bounds(void) {
    CommManager comm_manager;

    TEST_ASSERT_FALSE(comm_manager.set_controller_button_value(CommManager::JOYSTICK_BUTTON_COUNT, true));
    TEST_ASSERT_TRUE(comm_manager.set_controller_button_value(0, true));
    TEST_ASSERT_TRUE(comm_manager.set_controller_button_value(CommManager::JOYSTICK_BUTTON_COUNT - 1, true));
}

void test_comm_manager_controller_output_values(void) {
    CommManager comm_manager;
    float value_x = 0.25f;
    float value_rz = 0.75f;

    comm_manager.set_controller_output_value(ControllerAxis_CONTROLLER_AXIS_X, value_x);
    comm_manager.set_controller_output_value(ControllerAxis_CONTROLLER_AXIS_R_Z, value_rz);

    TEST_ASSERT_EQUAL_FLOAT(value_x, comm_manager.get_controller_output_value(ControllerAxis_CONTROLLER_AXIS_X));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, comm_manager.get_controller_output_value(ControllerAxis_CONTROLLER_AXIS_Y));
    TEST_ASSERT_EQUAL_FLOAT(value_rz, comm_manager.get_controller_output_value(ControllerAxis_CONTROLLER_AXIS_R_Z));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_comm_manager_button_bounds);
    RUN_TEST(test_comm_manager_controller_output_values);
    return UNITY_END();
}
