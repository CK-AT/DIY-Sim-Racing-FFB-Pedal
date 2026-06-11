#include <unity.h>

#include "FunctionConflict.h"

namespace {

// Builds a FunctionBase with up to 4 linked axes and a controller output axis.
FunctionBase make_base(AxisID a0, AxisID a1, AxisID a2, AxisID a3, ControllerAxis out) {
    FunctionBase b = FunctionBase_init_default;
    b.linked_axes[0] = a0;
    b.linked_axes[1] = a1;
    b.linked_axes[2] = a2;
    b.linked_axes[3] = a3;
    b.controller_output_axis = out;
    return b;
}

const AxisID U = AxisID_AXIS_UNDEFINED;

}  // namespace

void setUp(void) {}
void tearDown(void) {}

// Same physical axis -> conflict, regardless of output axis.
void test_conflict_shared_axis(void) {
    FunctionBase a = make_base(AxisID_AXIS_ID_2, U, U, U, ControllerAxis_CONTROLLER_AXIS_RUD);
    FunctionBase b = make_base(AxisID_AXIS_ID_2, U, U, U, ControllerAxis_CONTROLLER_AXIS_BRK);
    TEST_ASSERT_TRUE(functions_conflict(a, b));
}

// Disjoint physical axes and different output axes -> no conflict.
void test_no_conflict_disjoint(void) {
    FunctionBase a = make_base(AxisID_AXIS_ID_1, U, U, U, ControllerAxis_CONTROLLER_AXIS_ACC);
    FunctionBase b = make_base(AxisID_AXIS_ID_2, U, U, U, ControllerAxis_CONTROLLER_AXIS_BRK);
    TEST_ASSERT_FALSE(functions_conflict(a, b));
}

// Different physical axes but same controller output axis -> conflict.
void test_conflict_shared_output(void) {
    FunctionBase a = make_base(AxisID_AXIS_ID_1, U, U, U, ControllerAxis_CONTROLLER_AXIS_BRK);
    FunctionBase b = make_base(AxisID_AXIS_ID_2, U, U, U, ControllerAxis_CONTROLLER_AXIS_BRK);
    TEST_ASSERT_TRUE(functions_conflict(a, b));
}

// Two cleared configs (no axes, UNDEFINED output) must NOT conflict — otherwise
// clearing one function would evict every other cleared function.
void test_no_conflict_both_undefined_output(void) {
    FunctionBase a = make_base(U, U, U, U, ControllerAxis_CONTROLLER_AXIS_UNDEFINED);
    FunctionBase b = make_base(U, U, U, U, ControllerAxis_CONTROLLER_AXIS_UNDEFINED);
    TEST_ASSERT_FALSE(functions_conflict(a, b));
}

// Axis flags (INDEPENDENT/SUBTRACTIVE) in the high bits must be masked off when
// comparing physical axes.
void test_conflict_axis_flags_masked(void) {
    FunctionBase a = make_base(AxisID(AxisID_AXIS_ID_2 | AxisID_AXIS_INDEPENDENT), U, U, U,
                               ControllerAxis_CONTROLLER_AXIS_RUD);
    FunctionBase b = make_base(AxisID(AxisID_AXIS_ID_2 | AxisID_AXIS_SUBTRACTIVE), U, U, U,
                               ControllerAxis_CONTROLLER_AXIS_BRK);
    TEST_ASSERT_TRUE(functions_conflict(a, b));
}

// Overlap on a non-first linked axis (multi-axis function) still conflicts.
void test_conflict_overlap_secondary_axis(void) {
    FunctionBase a = make_base(AxisID_AXIS_ID_1, AxisID_AXIS_ID_3, U, U, ControllerAxis_CONTROLLER_AXIS_ACC);
    FunctionBase b = make_base(AxisID_AXIS_ID_3, U, U, U, ControllerAxis_CONTROLLER_AXIS_BRK);
    TEST_ASSERT_TRUE(functions_conflict(a, b));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_conflict_shared_axis);
    RUN_TEST(test_no_conflict_disjoint);
    RUN_TEST(test_conflict_shared_output);
    RUN_TEST(test_no_conflict_both_undefined_output);
    RUN_TEST(test_conflict_axis_flags_masked);
    RUN_TEST(test_conflict_overlap_secondary_axis);
    return UNITY_END();
}
