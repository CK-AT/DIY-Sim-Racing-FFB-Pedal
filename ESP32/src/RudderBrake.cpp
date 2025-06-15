#include "RudderBrake.h"

RudderBrake::RudderBrake(void) { }


void RudderBrake::process(CommManager &comm_manager, const AuxFunctionConfig &config) {
    if (config.which_specific == AuxFunctionConfig_rudder_brake_tag) {
        float f_sum;
        comm_manager.calc_input_force_sum(config.linked_axes, f_sum);
        float brake_magnitude = normalize_value(f_sum, config.specific.rudder_brake.f_min, config.specific.rudder_brake.f_max) * 2.0;
        float rudder_value = comm_manager.get_controller_output_value(config.specific.rudder_brake.controller_output_axis_flight_pedals);
        float brake_right = constrain(brake_magnitude * rudder_value, 0.0f, 1.0f);
        float brake_left = constrain(brake_magnitude * ((rudder_value * -1.0f) + 1.0f), 0.0f, 1.0f);
        comm_manager.set_controller_output_value(config.specific.rudder_brake.controller_output_axis_left_pedal, brake_left);
        comm_manager.set_controller_output_value(config.specific.rudder_brake.controller_output_axis_right_pedal, brake_right);
    }
}