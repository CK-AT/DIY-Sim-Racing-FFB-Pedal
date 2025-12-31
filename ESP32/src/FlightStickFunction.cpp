#include "FlightStickFunction.h"

FlightStickFunction::FlightStickFunction(void) {
    disable();
    add_element(&centering_spring);
    add_element(&damper);
}

void FlightStickFunction::update_config_common(const FlightStickConfigCommon &config) {
    _config = config;
    damper.set_k(_config.damping);
    centering_spring.set_k(_config.centering_spring_const);
    float center = float(_config.pos_min) + (float(_config.pos_max - _config.pos_min) / 2.0f);
    centering_spring.set_offset(center);
}

void FlightStickFunction::update_config(const FlightStickPitchConfig &config) {
    FlightStickConfigCommon common;
    common.pos_min = config.pos_min;
    common.pos_max = config.pos_max;
    common.damping = config.damping;
    common.centering_spring_const = config.centering_spring_const;
    update_config_common(common);
}

void FlightStickFunction::update_config(const FlightStickRollConfig &config) {
    FlightStickConfigCommon common;
    common.pos_min = config.pos_min;
    common.pos_max = config.pos_max;
    common.damping = config.damping;
    common.centering_spring_const = config.centering_spring_const;
    update_config_common(common);
}

void FlightStickFunction::on_ffb_action(const FFBAction &ffb_action) {
}
