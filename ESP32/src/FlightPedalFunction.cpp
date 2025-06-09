#include "FlightPedalFunction.h"

FlightPedalFunction::FlightPedalFunction(void) {
    disable();
    add_element(&centering_spring);
    add_element(&damper);
}

void FlightPedalFunction::update_config(const FlightPedalConfig &config) {
    _config = config;
    damper.set_k(_config.damping);
    centering_spring.set_k(_config.centering_spring_const);
    float center = float(_config.pos_near_lim) + (float(_config.pos_far_lim - _config.pos_near_lim) / 2.0f);
    centering_spring.set_offset(center);
}

void FlightPedalFunction::on_ffb_action(const FFBAction &ffb_action) {
}

