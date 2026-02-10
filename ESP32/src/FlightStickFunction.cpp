#include "FlightStickFunction.h"

FlightStickFunction::FlightStickFunction(void) {
    disable();
    add_element(&centering_spring);
    add_element(&damper);
    add_element(&buffet);
    add_element(&load_force);
}

void FlightStickFunction::update_config(const FlightStickConfig &config) {
    _config = config;
    damper.set_k(_config.damping);
    centering_spring.set_k(_config.centering_spring_const);
    _base_center = float(_config.pos_min) + (float(_config.pos_max - _config.pos_min) / 2.0f);
    centering_spring.set_offset(_base_center);
    load_force.set_f(0.0f);
}

void FlightStickFunction::on_ffb_action(const FFBAction &ffb_action) {
    if (ffb_action.which_function != FFBAction_flight_ffb_tag) {
        return;
    }
    damper.set_k(max(_config.damping, ffb_action.function.flight_ffb.k_damper));
    centering_spring.set_k(ffb_action.function.flight_ffb.k_spring);
    centering_spring.set_offset(_base_center + ffb_action.function.flight_ffb.trim_offset);
    buffet.set_amplitude(ffb_action.function.flight_ffb.buffet_amp);
    load_force.set_f(ffb_action.function.flight_ffb.load_force);
    _last_ffb_ms = millis();
    _ffb_overridden = true;
}

void FlightStickFunction::update(const SimState &state, SimAccumulators &accum) {
    if (_ffb_overridden) {
        uint32_t elapsed = millis() - _last_ffb_ms;
        if (elapsed > 200) {
            damper.set_k(_config.damping);
            centering_spring.set_k(_config.centering_spring_const);
            centering_spring.set_offset(_base_center);
            buffet.set_amplitude(0.0f);
            load_force.set_f(0.0f);
            _ffb_overridden = false;
        }
    }

    CompoundElement::update(state, accum);
}
