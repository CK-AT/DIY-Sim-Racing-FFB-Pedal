#include "FlightControlFunction.h"
#include "LogOutput.h"

FlightControlFunction::FlightControlFunction(void) {
    disable();
    add_element(&centering_spring);
    add_element(&damper);
    add_element(&buffet);
    add_element(&load_force);
    add_element(&vib1);
    add_element(&vib2);
}

void FlightControlFunction::update_config(const FlightControlConfig &config) {
    _config = config;
    damper.set_k(_config.damping);
    centering_spring.set_k(_config.centering_spring_const);
    _base_center = float(_config.pos_min) + (float(_config.pos_max - _config.pos_min) / 2.0f);
    centering_spring.set_offset(_base_center);
    load_force.set_f(0.0f);
    // DDS 1: phase_offset + ratios from config (encodes axis + rotor handedness)
    vib1.set_config(_config.phase_offset,
                    _config.vib_harmonic_ratios,
                    (uint8_t)_config.vib_harmonic_ratios_count);
    // DDS 2: engine vibration is isotropic — phase_offset always 0
    vib2.set_config(0.0f,
                    _config.vib2_harmonic_ratios,
                    (uint8_t)_config.vib2_harmonic_ratios_count);
    LogOutput::printf("FlightControl DDS cfg: phase_offset=%.3f rad, vib1 ratios=[%.2f,%.2f,%.2f,%.2f,%.2f] (n=%u), vib2 ratios=[%.2f,%.2f] (n=%u)",
                      _config.phase_offset,
                      _config.vib_harmonic_ratios[0], _config.vib_harmonic_ratios[1],
                      _config.vib_harmonic_ratios[2], _config.vib_harmonic_ratios[3],
                      _config.vib_harmonic_ratios[4],
                      (unsigned)_config.vib_harmonic_ratios_count,
                      _config.vib2_harmonic_ratios[0], _config.vib2_harmonic_ratios[1],
                      (unsigned)_config.vib2_harmonic_ratios_count);
}

void FlightControlFunction::on_ffb_action(const FFBAction &ffb_action) {
    if (ffb_action.which_function != FFBAction_flight_ffb_tag) {
        return;
    }
    const FlightFfbAction &flight = ffb_action.function.flight_ffb;
    // Frame is authoritative on damping. AxisConfig.min_damping enforces the
    // unconditional safety floor at the Sim integrator (Physics.cpp).
    damper.set_k(flight.k_damper);
    centering_spring.set_k(flight.k_spring);
    centering_spring.set_offset(_base_center + flight.trim_offset);
    buffet.set_amplitude(flight.buffet_amp);
    load_force.set_f(flight.load_force);
    // Wire format: uint8 at 0.05 N/LSB (range 0..12.75 N). Decode here.
    constexpr float kAmpScale = 0.05f;
    float amps1[5] = {flight.vib_amp_slot1 * kAmpScale, flight.vib_amp_slot2 * kAmpScale,
                      flight.vib_amp_slot3 * kAmpScale, flight.vib_amp_slot4 * kAmpScale,
                      flight.vib_amp_slot5 * kAmpScale};
    vib1.set_amplitudes(amps1, 5);
    float amps2[2] = {flight.vib2_amp_slot1 * kAmpScale, flight.vib2_amp_slot2 * kAmpScale};
    vib2.set_amplitudes(amps2, 2);
    _last_ffb_ms = millis();
    _ffb_overridden = true;
}

void FlightControlFunction::on_dds_sync(uint8_t dds_index, float phase, float hz) {
    if (dds_index == 0) vib1.on_sync(phase, hz);
    else if (dds_index == 1) vib2.on_sync(phase, hz);
}

void FlightControlFunction::update(const SimState &state, SimAccumulators &accum) {
    if (_ffb_overridden) {
        uint32_t elapsed = millis() - _last_ffb_ms;
        if (elapsed > 200) {
            damper.set_k(_config.damping);
            centering_spring.set_k(_config.centering_spring_const);
            centering_spring.set_offset(_base_center);
            buffet.set_amplitude(0.0f);
            load_force.set_f(0.0f);
            // Zero amplitudes; SyncVib's internal LPF (50 ms tau) handles fade.
            float zeros[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            vib1.set_amplitudes(zeros, 5);
            vib2.set_amplitudes(zeros, 2);
            _ffb_overridden = false;
        }
    }

    CompoundElement::update(state, accum);
}
