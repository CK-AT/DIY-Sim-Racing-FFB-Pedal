#pragma once
#include <Arduino.h>

// Authoritative DDS phase accumulator owned by the gateway. Two slots
// (DDS 1 = main rotor / primary engine, DDS 2 = secondary engine /
// tail rotor) advance independently at their respective fundamentals.
// Snooped from FlightFfbAction by CommManager; broadcast at 100 Hz over
// the active downlink channel via ICommChannel::send_dds_sync.
//
// Pure model — no transport knowledge.
class MasterDds {
    public:
        static constexpr uint8_t NUM_DDS = 2;

        void set_fundamental(uint8_t dds_index, float hz);

        // Advance phase by 2*pi*hz*dt, dt derived from now_us. Lazy-inits
        // _ti_prev_us on first call (first dt = 0).
        void tick(uint32_t now_us);

        float get_fundamental(uint8_t dds_index) const;
        float get_phase(uint8_t dds_index) const;

    private:
        float _hz[NUM_DDS] = {0.0f, 0.0f};
        float _phase[NUM_DDS] = {0.0f, 0.0f};
        uint32_t _ti_prev_us = 0;
};
