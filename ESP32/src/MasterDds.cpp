#include "MasterDds.h"

void MasterDds::set_fundamental(uint8_t dds_index, float hz) {
    if (dds_index >= NUM_DDS) return;
    if (hz < 0.0f) hz = 0.0f;
    _hz[dds_index] = hz;
}

void MasterDds::tick(uint32_t now_us) {
    if (_ti_prev_us == 0) {
        _ti_prev_us = now_us;
        return;
    }
    float dt_s = (now_us - _ti_prev_us) * 1e-6f;
    _ti_prev_us = now_us;
    for (uint8_t i = 0; i < NUM_DDS; i++) {
        _phase[i] += 2.0f * (float)M_PI * _hz[i] * dt_s;
        // fmodf is O(1); iterative subtraction spins forever on a non-finite or
        // large phase.
        if (!isfinite(_phase[i])) _phase[i] = 0.0f;
        _phase[i] = fmodf(_phase[i], 2.0f * (float)M_PI);
        if (_phase[i] < 0.0f) _phase[i] += 2.0f * (float)M_PI;
    }
}

float MasterDds::get_fundamental(uint8_t dds_index) const {
    if (dds_index >= NUM_DDS) return 0.0f;
    return _hz[dds_index];
}

float MasterDds::get_phase(uint8_t dds_index) const {
    if (dds_index >= NUM_DDS) return 0.0f;
    return _phase[dds_index];
}
