#pragma once

#include <stdint.h>

#include "Main.h"

struct LoadCellConfig {
    uint8_t channel_p = 0;
    uint8_t channel_n = 1;
    float loadcell_rating_kg = LOADCELL_WEIGHT_RATING_KG;
    float excitation_v = LOADCELL_EXCITATION_V;
    float sensitivity_mV_V = LOADCELL_SENSITIVITY_MV_V;
    uint32_t offset_samples = 10000;
    uint32_t variance_samples = 10000;
};

class LoadCellAds1256 {
    private:
        LoadCellConfig _cfg;
        float _zero_point = 0.0;
        float _variance_estimate = 0.0;
        float _standard_deviation_estimate = 0.0;
        bool begin() const;

    public:
        explicit LoadCellAds1256(const LoadCellConfig &cfg = LoadCellConfig());
        float get_reading_kg() const;
        bool try_get_reading_kg(float &reading_kg) const;
        void set_loadcell_rating(uint8_t load_cell_rating_u8);

    public:
        bool set_zero_point(uint32_t sample_count = 0);
        bool estimate_variance(uint32_t sample_count = 0);

    public:
        float get_variance_estimate() const {
            return _variance_estimate;
        }
        float get_standard_deviation_estimate() const {
            return _standard_deviation_estimate;
        }
};
