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
        float _zeroPoint = 0.0;
        float _varianceEstimate = 0.0;
        float _standardDeviationEstimate = 0.0;
        bool begin() const;

    public:
        explicit LoadCellAds1256(const LoadCellConfig &cfg = LoadCellConfig());
        float get_reading_kg() const;
        bool try_get_reading_kg(float &readingKg) const;
        void set_loadcell_rating(uint8_t loadcellRating_u8) const;

    public:
        bool set_zero_point(uint32_t sample_count = 0);
        bool estimate_variance(uint32_t sample_count = 0);

    public:
        float get_variance_estimate() const {
            return _varianceEstimate;
        }
        float get_standard_deviation_estimate() const {
            return _standardDeviationEstimate;
        }
};
