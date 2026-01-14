#pragma once

#include "Physics.h"
#include "ConfigManager.h"

// The number of segments, which are defined for the spline
#define NUMBER_OF_SPLINE_SEGMENTS 5

class SplineForceCurve : public SimElement {
    public:
        SplineForceCurve(void) { }
        void update(const SimState &state, SimAccumulators &accum);
        void set_config(const SplineForceCurveConfig &config) { _config = &config; }
    
    private:
        const SplineForceCurveConfig *_config = nullptr;
};
