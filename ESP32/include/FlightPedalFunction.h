#pragma once
#include "ABSOscillation.h"
#include "Arduino.h"
#include "ConfigManager.h"
#include "ForceCurve.h"
#include "IFunction.h"
#include "Physics.h"

class FlightPedalFunction : public IFunction {
    public:
        FlightPedalFunction(void);
        void update_config(const FlightPedalConfig &config);
        float get_x_contact_point_min(void) override {
            return _config.pos_near_lim;
        }
        float get_x_contact_point_max(void) override {
            return _config.pos_far_lim;
        }
        void on_ffb_action(const FFBAction &ffb_action) override;

    private:
        Spring centering_spring = Spring(0.0, 0.0);
        Damper damper = Damper(1.0);
        FlightPedalConfig _config = FlightPedalConfig_init_default;
};
