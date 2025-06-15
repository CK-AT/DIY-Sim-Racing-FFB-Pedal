#pragma once
#include "ABSOscillation.h"
#include "Arduino.h"
#include "ConfigManager.h"
#include "ForceCurve.h"
#include "IFunction.h"
#include "Physics.h"

class FlightPedalsFunction : public IFunction {
    public:
        FlightPedalsFunction(void);
        void update_config(const FlightPedalsConfig &config);
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
        FlightPedalsConfig _config = FlightPedalsConfig_init_default;
};
