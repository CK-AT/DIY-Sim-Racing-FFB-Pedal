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
        void update(const SimState &state, SimAccumulators &accum) override;
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
        Buffet buffet = Buffet(0.0f);
        FlightPedalsConfig _config = FlightPedalsConfig_init_default;
        float _base_center = 0.0f;
        uint32_t _last_ffb_ms = 0;
        bool _ffb_overridden = false;
};
