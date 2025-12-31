#pragma once
#include "Arduino.h"
#include "ConfigManager.h"
#include "IFunction.h"
#include "Physics.h"

class FlightStickFunction : public IFunction {
    public:
        FlightStickFunction(void);
        void update_config(const FlightStickPitchConfig &config);
        void update_config(const FlightStickRollConfig &config);
        float get_x_contact_point_min(void) override {
            return _config.pos_min;
        }
        float get_x_contact_point_max(void) override {
            return _config.pos_max;
        }
        void on_ffb_action(const FFBAction &ffb_action) override;

    private:
        struct FlightStickConfigCommon {
                int32_t pos_min = 0;
                int32_t pos_max = 0;
                float damping = 0.0f;
                float centering_spring_const = 0.0f;
        };
        void update_config_common(const FlightStickConfigCommon &config);
        Spring centering_spring = Spring(0.0f, 0.0f);
        Damper damper = Damper(1.0f);
        FlightStickConfigCommon _config = {};
};
