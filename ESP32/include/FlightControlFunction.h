#pragma once
#include "Arduino.h"
#include "ConfigManager.h"
#include "IFunction.h"
#include "Physics.h"

class FlightControlFunction : public IFunction {
    public:
        FlightControlFunction(void);
        void update_config(const FlightControlConfig &config);
        void update(const SimState &state, SimAccumulators &accum) override;
        float get_x_contact_point_min(void) override {
            return _config.pos_min;
        }
        float get_x_contact_point_max(void) override {
            return _config.pos_max;
        }
        void on_ffb_action(const FFBAction &ffb_action) override;
        void on_dds_sync(uint8_t dds_index, float phase, float hz) override;

    private:
        Spring centering_spring = Spring(0.0f, 0.0f);
        Damper damper = Damper(1.0f);
        Buffet buffet = Buffet(0.0f);
        ConstForce load_force = ConstForce(0.0f);
        SyncVib vib1;
        SyncVib vib2;
        FlightControlConfig _config = FlightControlConfig_init_zero;
        float _base_center = 0.0f;
        uint32_t _last_ffb_ms = 0;
        bool _ffb_overridden = false;
};
