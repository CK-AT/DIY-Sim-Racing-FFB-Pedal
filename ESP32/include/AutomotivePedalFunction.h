#pragma once
#include "ABSOscillation.h"
#include "Arduino.h"
#include "ConfigManager.h"
#include "ForceCurve.h"
#include "IFunction.h"
#include "Physics.h"

class AutomotivePedalFunction : public IFunction {
    public:
        AutomotivePedalFunction(void);
        void update_config(const AutomotivePedalConfig &config);
        void trigger_abs(void) {
            abs_effect.trigger();
        }
        float get_x_contact_point_min(void) override {
            return _config.pos_idle;
        }
        float get_x_contact_point_max(void) override {
            return _config.pos_end;
        }
        void on_ffb_action(const FFBAction &ffb_action) override;

    private:
        SplineForceCurve force_curve = {};
        Damper damper = Damper(1.0);
        ABSOscillation abs_effect = {};
        AutomotivePedalConfig _config = AutomotivePedalConfig_init_default;
};
