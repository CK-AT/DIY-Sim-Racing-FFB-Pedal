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
            if (_config) return _config->pos_idle;
            return -1.0f;
        }
        float get_x_contact_point_max(void) override {
            if (_config) return _config->pos_end;
            return 1.0f;
        }

    private:
        SplineForceCurve force_curve = {};
        Damper damper = Damper(1.0);
        ABSOscillation abs_effect = {};
        const AutomotivePedalConfig *_config = nullptr;
};
