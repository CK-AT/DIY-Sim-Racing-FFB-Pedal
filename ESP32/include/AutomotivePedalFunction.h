#pragma once
#include "ABSOscillation.h"
#include "Arduino.h"
#include "ConfigManager.h"
#include "ForceCurve.h"
#include "Physics.h"

class AutomotivePedalFunction : public CompoundElement {
    public:
        AutomotivePedalFunction(void);
        void update_config(const AutomotivePedalConfig &config);
        void trigger_abs(void) {
            abs_effect.trigger();
        }

    private:
        SplineForceCurve force_curve = {};
        Damper damper = Damper(1.0);
        ABSOscillation abs_effect = {};
};
