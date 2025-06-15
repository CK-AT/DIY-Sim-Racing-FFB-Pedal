#pragma once
#include "ABSOscillation.h"
#include "Arduino.h"
#include "CommManager.h"
#include "ForceCurve.h"
#include "IAuxFunction.h"
#include "Physics.h"

class RudderBrake : public IAuxFunction {
    public:
        RudderBrake(void);
        void process(CommManager &comm_manager, const AuxFunctionConfig &config) override;
};
