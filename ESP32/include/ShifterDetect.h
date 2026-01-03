#pragma once
#include "IAuxFunction.h"

class ShifterDetect : public IAuxFunction {
    public:
        void process(CommManager &comm_manager, const AuxFunctionConfig &config) override;

    private:
        int8_t _active_slot_index = -1;
};
