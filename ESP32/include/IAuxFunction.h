#pragma once
#include <Arduino.h>
#include <CommManager.fwd.h>
#include <Physics.h>
#include <MessageTools.h>

class IAuxFunction {
    public:
        virtual void process(CommManager &comm_manager, const AuxFunctionConfig &config) = 0;
};