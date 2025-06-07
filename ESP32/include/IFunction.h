#pragma once
#include <Arduino.h>
#include <CommManager.fwd.h>
#include <Physics.h>
#include <MessageTools.h>

class IFunction : public CompoundElement {
    public:
        virtual float get_x_contact_point_min(void) = 0;
        virtual float get_x_contact_point_max(void) = 0;
        virtual void on_ffb_action(const FFBAction &ffb_action) = 0;
};