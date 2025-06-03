#pragma once
#include <Arduino.h>
#include <FFBDataTools.h>

class IGatewayCommChannel {
    protected:
        typedef std::function<void(AxisID axis_id, uint8_t *data, uint32_t len)> OnAxisPayload;

    public:
        virtual void send_ffb_data_to_axis(AxisID axis_id, const FFBData &ffb_data, const uint8_t *raw_data, uint32_t len_raw_data) = 0;
        virtual bool get_force(AxisID axis_id, float &f_foot) = 0;
        virtual bool get_position(AxisID axis_id, float &x_foot) = 0;
        virtual bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) = 0;
        virtual bool is_online(AxisID axis_id) = 0;
};