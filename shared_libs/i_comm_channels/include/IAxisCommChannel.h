#pragma once
#include <Arduino.h>
#include <FFBDataTools.h>

class IAxisCommChannel {
    protected:
        typedef std::function<void(uint8_t *data, uint32_t len)> OnGatewayPayload;
        typedef std::function<void(FFBAction &action)> OnFFBAction;

    public:
        virtual void process(void) = 0;
        virtual void broadcast_position_limits(uint32_t now) = 0;
        virtual void send_force_and_position(float &f_foot, float &x_foot) = 0;
        virtual bool send_payload_to_gateway(const uint8_t *data, uint32_t len) = 0;
        virtual void send_position_limits(float x_foot_min, float x_foot_max) = 0;
        virtual bool get_force(AxisID axis_id, float &f_foot) = 0;
        virtual bool get_position(AxisID axis_id, float &x_foot) = 0;
        virtual bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) = 0;
        virtual bool is_online(AxisID axis_id) = 0;
};