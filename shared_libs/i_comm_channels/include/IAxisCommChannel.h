#pragma once
#include <Arduino.h>
#include <MessageTools.h>

class IAxisCommChannel {
    protected:
        typedef std::function<void(uint8_t *data, uint32_t len)> OnGatewayPayload;
        typedef std::function<void(FFBAction &action)> OnFFBAction;

    public:
        virtual void process(void) = 0;
        virtual void send_force_and_position(float &f_foot, float &x_foot) = 0;
        virtual void send_message_to_gateway(const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) = 0;
        virtual void send_position_limits(float x_foot_min, float x_foot_max) = 0;
        virtual bool get_force(AxisID axis_id, float &f_foot) = 0;
        virtual void update_force(float &x_foot) = 0;
        virtual bool get_position(AxisID axis_id, float &x_foot) = 0;
        virtual bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) = 0;
        virtual bool is_online(AxisID axis_id) = 0;
        virtual bool is_gateway_online(void) = 0;
};