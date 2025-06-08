#pragma once
#include <Arduino.h>
#include <MessageTools.h>

class ICommChannel {
    public:
        typedef std::function<void(const uint8_t *data, size_t len)> OnGatewayPayload;
        typedef std::function<void(AxisID axis_id, const uint8_t *data, size_t len)> OnAxisPayload;
        typedef std::function<void(const FFBAction &action)> OnFFBAction;

    public:
        virtual void process(void) = 0;
        virtual bool send_force_and_position(float &f_foot, float &x_foot) = 0;
        virtual bool send_message_to_gateway(const Message &message, const uint8_t *raw_data, size_t len_raw_data) = 0;
        virtual bool update_position_limits(float x_foot_min, float x_foot_max) = 0;
        virtual bool get_force(AxisID axis_id, float &f_foot) = 0;
        virtual bool update_force(float &f_foot) = 0;
        virtual bool get_position(AxisID axis_id, float &x_foot) = 0;
        virtual bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) = 0;
        virtual bool is_online(AxisID axis_id) = 0;
        virtual bool is_gateway_online(void) = 0;
        virtual bool send_message_to_axis(AxisID axis_id, const Message &message, const uint8_t *raw_data, size_t len_raw_data) = 0;
        virtual bool update_function_id(FunctionID function_id) = 0;
        virtual bool get_function_id(AxisID axis_id, FunctionID &function_id) = 0;
        virtual void set_gateway_mode(bool enable) = 0;
    };