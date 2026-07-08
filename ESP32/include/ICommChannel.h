#pragma once
#include <Arduino.h>
#include <MessageTools.h>

class ICommChannel {
    public:
        typedef std::function<void(const uint8_t *data, size_t len)> OnGatewayPayload;
        typedef std::function<void(AxisID axis_id, const uint8_t *data, size_t len)> OnAxisPayload;
        typedef std::function<void(const FFBAction &action)> OnFFBAction;
        typedef std::function<void(AxisID axis_id, bool is_online)> OnAxisStateChange;
        typedef std::function<void(ICommChannel *comm_channel, bool is_online)> OnGatewayStateChange;
        typedef std::function<void(uint8_t dds_index, float phase, float hz)> OnDdsSync;

    public:
        virtual void process(void) = 0;
        virtual bool send_force_and_position(float &f_contact_point, float &x_contact_point) = 0;
        virtual bool send_message_to_gateway(const Message &message, const uint8_t *raw_data, size_t len_raw_data) = 0;
        virtual bool ready_to_receive_log_message(void) = 0;
        virtual bool update_position_limits(float x_contact_point_min, float x_contact_point_max) = 0;
        virtual bool get_force(AxisID axis_id, float &f_contact_point) = 0;
        virtual bool update_force(float &f_contact_point) = 0;
        virtual bool get_position(AxisID axis_id, float &x_contact_point) = 0;
        virtual bool get_position_limits(AxisID axis_id, float &x_contact_point_min, float &x_contact_point_max) = 0;
        virtual bool is_online(AxisID axis_id) = 0;
        virtual bool is_gateway_online(void) = 0;
        virtual bool send_message_to_axis(AxisID axis_id, const Message &message, const uint8_t *raw_data, size_t len_raw_data) = 0;
        virtual bool update_function_id(FunctionID function_id) = 0;
        virtual bool get_function_id(AxisID axis_id, FunctionID &function_id) = 0;
        virtual void set_gateway_mode(bool enable) = 0;
        // Broadcast DDS phase sync from gateway to all axes (gateway-only).
        // Channels that don't carry DDS sync (e.g. USB) implement as no-op.
        virtual bool send_dds_sync(float dds1_hz, float dds1_phase,
                                   float dds2_hz, float dds2_phase) = 0;
    };