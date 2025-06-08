#pragma once
#include <Arduino.h>
#include <MessageTools.h>

#include "CANManager.h"
#include "CommManager.fwd.h"
#include "ConfigManager.fwd.h"
#include "ICommChannel.h"
#include "SerialManager.h"

class CommManager {
    public:
        struct CANConfig {
            uint16_t baud_rate;
            int8_t tx_pin;
            int8_t rx_pin;
        };

        typedef std::function<void(const FFBAction &ffb_action)> OnFFBAction;
        typedef std::function<void(const AxisAction &axis_action, CommChannel comm_channel)> OnAxisAction;
        void setup(Stream *serial, CANConfig &can_config, ConfigManager *config_manager, OnFFBAction on_ffb_action, OnAxisAction on_axis_action);
        AxisID get_axis_id(void);
        GatewayID get_gateway_id(void);
        void process(void);
        bool send_force_and_position(float &f_foot, float &x_foot);
        bool send_message_to_gateway(const Message &message, CommChannel comm_channel);
        bool send_message_to_host(const Message &message);
        bool update_position_limits(float x_foot_min, float x_foot_max);
        bool update_function_id(FunctionID function_id);
        bool get_force(AxisID axis_id, float &f_foot);
        bool update_force(float &f_foot);
        bool get_position(AxisID axis_id, float &x_foot);
        bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max);
        bool get_function_id(AxisID axis_id, FunctionID &function_id);
        bool is_online(AxisID axis_id);
        bool send_message_to_axis(AxisID axis_id, const Message &message);
        bool calc_controller_output_value(FunctionBase &function_base, float &controller_output);
        bool is_gateway(void) {
            return active_downlink_channel != nullptr;
        }
        bool has_gateway(void) {
            return active_uplink_channel != nullptr;
        }

    private:
        bool setup_can(CANConfig &can_config);
        bool setup_serial(Stream *serial);
        void on_gateway_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel);
        void on_gateway_message(const Message &msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel);
        void on_axis_packet_received(AxisID axis_id, const uint8_t *data, size_t len, CommChannel comm_channel);
        void on_axis_message(AxisID axis_id, const Message &msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel);
        bool send_message_to_axis(AxisID axis_id, const Message &message, const uint8_t *raw_data, uint32_t len_raw_data);
        void send_axis_log_msg(const char *buff);
        void send_gateway_log_msg(const char *buff);
        void pump_log(int max_samples, int timeout = 0);
        void on_ffb_action(const FFBAction &ffb_action);
        void periodic_task_func(void);
        void send_gateway_state_message(uint8_t online_flags);
        bool send_axis_state_message(AxisID axis_id, uint8_t &online_flags);
        static void periodic_task(void *pvParameters) {
            CommManager *logOutput = (CommManager *)pvParameters;
            for (;;) {
                logOutput->periodic_task_func();
                delay(1);
            }
        }

        SerialManager serial_manager;
        CANManager can_manager;
        ConfigManager *_config_manager;
        float _f_contact_point_own = 0.0f;
        float _x_contact_point_own = 0.0f;
        uint32_t ti_last_state_updates = 0;
        OnFFBAction _on_ffb_action;
        OnAxisAction _on_axis_action;
        Message log_msg = Message_init_zero;
        Message _state_message = Message_init_default;
        ICommChannel *active_uplink_channel = nullptr;
        ICommChannel *active_intercom_channel = nullptr;
        ICommChannel *active_downlink_channel = nullptr;
        bool _is_axis = false;
        bool _is_gateway = false;
        bool _config_manager_initialized = false;
        CANConfig _can_config;
};