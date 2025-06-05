#pragma once
#include <Arduino.h>
#include <MessageTools.h>

#include "CANManager.h"
#include "CommManager.fwd.h"
#include "ConfigManager.fwd.h"
#include "ICommChannel.h"
#include "SerialManager.h"

class AxisCommManager {
    public:
        typedef std::function<void(const FFBAction &ffb_action)> OnFFBAction;
        typedef std::function<void(AxisAction &axis_action)> OnAxisAction;
        void setup(ConfigManager *config_manager, OnFFBAction on_ffb_action, OnAxisAction on_axis_action);
        bool setup_serial(Stream *serial);
        bool setup_can(AxisID axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin);
        bool send_message(const Message &msg, bool broadcast = false);
        AxisID get_axis_id(void) {
            return own_axis_id;
        }
        void process(void);
        bool send_force_and_position(float &f_foot, float &x_foot);
        bool send_message_to_gateway(const Message &message) {
            return send_message(message);
        }
        bool send_position_limits(float x_foot_min, float x_foot_max);
        bool get_force(AxisID axis_id, float &f_foot);
        bool update_force(float &f_foot);
        bool get_position(AxisID axis_id, float &x_foot);
        bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max);
        bool is_online(AxisID axis_id);
        bool is_gateway_online(void);
        bool send_message_to_axis(AxisID axis_id, const Message &message);

    private:
        void on_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel);
        void on_message(Message *msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel);
        void send_log_msg(const char *buff);
        void pump_log(int max_samples, int timeout = 0);
        void periodic_task_func(void);
        static void periodic_task(void *pvParameters) {
            AxisCommManager *logOutput = (AxisCommManager *)pvParameters;
            for (;;) {
                logOutput->periodic_task_func();
                delay(1);
            }
        }

        AxisSerialManager serial_manager;
        CANManager can_manager;
        ConfigManager *config_manager;
        AxisID own_axis_id = AxisID_AXIS_UNDEFINED;
        float _f_foot_own = 0.0f;
        float _x_foot_own = 0.0f;
        OnFFBAction on_ffb_action;
        OnAxisAction on_axis_action;
        Message log_msg = Message_init_zero;
        ICommChannel *active_gateway = nullptr;
};