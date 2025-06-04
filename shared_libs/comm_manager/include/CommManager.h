#pragma once
#include <Arduino.h>
#include <MessageTools.h>

#include "CANManager.h"
#include "ConfigManager.h"
#include "ICommChannel.h"
#include "SerialManager.h"

enum CommChannel {
    USB_SERIAL,
    ISOTP,
    ESP_NOW
};

class AxisCommManager {
    public:
        typedef std::function<void(const FFBAction &ffb_action)> OnFFBAction;
        typedef std::function<void(AxisAction &axis_action, CommChannel comm_channel)> OnAxisAction;
        void setup(ConfigManager *config_manager, OnFFBAction on_ffb_action, OnAxisAction on_axis_action);
        bool setup_serial(Stream *serial); 
        bool setup_can(AxisID axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin);
        bool send_message(const Message &msg, CommChannel comm_channel);
        AxisID get_axis_id(void) {
            return own_axis_id;
        }

    private:
        void on_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel);
        void on_message(Message *msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel);
        AxisSerialManager serial_manager;
        CANManager can_manager;
        ConfigManager *config_manager;
        AxisID own_axis_id = AxisID_AXIS_UNDEFINED;
        OnFFBAction on_ffb_action;
        OnAxisAction on_axis_action;
};