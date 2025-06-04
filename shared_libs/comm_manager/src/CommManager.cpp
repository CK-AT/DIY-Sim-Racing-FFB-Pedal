#include <CommManager.h>
#include <CANManager.h>
#include <SerialManager.h>
#include <LogOutput.h>

bool AxisCommManager::send_message(const Message &msg, CommChannel comm_channel) {
    uint8_t tx_buffer[MessageTools::MAX_ENCODED_SIZE + 2];
    uint16_t crc;
    uint16_t num_bytes_encoded = MessageTools::encode_message_and_calc_crc(msg, tx_buffer, MessageTools::MAX_ENCODED_SIZE, crc);
    if (num_bytes_encoded) {
        memcpy(tx_buffer + num_bytes_encoded, &crc, sizeof(uint16_t));
        switch (comm_channel) {
            case CommChannel::USB_SERIAL:
                serial_manager.send_message_to_host(msg, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
                break;
            case CommChannel::ISOTP:
                can_manager.send_message_to_gateway(msg, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
                break;
            default:
                break;
        }
        return true;
    }
    return false;
}

void AxisCommManager::setup(ConfigManager *config_manager, OnFFBAction on_ffb_action, OnAxisAction on_axis_action) {
    this->config_manager = config_manager;
    this->on_ffb_action = on_ffb_action;
    this->on_axis_action = on_axis_action;
}

bool AxisCommManager::setup_serial(Stream *serial) {
    return serial_manager.setup(serial, this, [this](const uint8_t *buffer, size_t size) { on_packet_received(buffer, size, CommChannel::USB_SERIAL); });
};

void AxisCommManager::on_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel) {
    Message msg = Message_init_zero;
    uint16_t crc = *reinterpret_cast<const uint16_t *>(buffer + size - sizeof(uint16_t));
    if (MessageTools::check_and_decode_message(msg, buffer, size - sizeof(uint16_t), crc)) {
        on_message(&msg, buffer, size - sizeof(uint16_t), comm_channel);
    }
}

void AxisCommManager::on_message(Message *msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel) {
    switch (msg->which_payload) {
        case Message_axis_config_tag:
            config_manager->update_axis_config(msg->payload.axis_config, protobuf_msg, len_protobuf_msg);
            break;
        case Message_function_config_tag:
            config_manager->update_function_config(msg->payload.function_config, protobuf_msg, len_protobuf_msg);
            break;
        case Message_ffb_action_tag:
            on_ffb_action(msg->payload.ffb_action);
            break;
        case Message_axis_action_tag:
            on_axis_action(msg->payload.axis_action, comm_channel);
            break;
        default:
            LogOutput::printf("Unknown Message received");
            break;
    }
}

bool AxisCommManager::setup_can(AxisID axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin) {
    own_axis_id = axis_id;
    return can_manager.setup(axis_id, baud_rate, tx_pin, rx_pin,
        [this](const uint8_t *buffer, size_t size) { on_packet_received(buffer, size, CommChannel::ISOTP); }, on_ffb_action, nullptr);

}

