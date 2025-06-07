#include <CANManager.h>
#include <CommManager.h>
#include <ConfigManager.h>
#include <LogOutput.h>
#include <SerialManager.h>

#include "queue.h"

// RTDebugOutputService debugOutput = RTDebugOutputService();
QueueHandle_t _log_queue_data;

bool AxisCommManager::send_message(const Message &msg, bool broadcast) {
    uint8_t tx_buffer[MessageTools::MAX_ENCODED_SIZE + 2];
    uint16_t crc;
    uint16_t num_bytes_encoded = MessageTools::encode_message_and_calc_crc(msg, tx_buffer, MessageTools::MAX_ENCODED_SIZE, crc);
    if (num_bytes_encoded) {
        memcpy(tx_buffer + num_bytes_encoded, &crc, sizeof(uint16_t));
        if (can_manager.is_gateway_online()) {
            can_manager.send_message_to_gateway(msg, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
            if (!broadcast) return true;
        }
        serial_manager.send_message_to_host(msg, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
        return true;
    }
    return false;
}

void AxisCommManager::periodic_task_func(void) {
    if (get_axis_id() == AxisID_AXIS_UNDEFINED) {
        // process() is usualy called by the physics update task but we are not an axis right now, so call it here
        process();
    }
    pump_log(5);
}

void AxisCommManager::setup(ConfigManager *config_manager, OnFFBAction on_ffb_action, OnAxisAction on_axis_action) {
    _config_manager = config_manager;
    _on_ffb_action = on_ffb_action;
    _on_axis_action = on_axis_action;
    _log_queue_data = xQueueCreate(20, MAX_LOG_LINE_LENGTH);
    xTaskCreatePinnedToCore(this->periodic_task, "CommManagerTask", 2000, this, 1, NULL, 0);
}

void AxisCommManager::pump_log(int max_samples, int timeout) {
    char buff[MAX_LOG_LINE_LENGTH];
    while (max_samples && (pdTRUE == xQueueReceive(_log_queue_data, buff, /*xTicksToWait=*/timeout))) {
        if (_is_gateway) {
            send_gateway_log_msg(buff);
        } else {
            send_axis_log_msg(buff);
        }
        max_samples--;
    }
}

void AxisCommManager::send_axis_log_msg(const char *buff) {
    log_msg.payload.axis_log_message.axis_id = get_axis_id();
    log_msg.which_payload = Message_axis_log_message_tag;
    memset(log_msg.payload.axis_log_message.msg, 0, sizeof(log_msg.payload.axis_log_message.msg));
    strncpy(log_msg.payload.axis_log_message.msg, buff, sizeof(log_msg.payload.axis_log_message.msg) - 1);
    send_message(log_msg, true);
}

void AxisCommManager::send_gateway_log_msg(const char *buff) {
    log_msg.payload.gateway_log_message.gateway_id = get_gateway_id();
    log_msg.which_payload = Message_gateway_log_message_tag;
    memset(log_msg.payload.gateway_log_message.msg, 0, sizeof(log_msg.payload.gateway_log_message.msg));
    strncpy(log_msg.payload.gateway_log_message.msg, buff, sizeof(log_msg.payload.gateway_log_message.msg) - 1);
    send_message(log_msg, true);
}

bool AxisCommManager::setup_serial(Stream *serial) {
    return serial_manager.setup(serial, this,
                                [this](const uint8_t *buffer, size_t size) { on_gateway_packet_received(buffer, size, CommChannel::USB_SERIAL); });
};

void AxisCommManager::on_gateway_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel) {
    Message msg = Message_init_zero;
    uint16_t crc = *reinterpret_cast<const uint16_t *>(buffer + size - sizeof(uint16_t));
    if (MessageTools::check_and_decode_message(msg, buffer, size - sizeof(uint16_t), crc)) {
        on_gateway_message(msg, buffer, size, comm_channel);
    }
}

void AxisCommManager::on_gateway_message(const Message &msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel) {
    ConfigManager::UpdateResult update_result;
    switch (msg.which_payload) {
        case Message_axis_config_tag:
            update_result =
                _config_manager->update_axis_config(msg.payload.axis_config, protobuf_msg, len_protobuf_msg, comm_channel == CommChannel::USB_SERIAL);
            if (update_result == ConfigManager::UpdateResult::UPDATE_OTHER_AXIS) {
                if (active_gateway_channel) {
                    active_gateway_channel->send_message_to_axis(msg.payload.axis_config.axis_id, msg, protobuf_msg, len_protobuf_msg);
                }
            }
            break;
        case Message_function_config_tag:
            if ((_config_manager->get_axis_id() != AxisID_AXIS_UNDEFINED)) {
                _config_manager->update_function_config(msg.payload.function_config, protobuf_msg, len_protobuf_msg);
            }
            if (active_gateway_channel && (comm_channel == CommChannel::USB_SERIAL)) {
                const AxisID *linked_axes = msg.payload.function_config.base.linked_axes;
                for (uint8_t idx = 0; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
                    auto linked_axis_id = AxisID(linked_axes[idx] & AxisID_AXIS_ID_MASK);
                    active_gateway_channel->send_message_to_axis(linked_axis_id, msg, protobuf_msg, len_protobuf_msg);
                }
            }
            break;
        case Message_ffb_action_tag:
            if ((_config_manager->get_axis_id() != AxisID_AXIS_UNDEFINED)) {
                on_ffb_action(msg.payload.ffb_action);
            }
            if (active_gateway_channel && (comm_channel == CommChannel::USB_SERIAL)) {
                const AxisID *linked_axes = config_manager->get_function_config()->base.linked_axes;
                for (uint8_t idx = 0; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
                    auto linked_axis_id = AxisID(linked_axes[idx] & AxisID_AXIS_ID_MASK);
                    active_gateway_channel->send_message_to_axis(linked_axis_id, msg, protobuf_msg, len_protobuf_msg);
                }
            }
            break;
        case Message_axis_action_tag:
            if ((_config_manager->get_axis_id() != AxisID_AXIS_UNDEFINED) && _on_axis_action) {
                _on_axis_action(msg.payload.axis_action);
            }
            if (active_gateway_channel && (comm_channel == CommChannel::USB_SERIAL)) {
                if (active_gateway_channel->is_online(msg.payload.axis_config.axis_id)) {
                    active_gateway_channel->send_message_to_axis(msg.payload.axis_config.axis_id, msg, protobuf_msg, len_protobuf_msg);
                } else {
                    LogOutput::printf("Can't forward AxisAction: axis %d is offline", msg.payload.axis_config.axis_id);
                }
            }
            break;
        default:
            LogOutput::printf("Unknown Message received");
            break;
    }
}

void AxisCommManager::on_axis_packet_received(AxisID axis_id, const uint8_t *data, size_t len, CommChannel comm_channel) {
    Message msg = Message_init_zero;
    uint16_t crc = *reinterpret_cast<const uint16_t *>(data + len - sizeof(uint16_t));
    if (MessageTools::check_and_decode_message(msg, data, len - sizeof(uint16_t), crc)) {
        on_axis_message(axis_id, msg, data, len, comm_channel);
    }
}

void AxisCommManager::on_axis_message(AxisID axis_id, const Message &msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg,
                                      CommChannel comm_channel) {
    switch (msg.which_payload) {
        case Message_function_config_tag:
            _config_manager->update_function_config_lut(msg.payload.function_config);
            serial_manager.send_message_to_host(msg, protobuf_msg, len_protobuf_msg);
            break;
        case Message_axis_log_message_tag:
            serial_manager.send_message_to_host(msg, protobuf_msg, len_protobuf_msg);
            break;
        case Message_axis_state_tag:
            serial_manager.send_message_to_host(msg, protobuf_msg, len_protobuf_msg);
            break;
        default:
            break;
    }
}

bool AxisCommManager::setup_can(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin) {
    AxisID axis_id = get_axis_id();
    _is_axis = MessageTools::check_axis_id(axis_id);
    _is_gateway = !_is_axis;
    return can_manager.setup(
        axis_id, baud_rate, tx_pin, rx_pin,
        [this](const uint8_t *buffer, size_t size) { on_gateway_packet_received(buffer, size, CommChannel::ISOTP); },
        [this](const FFBAction &ffb_action) { on_ffb_action(ffb_action); },
        [this](AxisID axis_id, const uint8_t *buffer, size_t size) { on_axis_packet_received(axis_id, buffer, size, CommChannel::ISOTP); });
}

void AxisCommManager::process(void) {
    can_manager.process();
    if (can_manager.is_gateway_online()) {
        active_intercom_channel = &can_manager;
    }
}

bool AxisCommManager::send_force_and_position(float &f_foot, float &x_foot) {
    _f_foot_own = f_foot;
    _x_foot_own = x_foot;
    can_manager.send_force_and_position(f_foot, x_foot);
    return true;
}

bool AxisCommManager::send_position_limits(float x_foot_min, float x_foot_max) {
    can_manager.send_position_limits(x_foot_min, x_foot_max);
    return true;
}

bool AxisCommManager::get_force(AxisID axis_id, float &f_foot) {
    if (axis_id == get_axis_id()) {
        f_foot = _f_foot_own;
        return true;
    }
    if (!active_intercom_channel) return false;
    return active_intercom_channel->get_force(axis_id, f_foot);
}

bool AxisCommManager::update_force(float &f_foot) {
    _f_foot_own = f_foot;
    return true;
}

bool AxisCommManager::get_position(AxisID axis_id, float &x_foot) {
    if (axis_id == get_axis_id()) {
        x_foot = _x_foot_own;
        return true;
    }
    if (!active_intercom_channel) return false;
    return active_intercom_channel->get_position(axis_id, x_foot);
}

bool AxisCommManager::get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) {
    if (!active_intercom_channel) return false;
    return active_intercom_channel->get_position_limits(axis_id, x_foot_min, x_foot_max);
}

bool AxisCommManager::is_online(AxisID axis_id) {
    if (axis_id == get_axis_id()) return true;
    if (!active_intercom_channel) return false;
    return active_intercom_channel->is_online(axis_id);
}

bool AxisCommManager::is_gateway_online(void) {
    return active_gateway_channel != nullptr;
}

bool AxisCommManager::send_message_to_axis(AxisID axis_id, const Message &message) {
    if (!active_gateway_channel) return false;
    uint8_t tx_buffer[MessageTools::MAX_ENCODED_SIZE + 2];
    uint16_t crc;
    uint16_t num_bytes_encoded = MessageTools::encode_message_and_calc_crc(message, tx_buffer, MessageTools::MAX_ENCODED_SIZE, crc);
    if (num_bytes_encoded) {
        memcpy(tx_buffer + num_bytes_encoded, &crc, sizeof(uint16_t));
        return active_gateway_channel->send_message_to_axis(axis_id, message, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
    }
    return false;
}

AxisID AxisCommManager::get_axis_id(void) {
    return _config_manager->get_axis_id();
}

GatewayID AxisCommManager::get_gateway_id(void) {
    return _config_manager->get_gateway_id();
}

void AxisCommManager::on_ffb_action(const FFBAction &ffb_action) {
    if (ffb_action.function_id != _config_manager->get_function_id()) return;
    if (_on_ffb_action) {
        _on_ffb_action(ffb_action);
    }
}