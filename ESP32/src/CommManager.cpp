#include <CANManager.h>
#include <CommManager.h>
#include <ConfigManager.h>
#include <Joystick_ESP32S2.h>
#include <LogOutput.h>
#include <SerialManager.h>
#include <Version.h>
#include <Version_Board.h>

#include "queue.h"

// RTDebugOutputService debugOutput = RTDebugOutputService();
QueueHandle_t _log_queue_data;

Joystick_ _joystick = Joystick_(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, 1, 0,  // Button Count, Hat Switch Count
                                true, true, true,                                         // X, Y, Z
                                true, true, true,                                         // Rx, Ry, Rz
                                true, true,                                               // rudder, throttle
                                true, true, true);                                        // accelerator, brake, steering

void CommManager::periodic_task_func(void) {
    if (!_config_manager_initialized && (_config_manager->get_mode() != ConfigManager::MODE_UNDEFINED)) {
        setup_can(_can_config);
        setup_joystick();
        _config_manager_initialized = true;
    } else {
        if (!_physics_task_started) {
            // process() is usualy called by the physics task but it has not been started (yet), so call process() here
            process();
        }
        can_manager.process_isotp();
        uint32_t now = micros();
        if ((now - ti_last_state_updates) > 100000) {
            ti_last_state_updates = now;
            uint8_t online_flags = 0;
            if (is_gateway()) {
                for (uint8_t axis_idx = 0; axis_idx < MessageTools::MAX_AXES_COUNT; axis_idx++) {
                    AxisID axis_id = MessageTools::axis_id_from_index(axis_idx);
                    send_axis_state_message(axis_id, online_flags);
                }
            } else {
                send_axis_state_message(get_axis_id(), online_flags);
            }
            send_gateway_state_message(_config_manager->get_gateway_id(), online_flags);
        }
        if ((now - ti_last_joystick_update) > 10000) {
            ti_last_joystick_update = now;
            send_joystick_values();
        }
    }
    update_joystick_state();
    update_ota_state();
    pump_log(5);
}

void CommManager::setup_joystick() {
    USB.PID(0x8211);
    USB.VID(0x303b);
    if (_is_gateway) {
        snprintf(_usb_product_name, sizeof(_usb_product_name) - 1, "DIY-FFB-Gateway-%d", get_gateway_id());
    } else {
        snprintf(_usb_product_name, sizeof(_usb_product_name) - 1, "DIY-FFB-Axis-%d", get_axis_id());
    }
    USB.productName(_usb_product_name);
    USB.manufacturerName("OpenSource");
    USB.begin();
    switch_joystick_state(JOYSTICK_USB_UP);
}

void CommManager::update_joystick_state() {
    switch (_joystick_state) {
        case JOYSTICK_USB_UP:
            if ((micros() - _ti_joystick_state) > 1000000) {
                _joystick.setXAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setYAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setZAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setRxAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setRyAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setRzAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setRudderRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setThrottleRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setAcceleratorRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setBrakeRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.setSteeringRange(JOYSTICK_MIN, JOYSTICK_MAX);
                _joystick.begin(false);
                switch_joystick_state(JOYSTICK_PRE_READY);
            }
            break;
        case JOYSTICK_PRE_READY:
            if ((micros() - _ti_joystick_state) > 100000) {
                switch_joystick_state(JOYSTICK_READY);
            }
        default:
            break;
    }
}

void CommManager::update_ota_state() {
    ESP32OTAPull::ErrorCode result;
    switch (_ota_state) {
        case OTA_PREPARE_WIFI:
            if ((micros() - _ti_ota_state) > 100000) {
                // TODO: Deinit ESPNow
                LogOutput::printf("OTA: Initializing WiFi...");
                WiFi.begin(_wifi_info.ssid, _wifi_info.password);
                switch_ota_state(OTA_WAIT_FOR_WIFI);
            }
            break;
        case OTA_WAIT_FOR_WIFI:
            if (WiFi.isConnected()) {
                LogOutput::printf("OTA: WiFi online");
                switch_ota_state(OTA_CHECK);
            } else if ((micros() - _ti_ota_state) > 5000000) {
                LogOutput::printf("OTA: Failed to connect to WiFi within 5s");
                switch_ota_state(OTA_ERROR);
            }
            break;
        case OTA_CHECK:
            if ((micros() - _ti_ota_state) > 100000) {
                ota.OverrideBoard(CONTROL_BOARD);
                result = ESP32OTAPull::ErrorCode(ota.CheckForOTAUpdate(_ota_url.c_str(), VERSION, ESP32OTAPull::ActionType::DONT_DO_UPDATE));
                switch (result) {
                    case ESP32OTAPull::ErrorCode::HTTP_FAILED:
                        LogOutput::printf("OTA: HTTP failed");
                        switch_ota_state(OTA_ERROR);
                        break;
                    case ESP32OTAPull::ErrorCode::JSON_PROBLEM:
                        LogOutput::printf("OTA: JSON problem");
                        switch_ota_state(OTA_ERROR);
                        break;
                    case ESP32OTAPull::ErrorCode::NO_UPDATE_AVAILABLE:
                        LogOutput::printf("OTA: No update available");
                        switch_ota_state(OTA_ERROR);
                        break;
                    case ESP32OTAPull::ErrorCode::NO_UPDATE_PROFILE_FOUND:
                        LogOutput::printf("OTA: No update profile found");
                        switch_ota_state(OTA_ERROR);
                        break;
                    case ESP32OTAPull::ErrorCode::UPDATE_AVAILABLE:
                        LogOutput::printf("OTA: Update available, installing...");
                        switch_ota_state(OTA_UPDATE);
                        break;
                    default:
                        LogOutput::printf("OTA: Negative HTTP response: %d", int(result));
                        switch_ota_state(OTA_ERROR);
                        break;
                }
            }
            break;
        case OTA_UPDATE:
            if ((micros() - _ti_ota_state) > 100000) {
                result = ESP32OTAPull::ErrorCode(ota.CheckForOTAUpdate(_ota_url.c_str(), VERSION));
                switch (result) {
                    case ESP32OTAPull::ErrorCode::OTA_UPDATE_FAIL:
                        LogOutput::printf("OTA: Failed to begin update");
                        switch_ota_state(OTA_ERROR);
                        break;
                    case ESP32OTAPull::ErrorCode::WRITE_ERROR:
                        LogOutput::printf("OTA: Write error");
                        switch_ota_state(OTA_ERROR);
                        break;
                    default:
                        LogOutput::printf("OTA: Negative HTTP response: %d", int(result));
                        switch_ota_state(OTA_ERROR);
                        break;
                }
            }
            break;
        case OTA_ERROR:
            WiFi.disconnect(true);
            switch_ota_state(OTA_IDLE);
            break;
        default:
            break;
    }
}

void CommManager::send_gateway_state_message(GatewayID gateway_id, uint8_t online_flags) {
    _state_message.which_payload = Message_gateway_state_tag;
    _state_message.payload.gateway_state.axes_present = online_flags;
    _state_message.payload.gateway_state.rssi = 255;  // TODO: make this depend on active_intercom_channel
    _state_message.payload.gateway_state.gateway_id = gateway_id;
    send_message_to_host(_state_message);
}

bool CommManager::send_axis_state_message(AxisID axis_id, uint8_t &online_flags) {
    if (!MessageTools::check_axis_id(axis_id)) return false;
    if (is_online(axis_id)) {
        online_flags |= (1 << MessageTools::axis_index_from_id(axis_id));
        _state_message.which_payload = Message_axis_state_tag;
        _state_message.payload.axis_state.axis_id = axis_id;
        get_force(axis_id, _state_message.payload.axis_state.force);
        get_position(axis_id, _state_message.payload.axis_state.position);
        send_message_to_host(_state_message);
    }
    return true;
}

void CommManager::setup(Stream *serial, CANConfig &can_config, ConfigManager *config_manager, OnFFBAction on_ffb_action,
                        OnAxisAction on_axis_action) {
    _config_manager = config_manager;
    _on_ffb_action = on_ffb_action;
    _on_axis_action = on_axis_action;
    _can_config = can_config;
    _log_queue_data = xQueueCreate(20, MAX_LOG_LINE_LENGTH);
    setup_serial(serial);
    xTaskCreatePinnedToCore(this->periodic_task, "CommManagerTask", 8000, this, 1, NULL, 0);
}

void CommManager::pump_log(int max_samples, int timeout) {
    char buff[MAX_LOG_LINE_LENGTH];
    while (max_samples) {
        if (has_gateway()) {
            if (!active_uplink_channel->ready_to_receive_log_message()) return;
        }
        if (pdTRUE == xQueueReceive(_log_queue_data, buff, /*xTicksToWait=*/timeout)) {
            if (_is_gateway) {
                send_gateway_log_msg(buff);
            } else {
                send_axis_log_msg(buff);
            }
        } else {
            break;
        }
        max_samples--;
    }
}

void CommManager::send_axis_log_msg(const char *buff) {
    log_msg.payload.axis_log_message.axis_id = get_axis_id();
    log_msg.which_payload = Message_axis_log_message_tag;
    memset(log_msg.payload.axis_log_message.msg, 0, sizeof(log_msg.payload.axis_log_message.msg));
    strncpy(log_msg.payload.axis_log_message.msg, buff, sizeof(log_msg.payload.axis_log_message.msg) - 1);
    if (has_gateway()) {
        send_message_to_gateway(log_msg, CommChannel::ISOTP);
    }
    send_message_to_host(log_msg);
}

void CommManager::send_gateway_log_msg(const char *buff) {
    log_msg.payload.gateway_log_message.gateway_id = get_gateway_id();
    log_msg.which_payload = Message_gateway_log_message_tag;
    memset(log_msg.payload.gateway_log_message.msg, 0, sizeof(log_msg.payload.gateway_log_message.msg));
    strncpy(log_msg.payload.gateway_log_message.msg, buff, sizeof(log_msg.payload.gateway_log_message.msg) - 1);
    send_message_to_host(log_msg);
}

bool CommManager::setup_serial(Stream *serial) {
    return serial_manager.setup(serial, this,
                                [this](const uint8_t *buffer, size_t size) { on_gateway_packet_received(buffer, size, CommChannel::USB_SERIAL); });
};

void CommManager::on_gateway_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel) {
    Message msg = Message_init_zero;
    uint16_t crc = *reinterpret_cast<const uint16_t *>(buffer + size - sizeof(uint16_t));
    if (MessageTools::check_and_decode_message(msg, buffer, size - sizeof(uint16_t), crc)) {
        on_gateway_message(msg, buffer, size, comm_channel);
    }
}

void CommManager::on_gateway_message(const Message &msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel) {
    ConfigManager::UpdateResult update_result;
    bool force;
    switch (msg.which_payload) {
        case Message_axis_config_tag:
            if (_config_manager->get_mode() == ConfigManager::MODE_GATEWAY_ONLY) {
                send_message_to_axis(msg.payload.axis_config.axis_id, msg, protobuf_msg, len_protobuf_msg);
            } else {
                force = (comm_channel == CommChannel::USB_SERIAL) && !is_gateway();
                update_result = _config_manager->update_axis_config(msg.payload.axis_config, protobuf_msg, len_protobuf_msg, force);
                if (update_result == ConfigManager::UpdateResult::UPDATE_OTHER_AXIS) {
                    send_message_to_axis(msg.payload.axis_config.axis_id, msg, protobuf_msg, len_protobuf_msg);
                }
            }
            break;
        case Message_function_config_tag:
            if (_config_manager->is_axis()) {
                if (_config_manager->update_function_config(msg.payload.function_config, protobuf_msg, len_protobuf_msg) ==
                    ConfigManager::UPDATE_OK) {
                    send_active_function_message(comm_channel);
                }
            } else {
                // gateway only, no need to call _config_manager->update_function_config()
                _config_manager->update_lookup_tables(msg.payload.function_config);
            }
            if (is_gateway() && (comm_channel == CommChannel::USB_SERIAL)) {
                const AxisID *linked_axes = msg.payload.function_config.base.linked_axes;
                for (uint8_t idx = 0; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
                    auto linked_axis_id = AxisID(linked_axes[idx] & AxisID_AXIS_ID_MASK);
                    send_message_to_axis(linked_axis_id, msg, protobuf_msg, len_protobuf_msg);
                }
            }
            break;
        case Message_ffb_action_tag:
            if (_config_manager->is_axis()) {
                on_ffb_action(msg.payload.ffb_action);
            }
            if (is_gateway() && (comm_channel == CommChannel::USB_SERIAL)) {
                // only the primary axis will process FFB actions, no need to send it to other axes
                AxisID primary_axis_id = _config_manager->get_primary_axis_id(msg.payload.ffb_action.function_id);
                if (MessageTools::check_axis_id(primary_axis_id)) {
                    send_message_to_axis(primary_axis_id, msg, protobuf_msg, len_protobuf_msg);
                }
            }
            break;
        case Message_axis_action_tag:
            if (_config_manager->is_axis() && _on_axis_action && (msg.payload.axis_config.axis_id == _config_manager->get_axis_id())) {
                switch (msg.payload.axis_action.which_action) {
                    case AxisAction_return_axis_config_tag:
                        send_axis_config(comm_channel);
                        break;
                    case AxisAction_return_function_config_tag:
                        send_function_config(comm_channel);
                        break;
                    case AxisAction_return_active_function_tag:
                        send_active_function_message(comm_channel);
                    default:
                        if (_on_axis_action) {
                            _on_axis_action(msg.payload.axis_action, comm_channel);
                        }
                        break;
                }
            }
            if (is_gateway() && (comm_channel == CommChannel::USB_SERIAL)) {
                if (!send_message_to_axis(msg.payload.axis_config.axis_id, msg, protobuf_msg, len_protobuf_msg)) {
                    LogOutput::printf("Can't forward AxisAction: axis %d is offline", msg.payload.axis_config.axis_id);
                }
            }
            break;
        case Message_start_ota_update_tag:
            if (is_gateway() && (comm_channel == CommChannel::USB_SERIAL)) {
                for (int axis_idx = 0; axis_idx < MessageTools::MAX_AXES_COUNT; axis_idx++) {
                    send_message_to_axis(MessageTools::axis_id_from_index(axis_idx), msg);
                }
            }
            ota.AllowDowngrades(msg.payload.start_ota_update.allow_downgrades);
            _wifi_info = msg.payload.start_ota_update.wifi_info;
            _ota_url = msg.payload.start_ota_update.info_json_url;
            switch_ota_state(OtaState::OTA_PREPARE_WIFI);
            break;
        default:
            LogOutput::printf("Unknown Message received");
            break;
    }
}

void CommManager::send_active_function_message(AxisID axis_id, FunctionID function_id, CommChannel comm_channel) {
    Message msg;
    msg.which_payload = Message_active_function_tag;
    msg.payload.active_function.axis_id = axis_id;
    msg.payload.active_function.function_id = function_id;
    send_message_to_gateway(msg, comm_channel);
}

void CommManager::send_active_function_message(CommChannel comm_channel) {
    send_active_function_message(_config_manager->get_axis_id(), _config_manager->get_function_id(), comm_channel);
}

void CommManager::send_axis_config(CommChannel comm_channel) {
    Message msg;
    _config_manager->get_axis_config_as_message(msg);
    send_message_to_gateway(msg, comm_channel);
}

void CommManager::send_function_config(CommChannel comm_channel) {
    Message msg;
    _config_manager->get_function_config_as_message(msg);
    send_message_to_gateway(msg, comm_channel);
}

void CommManager::on_axis_packet_received(AxisID axis_id, const uint8_t *data, size_t len, CommChannel comm_channel) {
    Message msg = Message_init_zero;
    uint16_t crc = *reinterpret_cast<const uint16_t *>(data + len - sizeof(uint16_t));
    if (MessageTools::check_and_decode_message(msg, data, len - sizeof(uint16_t), crc)) {
        on_axis_message(axis_id, msg, data, len, comm_channel);
    }
}

void CommManager::on_axis_message(AxisID axis_id, const Message &msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg,
                                  CommChannel comm_channel) {
    switch (msg.which_payload) {
        case Message_function_config_tag:
            _config_manager->update_lookup_tables(msg.payload.function_config);
            send_active_function_message(axis_id, msg.payload.function_config.base.function_id, CommChannel::USB_SERIAL);
            break;
    }
    serial_manager.send_message_to_host(msg, protobuf_msg, len_protobuf_msg);
}

bool CommManager::setup_can(CANConfig &config) {
    AxisID axis_id = get_axis_id();
    _is_axis = MessageTools::check_axis_id(axis_id);
    _is_gateway = !_is_axis;
    can_manager.setup(axis_id, config.baud_rate, config.tx_pin, config.rx_pin,
                      std::bind(&CommManager::on_gateway_packet_received, this, std::placeholders::_1, std::placeholders::_2, CommChannel::ISOTP),
                      std::bind(&CommManager::on_ffb_action, this, std::placeholders::_1),
                      std::bind(&CommManager::on_axis_packet_received, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                CommChannel::ISOTP),
                      std::bind(&CommManager::on_axis_state_change, this, std::placeholders::_1, std::placeholders::_2),
                      std::bind(&CommManager::on_gateway_state_change, this, std::placeholders::_1, std::placeholders::_2));
    active_intercom_channel = &can_manager;
    if (_is_gateway) {
        active_downlink_channel = &can_manager;
        can_manager.set_gateway_mode(true);
    }
    return true;
}

void CommManager::on_axis_state_change(AxisID axis_id, bool is_online) {
    if (axis_id == get_axis_id()) return;
    if (!is_gateway()) return;
    if (_ota_state != OTA_IDLE) return;
    if (is_online) {
        LogOutput::printf("CommManager: Axis %d online, requesting function config", axis_id);
        if (!is_gateway()) return;
        Message msg = Message_init_default;
        msg.which_payload = Message_axis_action_tag;
        msg.payload.axis_action.axis_id = axis_id;
        msg.payload.axis_action.which_action = AxisAction_return_function_config_tag;
        msg.payload.axis_action.action.return_function_config = true;
        send_message_to_axis(axis_id, msg);
    } else {
        LogOutput::printf("CommManager: Axis %d offline", axis_id);
    }
}

void CommManager::on_gateway_state_change(ICommChannel *comm_channel, bool is_online) {
    if (is_online) {
        if (!active_uplink_channel) {
            LogOutput::printf("CommManager: Uplink online");
            active_uplink_channel = comm_channel;
        }
    } else if (active_uplink_channel == comm_channel) {
        LogOutput::printf("CommManager: Uplink lost");
        active_uplink_channel = nullptr;
    }
}

void CommManager::process(void) {
    can_manager.process();
}

bool CommManager::send_force_and_position(float &f_contact_point, float &x_contact_point) {
    _f_contact_point_own = f_contact_point;
    _x_contact_point_own = x_contact_point;
    can_manager.send_force_and_position(f_contact_point, x_contact_point);
    return true;
}

bool CommManager::update_position_limits(float x_contact_point_min, float x_contact_point_max) {
    can_manager.update_position_limits(x_contact_point_min, x_contact_point_max);
    return true;
}

bool CommManager::update_function_id(FunctionID function_id) {
    can_manager.update_function_id(function_id);
    return true;
}

bool CommManager::get_force(AxisID axis_id, float &f_contact_point) {
    if (axis_id == get_axis_id()) {
        f_contact_point = _f_contact_point_own;
        return true;
    }
    if (!active_intercom_channel) return false;
    return active_intercom_channel->get_force(axis_id, f_contact_point);
}

bool CommManager::update_force(float &f_contact_point) {
    _f_contact_point_own = f_contact_point;
    return true;
}

bool CommManager::get_position(AxisID axis_id, float &x_contact_point) {
    if (axis_id == get_axis_id()) {
        x_contact_point = _x_contact_point_own;
        return true;
    }
    if (!active_intercom_channel) return false;
    return active_intercom_channel->get_position(axis_id, x_contact_point);
}

bool CommManager::get_position_limits(AxisID axis_id, float &x_contact_point_min, float &x_contact_point_max) {
    if (!active_intercom_channel) return false;
    return active_intercom_channel->get_position_limits(axis_id, x_contact_point_min, x_contact_point_max);
}

bool CommManager::get_function_id(AxisID axis_id, FunctionID &function_id) {
    if (!active_intercom_channel) return false;
    return active_intercom_channel->get_function_id(axis_id, function_id);
}

bool CommManager::is_online(AxisID axis_id) {
    if (axis_id == get_axis_id()) return true;
    if (!active_intercom_channel) return false;
    return active_intercom_channel->is_online(axis_id);
}

bool CommManager::send_message_to_axis(AxisID axis_id, const Message &message) {
    if (!is_gateway()) return false;
    if (!active_downlink_channel->is_online(axis_id)) return false;
    uint8_t tx_buffer[MessageTools::MAX_ENCODED_SIZE + 2];
    uint16_t crc;
    uint16_t num_bytes_encoded = MessageTools::encode_message_and_calc_crc(message, tx_buffer, MessageTools::MAX_ENCODED_SIZE, crc);
    if (num_bytes_encoded) {
        memcpy(tx_buffer + num_bytes_encoded, &crc, sizeof(uint16_t));
        return active_downlink_channel->send_message_to_axis(axis_id, message, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
    }
    return false;
}

bool CommManager::send_message_to_axis(AxisID axis_id, const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) {
    if (!is_gateway()) return false;
    if (!active_downlink_channel->is_online(axis_id)) return false;
    return active_downlink_channel->send_message_to_axis(axis_id, message, raw_data, len_raw_data);
}

bool CommManager::send_message_to_gateway(const Message &message, CommChannel comm_channel) {
    uint8_t tx_buffer[MessageTools::MAX_ENCODED_SIZE + 2];
    uint16_t crc;
    uint16_t num_bytes_encoded = MessageTools::encode_message_and_calc_crc(message, tx_buffer, MessageTools::MAX_ENCODED_SIZE, crc);
    if (!num_bytes_encoded) return false;
    memcpy(tx_buffer + num_bytes_encoded, &crc, sizeof(uint16_t));
    if (comm_channel == CommChannel::USB_SERIAL) {
        return serial_manager.send_message_to_host(message, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
    }
    bool result = false;
    if (has_gateway()) {
        result = active_uplink_channel->send_message_to_gateway(message, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
    }
    return result;
}

bool CommManager::send_message_to_host(const Message &message) {
    uint8_t tx_buffer[MessageTools::MAX_ENCODED_SIZE + 2];
    uint16_t crc;
    uint16_t num_bytes_encoded = MessageTools::encode_message_and_calc_crc(message, tx_buffer, MessageTools::MAX_ENCODED_SIZE, crc);
    memcpy(tx_buffer + num_bytes_encoded, &crc, sizeof(uint16_t));
    if (!num_bytes_encoded) return false;
    return serial_manager.send_message_to_host(message, tx_buffer, num_bytes_encoded + sizeof(uint16_t));
}

AxisID CommManager::get_axis_id(void) {
    return _config_manager->get_axis_id();
}

GatewayID CommManager::get_gateway_id(void) {
    return _config_manager->get_gateway_id();
}

void CommManager::on_ffb_action(const FFBAction &ffb_action) {
    if (ffb_action.function_id != _config_manager->get_function_id()) return;
    if (_on_ffb_action) {
        _on_ffb_action(ffb_action);
    }
}

bool CommManager::calc_controller_output_value(FunctionBase &function_base, float &controller_output) {
    AxisID primary_axis_id = AxisID(function_base.linked_axes[0] & AxisID_AXIS_ID_MASK);
    float src_value;
    bool success = false;
    switch (function_base.output_mode) {
        case OutputMode_OUTPUT_MODE_FORCE:
            success = get_force(primary_axis_id, src_value);
            break;
        case OutputMode_OUTPUT_MODE_TRAVEL:
            success = get_position(primary_axis_id, src_value);
            break;
        default:
            break;
    }
    if (!success) return false;
    controller_output = normalize_value(src_value, function_base.output_min, function_base.output_max);
    return true;
}

void CommManager::set_controller_axis(ControllerAxis controller_axis, float &value) {
    uint16_t final_value = uint16_t(value * float(JOYSTICK_MAX));
    switch (controller_axis) {
        case ControllerAxis_CONTROLLER_AXIS_X:
            _joystick.setXAxis(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_Y:
            _joystick.setYAxis(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_Z:
            _joystick.setZAxis(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_R_X:
            _joystick.setRxAxis(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_R_Y:
            _joystick.setRyAxis(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_R_Z:
            _joystick.setRzAxis(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_RUD:
            _joystick.setRudder(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_THR:
            _joystick.setThrottle(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_ACC:
            _joystick.setAccelerator(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_BRK:
            _joystick.setBrake(final_value);
            break;
        case ControllerAxis_CONTROLLER_AXIS_STEER:
            _joystick.setSteering(final_value);
            break;
        default:
            break;
    }
}

void CommManager::send_joystick_values(void) {
    if (_joystick_state != JOYSTICK_READY) return;
    uint16_t function_flags;
    FunctionID function_id;
    for (uint8_t axis_idx = 0; axis_idx < MessageTools::MAX_AXES_COUNT; axis_idx++) {
        if (get_function_id(MessageTools::axis_id_from_index(axis_idx), function_id)) {
            function_flags |= 1 << function_id;
        }
    }
    float output_value;
    memset(controller_axis_values, 0, sizeof(controller_axis_values));
    for (uint8_t function_idx = 0; function_idx < _FunctionID_MAX; function_idx++) {
        function_id = FunctionID(function_idx + 1);
        if (function_flags & (1 << function_id)) {
            FunctionBase *function_base = _config_manager->get_function_base(function_id);
            if (function_base && function_base->controller_output_axis) {
                if (calc_controller_output_value(*function_base, output_value)) {
                    if (MessageTools::check_controller_axis_id(function_base->controller_output_axis)) {
                        controller_axis_values[MessageTools::controller_axis_index_from_id(function_base->controller_output_axis)] = output_value;
                    }
                }
            }
        }
    }
    for (uint8_t function_idx = 0; function_idx < _FunctionID_MAX; function_idx++) {
        function_id = FunctionID(function_idx + 1);
        if (function_flags & (1 << function_id)) {
            auto result = _config_manager->get_aux_function(function_id);
            if (std::get<0>(result)) {
                std::get<0>(result)->process(*this, std::get<1>(result));
            }
        }
    }
    for (uint8_t controller_axis_idx = 0; controller_axis_idx < _ControllerAxis_MAX; controller_axis_idx++) {
        ControllerAxis controller_axis = MessageTools::controller_axis_id_from_index(controller_axis_idx);
        set_controller_axis(controller_axis, controller_axis_values[controller_axis_idx]);
    }
    _joystick.sendState();
}

bool CommManager::calc_input_force_sum(const AxisID *linked_axes, float &input_force) {
    float f_sum = 0.0f;
    float temp;
    bool is_subtractive_axis = false;
    AxisID own_axis_id = get_axis_id();
    for (uint8_t idx = 0; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
        AxisID axis_id = AxisID(linked_axes[idx] & AxisID_AXIS_ID_MASK);
        if (axis_id == AxisID_AXIS_UNDEFINED) break;
        temp = 0.0f;
        get_force(axis_id, temp);  // get_force won't touch temp if the associated axis is not online, no need to check the return value
        if (linked_axes[idx] & AxisID_AXIS_SUBTRACTIVE) {
            if (axis_id == own_axis_id) is_subtractive_axis = true;
            f_sum -= temp;
        } else {
            f_sum += temp;
        }
    }
    input_force = f_sum;
    return is_subtractive_axis;
}

bool CommManager::calc_input_force_sum(float &input_force) {
    return calc_input_force_sum(_config_manager->get_function_config()->base.linked_axes, input_force);
}

bool CommManager::calc_final_position(float own_position, float &final_position) {
    const FunctionBase &func_base = _config_manager->get_function_config()->base;
    float other_position;
    AxisID primary_axis_id = AxisID(func_base.linked_axes[0] & AxisID_AXIS_ID_MASK);
    AxisID own_axis_id = get_axis_id();
    if (primary_axis_id == own_axis_id) {
        // we are the primary axis -> own_position is the final position
        final_position = own_position;
        return true;
    } else if (get_position(primary_axis_id, other_position)) {
        // we are NOT the primary axis, start at idx 1
        for (uint8_t idx = 1; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
            AxisID axis_id = AxisID(func_base.linked_axes[idx] & AxisID_AXIS_ID_MASK);
            if (axis_id == own_axis_id) {
                if (func_base.linked_axes[idx] & AxisID_AXIS_SUBTRACTIVE) {
                    final_position = (_config_manager->get_x_contact_point_center() * 2.0f) - other_position;
                    return true;
                } else {
                    final_position = other_position;
                    return true;
                }
            } else if (axis_id == AxisID_AXIS_UNDEFINED) {
                break;
            }
        }
    }
    return false;
}
