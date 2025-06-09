#include <CANManager.h>
#include <LogOutput.h>

/*****************************************************************************************************************/
/* isotp-c shim functions */
/*****************************************************************************************************************/
extern "C" int isotp_user_send_can(const uint32_t id, const uint8_t *data, const uint8_t size) {
    CanFrame tx_frame = {};
    tx_frame.identifier = id;
    memcpy(tx_frame.data, data, size);
    tx_frame.data_length_code = size;
    if (!ESP32Can.writeFrame(&tx_frame, 0)) {
        return ISOTP_RET_NOSPACE;
    }
    return ISOTP_RET_OK;
}

extern "C" uint32_t isotp_user_get_us(void) {
    return micros();
}

extern "C" void isotp_user_debug(const char *message, ...) {
    LogOutput::printf(message);
}

/*****************************************************************************************************************/
/* CANManager */
/*****************************************************************************************************************/
bool CANManager::get_force(AxisID axis_id, float &f_foot) {
    if (!MessageTools::check_axis_id(axis_id)) return false;
    uint8_t axis_idx = MessageTools::axis_index_from_id(axis_id);
    if (axis_states[axis_idx].online) {
        f_foot = axis_states[axis_idx].force_and_position.f_foot;
        return true;
    }
    return false;
}

bool CANManager::get_position(AxisID axis_id, float &x_foot) {
    if (!MessageTools::check_axis_id(axis_id)) return false;
    uint8_t axis_idx = MessageTools::axis_index_from_id(axis_id);
    if (axis_states[axis_idx].online) {
        x_foot = axis_states[axis_idx].force_and_position.x_foot;
        return true;
    }
    return false;
}

bool CANManager::get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) {
    if (!MessageTools::check_axis_id(axis_id)) return false;
    uint8_t axis_idx = MessageTools::axis_index_from_id(axis_id);
    if (axis_states[axis_idx].online) {
        x_foot_min = axis_states[axis_idx].position_limits.x_foot_min;
        x_foot_max = axis_states[axis_idx].position_limits.x_foot_max;
        return true;
    }
    return false;
}

bool CANManager::get_function_id(AxisID axis_id, FunctionID &function_id) {
    if (!MessageTools::check_axis_id(axis_id)) return false;
    uint8_t axis_idx = MessageTools::axis_index_from_id(axis_id);
    if (axis_states[axis_idx].online) {
        function_id = axis_states[axis_idx].function_id;
        return true;
    }
    return false;
}

bool CANManager::is_online(AxisID axis_id) {
    if (!MessageTools::check_axis_id(axis_id)) return false;
    return axis_states[MessageTools::axis_index_from_id(axis_id)].online;
}

bool CANManager::check_bus(uint32_t ti_now) {
    auto can_state = ESP32Can.canState();
    switch (bus_state) {
        case BusState::ONLINE:
            if (can_state == TWAI_STATE_BUS_OFF) {
                switch_bus_state(ti_now, BusState::BUS_OFF);
                LogOutput::printf("CANManager: BUS_OFF");
            }
            break;
        case BusState::BUS_OFF:
            if ((ti_now - ti_state_change) > 1000000) {
                switch_bus_state(ti_now, BusState::PRE_ONLINE);
                ESP32Can.recover();
            }
            break;
        case BusState::PRE_ONLINE:
            if (can_state == TWAI_STATE_BUS_OFF) {
                switch_bus_state(ti_now, BusState::BUS_OFF);
            } else if ((ti_now - ti_state_change) > 100000) {
                if (can_state != TWAI_STATE_RUNNING) {
                    ti_state_change = ti_now;
                    ESP32Can.restart();
                } else {
                    switch_bus_state(ti_now, BusState::ONLINE);
                    LogOutput::printf("CANManager: Online");
                }
            }
            break;
        default:
            switch_bus_state(ti_now, BusState::PRE_ONLINE);
            LogOutput::printf("CANManager: Error: Unknown BusState (0x%02X)!", bus_state);
            break;
    }
    return bus_state == BusState::ONLINE;
}

bool CANManager::try_process_high_prio_axis_frame(CanFrame &rx_frame, uint32_t now) {
    if ((rx_frame.identifier & 0xF00) == 0x100) {
        uint8_t axis_idx = rx_frame.identifier & 0x00F;
        uint8_t frame_type = (rx_frame.identifier >> 4) & 0x00F;
        axis_states[axis_idx].ti_last_seen = now;
        axis_states[axis_idx].online = true;
        switch (frame_type) {
            case AxisFrameTypesHS::FORCE_AND_POSITION:
                if (axis_idx < MessageTools::MAX_AXES_COUNT) {
                    memcpy(&(axis_states[axis_idx].force_and_position), rx_frame.data, sizeof(ForceAndPosition));
                }
                break;
            default:
                break;
        }
        return true;
    }
    return false;
}

bool CANManager::try_process_low_prio_axis_frame(CanFrame &rx_frame, uint32_t now) {
    if ((rx_frame.identifier & 0xF00) == 0x300) {
        uint8_t axis_idx = rx_frame.identifier & 0x00F;
        uint8_t frame_type = (rx_frame.identifier >> 4) & 0x00F;
        axis_states[axis_idx].ti_last_status_update = now;
        axis_states[axis_idx].status_valid = true;
        switch (frame_type) {
            case AxisFrameTypesLS::POSITION_LIMITS:
                if (axis_idx < MessageTools::MAX_AXES_COUNT) {
                    memcpy(&(axis_states[axis_idx].position_limits), rx_frame.data, sizeof(PositionLimits));
                }
                break;
            case AxisFrameTypesLS::FUNCTION_ID:
                if (axis_idx < MessageTools::MAX_AXES_COUNT) {
                    memcpy(&(axis_states[axis_idx].function_id), rx_frame.data, sizeof(FunctionID));
                }
                break;
            default:
                break;
        }
        return true;
    }
    return false;
}

void CANManager::process(void) {
    CanFrame rx_frame;
    uint8_t num_max_frames = 40;
    uint32_t now = micros();
    update_timeouts(now);
    broadcast_state_updates(now);
    if (check_bus(now) == false) {
        return;
    }
    while (num_max_frames--) {
        if (ESP32Can.readFrame(&rx_frame, 0)) {
            if (try_process_high_prio_axis_frame(rx_frame, now)) continue;
            if (!_is_gateway) {
                if (try_process_ffb_update_frame(rx_frame)) continue;
            }
            if (try_process_low_prio_axis_frame(rx_frame, now)) continue;
            if (try_process_gateway_isotp_can_frame(rx_frame)) continue;
            if (try_process_ping_frame(rx_frame, now)) continue;
            if (_is_gateway) {
                if (try_process_axis_isotp_can_frame(rx_frame)) continue;
            }
        } else {
            return;
        }
    }
    if (rx_err_cnt < 0xFFFFFFFF) {
        rx_err_cnt++;
    }
}

void CANManager::process_isotp(void) {
    if (!_is_gateway) {
        isotp_poll(&(gateway_isotp_state.link));
        if (isotp_receive(&(gateway_isotp_state.link), isotp_rx_buff, ISOTP_BUFFER_SIZE, &isotp_rx_size) == ISOTP_RET_OK) {
            if (on_gateway_payload) {
                on_gateway_payload(isotp_rx_buff, isotp_rx_size);
            }
        }
    } else {
        send_ping_frame(micros());
        for (int axis_idx = 0; axis_idx < MessageTools::MAX_AXES_COUNT; axis_idx++) {
            isotp_poll(&(isotp_state[axis_idx].link));
            if (isotp_receive(&(isotp_state[axis_idx].link), isotp_rx_buff, ISOTP_BUFFER_SIZE, &isotp_rx_size) == ISOTP_RET_OK) {
                if (on_axis_payload) {
                    on_axis_payload(MessageTools::axis_id_from_index(axis_idx), isotp_rx_buff, isotp_rx_size);
                }
            }
        }
    }
}

void CANManager::update_timeouts(uint32_t now) {
    for (int axis_idx = 0; axis_idx < MessageTools::MAX_AXES_COUNT; axis_idx++) {
        if (axis_states[axis_idx].online && ((now - axis_states[axis_idx].ti_last_seen) > 5000)) {
            axis_states[axis_idx].online = false;
        }
        if (axis_states[axis_idx].status_valid && ((now - axis_states[axis_idx].ti_last_status_update) > 5000000)) {
            axis_states[axis_idx].status_valid = false;
        }
    }
    if (!_is_gateway) {
        if (_gateway_online && ((now - ti_last_ping) > 1100000)) {
            LogOutput::printf("CANManager: Gateway offline");
            _gateway_online = false;
        }
    }
}

void CANManager::shared_setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin) {
    ESP32Can.begin(ESP32Can.convertSpeed(baud_rate), tx_pin, rx_pin, 40, 40);
    isotp_init_link(&(gateway_isotp_state.link), 0x710 + own_axis_index, gateway_isotp_state.isotp_link_tx_buff, ISOTP_BUFFER_SIZE,
                    gateway_isotp_state.isotp_link_rx_buff, ISOTP_BUFFER_SIZE);
    for (int i = 0; i < MessageTools::MAX_AXES_COUNT; i++) {
        isotp_init_link(&(isotp_state[i].link), 0x700 + i, isotp_state[i].isotp_link_tx_buff, ISOTP_BUFFER_SIZE, isotp_state[i].isotp_link_rx_buff,
                        ISOTP_BUFFER_SIZE);
    }
    switch_bus_state(BusState::ONLINE);
}

/*****************************************************************************************************************/
/* AxisCANManager */
/*****************************************************************************************************************/
bool CANManager::try_process_ping_frame(CanFrame &rx_frame, uint32_t now) {
    if (rx_frame.identifier == 0x7FE) {
        ti_last_ping = now;
        if (!_gateway_online) {
            LogOutput::printf("CANManager: Gateway online");
            _gateway_online = true;
            if (own_axis_index > 0) {
                _is_gateway = false;
                LogOutput::printf(" -> giving up Gateway role");
            }
        }
        return true;
    }
    return false;
}

void CANManager::broadcast_state_updates(void) {
    if (own_axis_index < 0) return;
    if (_x_foot_min < _x_foot_max) {
        update_position_limits(_x_foot_min, _x_foot_max);
    }
    update_function_id(_function_id);
}

void CANManager::broadcast_state_updates(uint32_t now) {
    if ((now - axis_states[own_axis_index].ti_last_status_update) > 2000000) {
        broadcast_state_updates();
    }
}

bool CANManager::setup(AxisID axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnGatewayPayload on_gateway_payload,
                       OnFFBAction on_ffb_action, OnAxisPayload on_axis_payload) {
    LogOutput::printf("CANManager: Performing setup...");
    own_axis_id = axis_id;
    own_axis_index = MessageTools::axis_index_from_id(axis_id);
    this->on_gateway_payload = on_gateway_payload;
    this->on_ffb_action = on_ffb_action;
    this->on_axis_payload = on_axis_payload;
    shared_setup(baud_rate, tx_pin, rx_pin);
    broadcast_state_updates();
    LogOutput::printf(" -> done");
    return true;
}

bool CANManager::send_force_and_position(float &f_foot, float &x_foot) {
    if (own_axis_index < 0) return false;
    axis_states[own_axis_index].force_and_position.f_foot = f_foot;
    axis_states[own_axis_index].force_and_position.x_foot = x_foot;
    axis_states[own_axis_index].online = true;
    fast_update_cnt++;
    /* send the CAN frame on every second call only to keep bus load reasonable even with eight axes */
    if (fast_update_cnt == 2) {
        fast_update_cnt = 0;
        CanFrame tx_frame = {};
        tx_frame.identifier = 0x100 + (AxisFrameTypesHS::FORCE_AND_POSITION << 4) + own_axis_index;
        memcpy(tx_frame.data, &(axis_states[own_axis_index].force_and_position), sizeof(ForceAndPosition));
        tx_frame.data_length_code = sizeof(ForceAndPosition);
        if (!ESP32Can.writeFrame(&tx_frame, 0)) {
            if (tx_err_cnt < 0xFFFFFFFF) {
                tx_err_cnt++;
            }
            return false;
        }
    }
    return true;
}

bool CANManager::update_position_limits(float x_foot_min, float x_foot_max) {
    _x_foot_min = x_foot_min;
    _x_foot_max = x_foot_max;
    if (own_axis_index < 0) return false;
    axis_states[own_axis_index].position_limits.x_foot_min = x_foot_min;
    axis_states[own_axis_index].position_limits.x_foot_max = x_foot_max;
    axis_states[own_axis_index].ti_last_status_update = micros();
    axis_states[own_axis_index].status_valid = true;
    CanFrame tx_frame = {};
    tx_frame.identifier = 0x300 + (AxisFrameTypesLS::POSITION_LIMITS << 4) + own_axis_index;
    memcpy(tx_frame.data, &(axis_states[own_axis_index].position_limits), sizeof(PositionLimits));
    tx_frame.data_length_code = sizeof(PositionLimits);
    if (!ESP32Can.writeFrame(&tx_frame, 0)) {
        if (tx_err_cnt < 0xFFFFFFFF) {
            tx_err_cnt++;
        }
        return false;
    }
    return true;
}

bool CANManager::update_function_id(FunctionID function_id) {
    _function_id = function_id;
    if (own_axis_index < 0) return false;
    axis_states[own_axis_index].function_id = _function_id;
    axis_states[own_axis_index].ti_last_status_update = micros();
    axis_states[own_axis_index].status_valid = true;
    CanFrame tx_frame = {};
    tx_frame.identifier = 0x300 + (AxisFrameTypesLS::FUNCTION_ID << 4) + own_axis_index;
    memcpy(tx_frame.data, &(axis_states[own_axis_index].function_id), sizeof(FunctionID));
    tx_frame.data_length_code = sizeof(FunctionID);
    if (!ESP32Can.writeFrame(&tx_frame, 0)) {
        if (tx_err_cnt < 0xFFFFFFFF) {
            tx_err_cnt++;
        }
        return false;
    }
    return true;
}

bool CANManager::send_payload_to_gateway(const uint8_t *data, uint32_t len) {
    return isotp_send(&(gateway_isotp_state.link), data, len) == ISOTP_RET_OK;
}

bool CANManager::send_message_to_gateway(const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) {
    return send_payload_to_gateway(raw_data, len_raw_data);
}

bool CANManager::try_process_gateway_isotp_can_frame(CanFrame &rx_frame) {
    if (own_axis_index < 0) return false;
    if (rx_frame.identifier == (0x700 + own_axis_index)) {
        IsoTpLink *link = &(gateway_isotp_state.link);
        isotp_on_can_message(link, rx_frame.data, rx_frame.data_length_code);
        return true;
    }
    return false;
}

bool CANManager::try_process_ffb_update_frame(CanFrame &rx_frame) {
    if (own_axis_index < 0) return false;
    if ((rx_frame.identifier & 0xF00) == 0x200) {
        uint8_t function_id = rx_frame.identifier & 0x00F;
        uint8_t frame_type = (rx_frame.identifier >> 4) & 0x00F;
        FFBAction action = FFBAction_init_default;
        switch (frame_type) {
            case FFBFrameTypes::ABS:
                action.function_id = FunctionID(function_id);
                action.which_function = FFBAction_automotive_pedal_tag;
                action.function.automotive_pedal.trigger_abs = true;
                on_ffb_action(action);
                break;
            default:
                break;
        }
        return true;
    }
    return false;
}

/*****************************************************************************************************************/
/* GatewayCANManager */
/*****************************************************************************************************************/
void CANManager::send_ping_frame(uint32_t now) {
    if ((now - ti_last_ping) > 1000000) {
        ti_last_ping = now;
        CanFrame tx_frame = {};
        tx_frame.identifier = 0x7FE;
        tx_frame.data_length_code = 0;
        if (!ESP32Can.writeFrame(&tx_frame, 0)) {
            if (tx_err_cnt < 0xFFFFFFFF) {
                tx_err_cnt++;
            }
        }
    }
}

bool CANManager::try_process_axis_isotp_can_frame(CanFrame &rx_frame) {
    if ((rx_frame.identifier & 0xFF0) == 0x710) {
        uint8_t axis_idx = rx_frame.identifier & 0x00F;
        if (axis_idx < MessageTools::MAX_AXES_COUNT) {
            IsoTpLink *link = &(isotp_state[axis_idx].link);
            isotp_on_can_message(link, rx_frame.data, rx_frame.data_length_code);
        }
        return true;
    }
    return false;
}

bool CANManager::setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnAxisPayload cb) {
    LogOutput::printf("CANManager: Performing setup (gateway only mode)...");
    _is_gateway = true;
    on_axis_payload = cb;
    shared_setup(baud_rate, tx_pin, rx_pin);
    LogOutput::printf(" -> done");
    return true;
}

bool CANManager::send_payload_to_axis(AxisID axis_id, const uint8_t *data, uint32_t len) {
    if (!MessageTools::check_axis_id(axis_id)) return false;
    return isotp_send(&(isotp_state[MessageTools::axis_index_from_id(axis_id)].link), data, len) == ISOTP_RET_OK;
}

bool CANManager::send_abs_trigger(const FFBAction &action) {
    CanFrame tx_frame = {};
    tx_frame.identifier = 0x200 + (FFBFrameTypes::ABS << 4) + action.function_id;
    tx_frame.data_length_code = 0;
    if (!ESP32Can.writeFrame(&tx_frame, 0)) {
        if (tx_err_cnt < 0xFFFFFFFF) {
            tx_err_cnt++;
        }
        return false;
    }
    return true;
}

bool CANManager::send_message_to_axis(AxisID axis_id, const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) {
    switch (message.which_payload) {
        case Message_ffb_action_tag:
            if ((message.payload.ffb_action.which_function == FFBAction_automotive_pedal_tag) && message.payload.ffb_action.function.automotive_pedal.trigger_abs) {
                return send_abs_trigger(message.payload.ffb_action);
            } else {
                return send_payload_to_axis(axis_id, raw_data, len_raw_data);
            }
            break;
        default:
            return send_payload_to_axis(axis_id, raw_data, len_raw_data);
            break;
    }
    return false;
}
