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
    if (!FFBDataTools::check_axis_id(axis_id)) return false;
    uint8_t axis_idx = FFBDataTools::axis_index_from_id(axis_id);
    if (axis_states[axis_idx].online) {
        f_foot = axis_states[axis_idx].force_and_position.f_foot;
        return true;
    }
    return false;
}

bool CANManager::get_position(AxisID axis_id, float &x_foot) {
    if (!FFBDataTools::check_axis_id(axis_id)) return false;
    uint8_t axis_idx = FFBDataTools::axis_index_from_id(axis_id);
    if (axis_states[axis_idx].online) {
        x_foot = axis_states[axis_idx].force_and_position.x_foot;
        return true;
    }
    return false;
}

bool CANManager::get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) {
    if (!FFBDataTools::check_axis_id(axis_id)) return false;
    uint8_t axis_idx = FFBDataTools::axis_index_from_id(axis_id);
    if (axis_states[axis_idx].online) {
        x_foot_min = axis_states[axis_idx].position_limits.x_foot_min;
        x_foot_max = axis_states[axis_idx].position_limits.x_foot_max;
        return true;
    }
    return false;
}

bool CANManager::is_online(AxisID axis_id) {
    if (!FFBDataTools::check_axis_id(axis_id)) return false;
    return axis_states[FFBDataTools::axis_index_from_id(axis_id)].online;
}

bool CANManager::check_bus(uint32_t ti_now) {
    auto can_state = ESP32Can.canState();
    switch (bus_state) {
        case BusState::ONLINE:
            if (can_state == TWAI_STATE_BUS_OFF) {
                switch_bus_state(ti_now, BusState::BUS_OFF);
                LogOutput::printf("CANManager: BUS_OFF\n");
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
                    LogOutput::printf("CANManager: Online\n");
                }
            }
            break;
        default:
            switch_bus_state(ti_now, BusState::PRE_ONLINE);
            LogOutput::printf("CANManager: Error: Unknown BusState (0x%02X)!\n", bus_state);
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
                if (axis_idx < FFBDataTools::MAX_AXES_COUNT) {
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
        axis_states[axis_idx].ti_last_limit_update = now;
        axis_states[axis_idx].limits_valid = true;
        switch (frame_type) {
            case AxisFrameTypesLS::POSITION_LIMITS:
                if (axis_idx < FFBDataTools::MAX_AXES_COUNT) {
                    memcpy(&(axis_states[axis_idx].position_limits), rx_frame.data, sizeof(PositionLimits));
                }
                break;
            default:
                break;
        }
        return true;
    }
    return false;
}

void CANManager::update_axis_timeouts(uint32_t now) {
    for (int axis_idx = 0; axis_idx < FFBDataTools::MAX_AXES_COUNT; axis_idx++) {
        if (axis_states[axis_idx].online && ((now - axis_states[axis_idx].ti_last_seen) > 5000)) {
            axis_states[axis_idx].online = false;
        }
        if (axis_states[axis_idx].limits_valid && ((now - axis_states[axis_idx].ti_last_limit_update) > 5000000)) {
            axis_states[axis_idx].limits_valid = false;
        }
    }
}

/*****************************************************************************************************************/
/* AxisCANManager */
/*****************************************************************************************************************/
void AxisCANManager::process(void) {
    CanFrame rx_frame;
    uint8_t num_max_frames = 40;
    uint32_t now = micros();
    update_axis_timeouts(now);
    broadcast_position_limits(now);
    if (check_bus(now) == false) {
        return;
    }
    isotp_poll(&(isotp_state.link));
    while (num_max_frames--) {
        if (ESP32Can.readFrame(&rx_frame, 0)) {
            if (try_process_high_prio_axis_frame(rx_frame, now)) continue;
            if (try_process_ffb_update_frame(rx_frame)) continue;
            if (try_process_low_prio_axis_frame(rx_frame, now)) continue;
            if (try_process_isotp_can_frame(rx_frame)) continue;
        } else {
            return;
        }
    }
    if (rx_err_cnt < 0xFFFFFFFF) {
        rx_err_cnt++;
    }
}

void AxisCANManager::broadcast_position_limits(void) {
    if (own_axis_index < 0) return;
    if (_x_foot_min < _x_foot_max) {
        send_position_limits(_x_foot_min, _x_foot_max);
    }
}

void AxisCANManager::broadcast_position_limits(uint32_t now) {
    if ((now - axis_states[own_axis_index].ti_last_limit_update) > 2000000) {
        broadcast_position_limits();
    }
}

void AxisCANManager::setup(AxisID axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnGatewayPayload on_gateway_payload,
                           OnFFBAction on_ffb_action) {
    if (FFBDataTools::check_axis_id(axis_id)) {
        own_axis_id = axis_id;
        own_axis_index = FFBDataTools::axis_index_from_id(axis_id);
        this->on_gateway_payload = on_gateway_payload;
        this->on_ffb_action = on_ffb_action;
        ESP32Can.begin(ESP32Can.convertSpeed(baud_rate), tx_pin, rx_pin, 40, 40);
        isotp_init_link(&(isotp_state.link), 0x710 + own_axis_index, isotp_state.isotp_link_tx_buff, ISOTP_BUFFER_SIZE,
                        isotp_state.isotp_link_rx_buff, ISOTP_BUFFER_SIZE);
        switch_bus_state(BusState::ONLINE);
        broadcast_position_limits();
        LogOutput::printf("CANManager: Init done\n");
    }
}

void AxisCANManager::send_force_and_position(float &f_foot, float &x_foot) {
    if (own_axis_index >= 0) {
        axis_states[own_axis_index].force_and_position.f_foot = f_foot;
        axis_states[own_axis_index].force_and_position.x_foot = x_foot;
        CanFrame tx_frame = {};
        tx_frame.identifier = 0x100 + (AxisFrameTypesHS::FORCE_AND_POSITION << 4) + own_axis_index;
        memcpy(tx_frame.data, &(axis_states[own_axis_index].force_and_position), sizeof(ForceAndPosition));
        tx_frame.data_length_code = sizeof(ForceAndPosition);
        if (!ESP32Can.writeFrame(&tx_frame, 0)) {
            if (tx_err_cnt < 0xFFFFFFFF) {
                tx_err_cnt++;
            }
        }
    }
}

void AxisCANManager::send_position_limits(float x_foot_min, float x_foot_max) {
    _x_foot_min = x_foot_min;
    _x_foot_max = x_foot_max;
    if (own_axis_index >= 0) {
        axis_states[own_axis_index].position_limits.x_foot_min = x_foot_min;
        axis_states[own_axis_index].position_limits.x_foot_max = x_foot_max;
        axis_states[own_axis_index].ti_last_limit_update = micros();
        axis_states[own_axis_index].limits_valid = true;
        CanFrame tx_frame = {};
        tx_frame.identifier = 0x300 + (AxisFrameTypesLS::POSITION_LIMITS << 4) + own_axis_index;
        memcpy(tx_frame.data, &(axis_states[own_axis_index].position_limits), sizeof(PositionLimits));
        tx_frame.data_length_code = sizeof(PositionLimits);
        if (!ESP32Can.writeFrame(&tx_frame, 0)) {
            if (tx_err_cnt < 0xFFFFFFFF) {
                tx_err_cnt++;
            }
        }
    }
}

bool AxisCANManager::send_payload_to_gateway(const uint8_t *data, uint32_t len) {
    return isotp_send(&(isotp_state.link), data, len) == ISOTP_RET_OK;
}

bool AxisCANManager::try_process_isotp_can_frame(CanFrame &rx_frame) {
    if (own_axis_index < 0) return false;
    if (rx_frame.identifier == (0x700 + own_axis_index)) {
        IsoTpLink *link = &(isotp_state.link);
        isotp_on_can_message(link, rx_frame.data, rx_frame.data_length_code);
        isotp_poll(link);
        if (isotp_receive(link, isotp_rx_buff, ISOTP_BUFFER_SIZE, &isotp_rx_size) == ISOTP_RET_OK) {
            if (on_gateway_payload) {
                on_gateway_payload(isotp_rx_buff, isotp_rx_size);
            }
        }
        return true;
    }
    return false;
}

bool AxisCANManager::try_process_ffb_update_frame(CanFrame &rx_frame) {
    if (own_axis_index < 0) return false;
    if ((rx_frame.identifier & 0xF00) == 0x200) {
        uint8_t axis_idx = rx_frame.identifier & 0x00F;
        if (axis_idx != own_axis_index) {
            return false;
        }
        uint8_t frame_type = (rx_frame.identifier >> 4) & 0x00F;
        FFBAction action = FFBAction_init_default;
        switch (frame_type) {
            case FFBFrameTypes::ABS:
                action.trigger_abs = true;
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
void GatewayCANManager::process(void) {
    CanFrame rx_frame;
    uint8_t num_max_frames = 40;
    uint32_t now = micros();
    update_axis_timeouts(now);
    if (check_bus(now) == false) {
        return;
    }
    for (int axis_idx = 0; axis_idx < FFBDataTools::MAX_AXES_COUNT; axis_idx++) {
        isotp_poll(&(isotp_state[axis_idx].link));
    }
    while (num_max_frames--) {
        if (ESP32Can.readFrame(&rx_frame, 0)) {
            if (try_process_high_prio_axis_frame(rx_frame, now)) continue;
            if (try_process_low_prio_axis_frame(rx_frame, now)) continue;
            if (try_process_isotp_can_frame(rx_frame)) continue;
        } else {
            return;
        }
    }
    if (rx_err_cnt < 0xFFFFFFFF) {
        rx_err_cnt++;
    }
}

bool GatewayCANManager::try_process_isotp_can_frame(CanFrame &rx_frame) {
    if ((rx_frame.identifier & 0xFF0) == 0x710) {
        uint8_t axis_idx = rx_frame.identifier & 0x00F;
        if (axis_idx < FFBDataTools::MAX_AXES_COUNT) {
            IsoTpLink *link = &(isotp_state[axis_idx].link);
            isotp_on_can_message(link, rx_frame.data, rx_frame.data_length_code);
            isotp_poll(link);
            if (isotp_receive(link, isotp_rx_buff, ISOTP_BUFFER_SIZE, &isotp_rx_size) == ISOTP_RET_OK) {
                if (on_axis_payload) {
                    on_axis_payload(FFBDataTools::axis_id_from_index(axis_idx), isotp_rx_buff, isotp_rx_size);
                }
            }
        }
        return true;
    }
    return false;
}

void GatewayCANManager::setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnAxisPayload cb) {
    on_axis_payload = cb;
    ESP32Can.begin(ESP32Can.convertSpeed(baud_rate), tx_pin, rx_pin, 40, 40);
    for (int i = 0; i < FFBDataTools::MAX_AXES_COUNT; i++) {
        isotp_init_link(&(isotp_state[i].link), 0x700 + i, isotp_state[i].isotp_link_tx_buff, ISOTP_BUFFER_SIZE, isotp_state[i].isotp_link_rx_buff,
                        ISOTP_BUFFER_SIZE);
    }
    switch_bus_state(BusState::ONLINE);
    xTaskCreatePinnedToCore(this->task_func, "CANManagerTask", 5000, this, 1, NULL, 0);
    LogOutput::printf("CANManager: Init done\n");
}

bool GatewayCANManager::send_payload_to_axis(AxisID axis_id, const uint8_t *data, uint32_t len) {
    if (!FFBDataTools::check_axis_id(axis_id)) return false;
    return isotp_send(&(isotp_state[FFBDataTools::axis_index_from_id(axis_id)].link), data, len) == ISOTP_RET_OK;
}

void GatewayCANManager::send_abs_trigger_to_axis(AxisID axis_id) {
    if (!FFBDataTools::check_axis_id(axis_id)) return;
    CanFrame tx_frame = {};
    tx_frame.identifier = 0x200 + (FFBFrameTypes::ABS << 4) + FFBDataTools::axis_index_from_id(axis_id);
    tx_frame.data_length_code = 0;
    if (!ESP32Can.writeFrame(&tx_frame, 0)) {
        if (tx_err_cnt < 0xFFFFFFFF) {
            tx_err_cnt++;
        }
    }
}

void GatewayCANManager::send_ffb_data_to_axis(AxisID axis_id, FFBData &ffb_data, const uint8_t *raw_data, uint32_t len_raw_data) {
    switch (ffb_data.which_payload) {
        case FFBData_ffb_action_tag:
            if (ffb_data.payload.ffb_action.trigger_abs) {
                send_abs_trigger_to_axis(axis_id);
            }
            break;
        default:
            send_payload_to_axis(axis_id, raw_data, len_raw_data);
            break;
    }
}
