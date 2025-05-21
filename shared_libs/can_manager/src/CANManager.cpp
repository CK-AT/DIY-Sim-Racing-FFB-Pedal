#include <CANManager.h>

/*****************************************************************************************************************/
/* isotp-c shim functions                                                                                        */
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

extern "C" void isotp_user_debug(const char* message, ...) {
    Serial.printf(message);
    Serial.println();
}

/*****************************************************************************************************************/
/* CANManager                                                                                                    */
/*****************************************************************************************************************/
bool CANManager::get_force(int8_t axis_id, float &f_foot) {
    if (axis_id < MAX_AXES) {
        if (axis_states[axis_id].online) {
            f_foot = axis_states[axis_id].force_and_position.f_foot;
            return true;
        }
    }
    return false;
}

bool CANManager::get_position(int8_t axis_id, float & x_foot) {
    if (axis_id < MAX_AXES) {
        if (axis_states[axis_id].online) {
            x_foot = axis_states[axis_id].force_and_position.x_foot;
            return true;
        }
    }
    return false;
}

bool CANManager::is_online(int8_t axis_id) {
    if (axis_id < MAX_AXES) {
        return axis_states[axis_id].online;
    }
    return false;
}

/*****************************************************************************************************************/
/* AxisCANManager                                                                                                */
/*****************************************************************************************************************/
void AxisCANManager::process(void) {
    CanFrame rx_frame;
    uint8_t num_max_frames = 40;
    uint32_t now = micros();
    for (int i = 0; i < MAX_AXES; i++) {
        if (axis_states[i].online && ((now - axis_states[i].ti_last_seen) > 5000)) {
            axis_states[i].online = false;
        }
    }
    isotp_poll(&(isotp_state.link));
    while (num_max_frames--) {
        if (ESP32Can.readFrame(&rx_frame, 0)) {
            if ((rx_frame.identifier & 0xF00) == 0x100) {
                /* high prio axis-specific frames */
                uint8_t axis_id = rx_frame.identifier & 0x00F;
                uint8_t frame_type = (rx_frame.identifier >> 4) & 0x00F;
                axis_states[axis_id].ti_last_seen = now;
                axis_states[axis_id].online = true;
                switch (frame_type) {
                    case FORCE_AND_POSITION:
                        if (axis_id < MAX_AXES) {
                            memcpy(&(axis_states[axis_id].force_and_position), rx_frame.data, sizeof(ForceAndPosition));
                        }
                        break;
                    default:
                        break;
                }
            } else if (rx_frame.identifier == (0x700 + own_axis_id)) {
                IsoTpLink *link = &(isotp_state.link);
                isotp_on_can_message(link, rx_frame.data, rx_frame.data_length_code);
                isotp_poll(link);
                if (isotp_receive(link, isotp_rx_buff, ISOTP_BUFFER_SIZE, &isotp_rx_size) == ISOTP_RET_OK) {
                    if (on_gateway_payload) {
                        on_gateway_payload(isotp_rx_buff, isotp_rx_size);
                    }
                }
            }
        } else {
            return;
        }
    }
    if (rx_err_cnt < 0xFFFFFFFF) {
        rx_err_cnt++;
    }
}

void AxisCANManager::setup(int8_t axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnGatewayPayload cb) {
    if (axis_id < MAX_AXES) {
        own_axis_id = axis_id;
        on_gateway_payload = cb;
        ESP32Can.begin(ESP32Can.convertSpeed(baud_rate), tx_pin, rx_pin, 40, 40);
        isotp_init_link(&(isotp_state.link), 0x710 + own_axis_id, isotp_state.isotp_link_tx_buff, ISOTP_BUFFER_SIZE, isotp_state.isotp_link_rx_buff, ISOTP_BUFFER_SIZE);
    }
}

void AxisCANManager::send_force_and_position(float &f_foot, float & x_foot) {
    if (own_axis_id >= 0) {
        axis_states[own_axis_id].force_and_position.f_foot = f_foot;
        axis_states[own_axis_id].force_and_position.x_foot = x_foot;
        CanFrame tx_frame = {};
        tx_frame.identifier = 0x100 + (FORCE_AND_POSITION << 4) + own_axis_id;
        memcpy(tx_frame.data, &(axis_states[own_axis_id].force_and_position), sizeof(ForceAndPosition));
        tx_frame.data_length_code = sizeof(ForceAndPosition);
        if (!ESP32Can.writeFrame(&tx_frame, 0)) {
            if (tx_err_cnt < 0xFFFFFFFF) {
                tx_err_cnt++;
            }
        }
    }
}

bool AxisCANManager::send_payload_to_gateway(uint8_t *data, uint32_t len) {
    return isotp_send(&(isotp_state.link), data, len) == ISOTP_RET_OK;
}

/*****************************************************************************************************************/
/* GatewayCANManager                                                                                             */
/*****************************************************************************************************************/
void GatewayCANManager::process(void) {
    CanFrame rx_frame;
    uint8_t num_max_frames = 40;
    uint32_t now = micros();
    for (int i = 0; i < MAX_AXES; i++) {
        if (axis_states[i].online && ((now - axis_states[i].ti_last_seen) > 5000)) {
            axis_states[i].online = false;
        }
        isotp_poll(&(isotp_state[i].link));
    }
    while (num_max_frames--) {
        if (ESP32Can.readFrame(&rx_frame, 0)) {
            if ((rx_frame.identifier & 0xF00) == 0x100) {
                /* high prio axis-specific frames */
                uint8_t axis_id = rx_frame.identifier & 0x00F;
                uint8_t frame_type = (rx_frame.identifier >> 4) & 0x00F;
                axis_states[axis_id].ti_last_seen = now;
                axis_states[axis_id].online = true;
                switch (frame_type) {
                    case FORCE_AND_POSITION:
                        if (axis_id < MAX_AXES) {
                            memcpy(&(axis_states[axis_id].force_and_position), rx_frame.data, sizeof(ForceAndPosition));
                        }
                        break;
                    default:
                        break;
                }
            } else if ((rx_frame.identifier & 0xFF0) == 0x710) {
                uint8_t axis_id = rx_frame.identifier & 0x00F;
                if (axis_id < MAX_AXES) {
                    IsoTpLink *link = &(isotp_state[axis_id].link);
                    isotp_on_can_message(link, rx_frame.data, rx_frame.data_length_code);
                    isotp_poll(link);
                    if (isotp_receive(link, isotp_rx_buff, ISOTP_BUFFER_SIZE, &isotp_rx_size) == ISOTP_RET_OK) {
                        if (on_axis_payload) {
                            on_axis_payload(axis_id, isotp_rx_buff, isotp_rx_size);
                        }
                    }
                }
            }
        } else {
            return;
        }
    }
    if (rx_err_cnt < 0xFFFFFFFF) {
        rx_err_cnt++;
    }
}

void GatewayCANManager::setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnAxisPayload cb) {
    on_axis_payload = cb;
    ESP32Can.begin(ESP32Can.convertSpeed(baud_rate), tx_pin, rx_pin, 40, 40);
    for (int i = 0; i < MAX_AXES; i++) {
        isotp_init_link(&(isotp_state[i].link), 0x700 + i, isotp_state[i].isotp_link_tx_buff, ISOTP_BUFFER_SIZE, isotp_state[i].isotp_link_rx_buff, ISOTP_BUFFER_SIZE);
    }
    xTaskCreatePinnedToCore(this->task_func, "CANManagerTask", 5000, this, 1, NULL, 0);
}

bool GatewayCANManager::send_payload_to_axis(uint8_t axis_id, uint8_t *data, uint32_t len) {
    return isotp_send(&(isotp_state[axis_id].link), data, len) == ISOTP_RET_OK;
}

