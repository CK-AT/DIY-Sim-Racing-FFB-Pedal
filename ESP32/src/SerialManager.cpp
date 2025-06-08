#include <SerialManager.h>
#include <LogOutput.h>
#include <CommManager.h>

/*****************************************************************************************************************/
/* SerialManager */
/*****************************************************************************************************************/

static ICommChannel::OnGatewayPayload _on_host_payload;

static void _on_gateway_payload_wrapper(const uint8_t *data, size_t len) {
    _on_host_payload(data, len);
}

void SerialManager::process(void) {
    packet_serial.update();
}

bool SerialManager::setup(Stream *serial, CommManager *comm_manager, ICommChannel::OnGatewayPayload on_host_payload) {
    this->comm_manager = comm_manager;
    _on_host_payload = on_host_payload;
    state_message.which_payload = Message_axis_state_tag;
    packet_serial.setStream(serial);
    packet_serial.setPacketHandler(_on_gateway_payload_wrapper);
    // send some zero bytes to ensure proper COBS sync on the first message
    serial->print("\x00\x00\x00");
    xTaskCreatePinnedToCore(this->task_func, "SerialManagerTask", 5000, this, 1, NULL, 0);
    return true;
}

void SerialManager::update_force_and_position(float &f_contact_point, float &x_contact_point) {
    _f_contact_point = f_contact_point;
    _x_contact_point = x_contact_point;
}

bool SerialManager::send_message_to_host(const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) {
    packet_serial.send(raw_data, len_raw_data);
    return true;
}

// void CANManager::send_ping_frame(uint32_t now) {
//     _gateway_online = true;
//     if ((now - ti_last_ping) > 1000000) {
//         CanFrame tx_frame = {};
//         tx_frame.identifier = 0x7FE;
//         tx_frame.data_length_code = 0;
//         if (!ESP32Can.writeFrame(&tx_frame, 0)) {
//             if (tx_err_cnt < 0xFFFFFFFF) {
//                 tx_err_cnt++;
//             }
//         }
//     }
// }

// bool CANManager::try_process_axis_isotp_can_frame(CanFrame &rx_frame) {
//     if ((rx_frame.identifier & 0xFF0) == 0x710) {
//         uint8_t axis_idx = rx_frame.identifier & 0x00F;
//         if (axis_idx < MessageTools::MAX_AXES_COUNT) {
//             IsoTpLink *link = &(isotp_state[axis_idx].link);
//             isotp_on_can_message(link, rx_frame.data, rx_frame.data_length_code);
//             isotp_poll(link);
//             if (isotp_receive(link, isotp_rx_buff, ISOTP_BUFFER_SIZE, &isotp_rx_size) == ISOTP_RET_OK) {
//                 if (on_axis_payload) {
//                     on_axis_payload(MessageTools::axis_id_from_index(axis_idx), isotp_rx_buff, isotp_rx_size);
//                 }
//             }
//         }
//         return true;
//     }
//     return false;
// }

// bool CANManager::setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnAxisPayload cb) {
//     LogOutput::printf("CANManager: Performing setup (gateway only mode)...");
//     _is_gateway = true;
//     on_axis_payload = cb;
//     shared_setup(baud_rate, tx_pin, rx_pin);
//     xTaskCreatePinnedToCore(this->task_func, "CANManagerTask", 5000, this, 1, NULL, 0);
//     LogOutput::printf(" -> done");
//     return true;
// }

// bool CANManager::send_payload_to_axis(AxisID axis_id, const uint8_t *data, uint32_t len) {
//     if (!MessageTools::check_axis_id(axis_id)) return false;
//     return isotp_send(&(isotp_state[MessageTools::axis_index_from_id(axis_id)].link), data, len) == ISOTP_RET_OK;
// }

// bool CANManager::send_abs_trigger_to_axis(AxisID axis_id) {
//     if (!MessageTools::check_axis_id(axis_id)) return false;
//     CanFrame tx_frame = {};
//     tx_frame.identifier = 0x200 + (FFBFrameTypes::ABS << 4) + MessageTools::axis_index_from_id(axis_id);
//     tx_frame.data_length_code = 0;
//     if (!ESP32Can.writeFrame(&tx_frame, 0)) {
//         if (tx_err_cnt < 0xFFFFFFFF) {
//             tx_err_cnt++;
//         }
//         return false;
//     }
//     return true;
// }

// bool CANManager::send_message_to_axis(AxisID axis_id, const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) {
//     switch (message.which_payload) {
//         case Message_ffb_action_tag:
//             if (message.payload.ffb_action.trigger_abs) {
//                 return send_abs_trigger_to_axis(axis_id);
//             }
//             break;
//         default:
//             return send_payload_to_axis(axis_id, raw_data, len_raw_data);
//             break;
//     }
//     return false;
// }
