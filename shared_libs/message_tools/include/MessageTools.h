#pragma once
#include "Arduino.h"
#include "diy_ffb_protocol.pb.h"

class MessageTools {
    public:
        static bool check_and_decode_message(Message &message, const uint8_t *buffer, uint16_t len, uint16_t crc_expected);
        static uint16_t encode_message_and_calc_crc(const Message &message, uint8_t *buffer, uint16_t len, uint16_t &crc);
        static const uint16_t MAX_ENCODED_SIZE = DIY_FFB_PROTOCOL_PB_H_MAX_SIZE;
        static const uint16_t MAX_AXES_COUNT = _AxisID_MAX;
        static uint16_t calc_crc(const uint8_t *buffer, uint16_t len);
        static AxisID axis_id_from_index(uint8_t axis_index) {
            return AxisID(axis_index + 1);
        }
        static uint8_t axis_index_from_id(AxisID axis_id) {
            /* limit to zero */
            return uint8_t(max(axis_id - 1, 0));
        }
        static bool check_axis_id(AxisID axis_id) {
            if (axis_id == AxisID_AXIS_UNDEFINED) return false;
            if (MessageTools::axis_index_from_id(axis_id) < MAX_AXES_COUNT) return true;
            return false;
        }
};
