#pragma once
#include "Arduino.h"
#include "diy_ffb_protocol.pb.h"

class MessageTools {
    public:
        static bool check_and_decode_message(Message &message, const uint8_t *buffer, uint16_t len, uint16_t crc_expected);
        static uint16_t encode_message_and_calc_crc(const Message &message, uint8_t *buffer, uint16_t len, uint16_t &crc);
        static const uint16_t MAX_ENCODED_SIZE = DIY_FFB_PROTOCOL_PB_H_MAX_SIZE;
        static const uint16_t MAX_AXES_COUNT = 8; // Can't use _AxisID_MAX because it includes AxisID_AXIS_SUBTRACTIVE, which is a flag encoded into bit 7
        static uint16_t calc_crc(const uint8_t *buffer, uint16_t len);
        static AxisID axis_id_from_index(uint8_t axis_index) {
            return AxisID(constrain(axis_index, 0, MAX_AXES_COUNT- 1) + 1);
        }
        static uint8_t axis_index_from_id(AxisID axis_id) {
            /* limit to zero */
            return uint8_t(constrain(axis_id - 1, 0, MAX_AXES_COUNT- 1));
        }
        static bool check_axis_id(AxisID axis_id) {
            if (axis_id == AxisID_AXIS_UNDEFINED) return false;
            if ((axis_id - 1) < MAX_AXES_COUNT) return true;
            return false;
        }
        static ControllerAxis controller_axis_id_from_index(uint8_t controller_axis_index) {
            return ControllerAxis(constrain(controller_axis_index, 0,  _ControllerAxis_MAX - 1) + 1);
        }
        static uint8_t controller_axis_index_from_id(ControllerAxis controller_axis) {
            /* limit to zero */
            return uint8_t(constrain(controller_axis - 1, 0, _ControllerAxis_MAX - 1));
        }
        static bool check_controller_axis_id(ControllerAxis controller_axis) {
            if (controller_axis == ControllerAxis_CONTROLLER_AXIS_UNDEFINED) return false;
            if ((controller_axis - 1) < _ControllerAxis_MAX) return true;
            return false;
        }
};
