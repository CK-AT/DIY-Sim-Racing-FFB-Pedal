#pragma once
#include "Arduino.h"
#include "ffb_data_types.pb.h"

class FFBDataTools {
    public:
        static bool check_and_decode_ffb_data(FFBData &ffb_data, const uint8_t *buffer, uint16_t len, uint16_t crc_expected);
        static uint16_t encode_ffb_data_and_calc_crc(const FFBData &ffb_data, uint8_t *buffer, uint16_t len, uint16_t &crc);
        static const uint16_t MAX_ENCODED_SIZE = FFB_DATA_TYPES_PB_H_MAX_SIZE;
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
            if (FFBDataTools::axis_index_from_id(axis_id) < MAX_AXES_COUNT) return true;
            return false;
        }
};
