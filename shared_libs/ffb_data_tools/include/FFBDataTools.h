#pragma once
#include "ffb_data_types.pb.h"

class FFBDataTools {
    public:
        static bool check_and_decode_ffb_data(FFBData &ffb_data, const uint8_t *buffer, uint16_t len, uint16_t crc_expected);
        static uint16_t encode_ffb_data_and_calc_crc(const FFBData &ffb_data, uint8_t *buffer, uint16_t len, uint16_t &crc);
        static const uint16_t MAX_ENCODED_SIZE = FFB_DATA_TYPES_PB_H_MAX_SIZE;
        static uint16_t calc_crc(const uint8_t *buffer, uint16_t len);
};
