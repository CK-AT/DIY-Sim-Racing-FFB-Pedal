
#include "FFBDataTools.h"

#include "FastCRC.h"
#include "LogOutput.h"
#include "pb_decode.h"
#include "pb_encode.h"

FastCRC16 CRC16;

bool FFBDataTools::check_and_decode_ffb_data(FFBData &ffb_data, const uint8_t *buffer, uint16_t len, uint16_t crc_expected) {
    uint16_t crc = CRC16.modbus(buffer, len);
    if (crc == crc_expected) {
        pb_istream_t istream = pb_istream_from_buffer(buffer, len);
        if (pb_decode(&istream, &FFBData_msg, &ffb_data)) {
            return true;
        } else {
            LogOutput::printf(" -> Decoding error: %s", istream.errmsg);
        }
    } else {
        LogOutput::printf(" -> CRC error");
    }
    return false;
}

uint16_t FFBDataTools::encode_ffb_data_and_calc_crc(const FFBData &ffb_data, uint8_t *buffer, uint16_t len, uint16_t &crc) {
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, len);
    if (pb_encode(&ostream, &FFBData_msg, &ffb_data)) {
        crc = CRC16.modbus(buffer, ostream.bytes_written);
        return ostream.bytes_written;
    }
    return 0;
}

uint16_t FFBDataTools::calc_crc(const uint8_t *buffer, uint16_t len) {
    return CRC16.modbus(buffer, len);
}