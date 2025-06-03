
#include "MessageTools.h"

#include "FastCRC.h"
#include "LogOutput.h"
#include "pb_decode.h"
#include "pb_encode.h"

FastCRC16 CRC16;

bool MessageTools::check_and_decode_message(Message &message, const uint8_t *buffer, uint16_t len, uint16_t crc_expected) {
    uint16_t crc = CRC16.modbus(buffer, len);
    if (crc == crc_expected) {
        pb_istream_t istream = pb_istream_from_buffer(buffer, len);
        if (pb_decode(&istream, &Message_msg, &message)) {
            return true;
        } else {
            LogOutput::printf(" -> Decoding error: %s", istream.errmsg);
        }
    } else {
        LogOutput::printf(" -> CRC error");
    }
    return false;
}

uint16_t MessageTools::encode_message_and_calc_crc(const Message &message, uint8_t *buffer, uint16_t len, uint16_t &crc) {
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, len);
    if (pb_encode(&ostream, &Message_msg, &message)) {
        crc = CRC16.modbus(buffer, ostream.bytes_written);
        return ostream.bytes_written;
    }
    return 0;
}

uint16_t MessageTools::calc_crc(const uint8_t *buffer, uint16_t len) {
    return CRC16.modbus(buffer, len);
}