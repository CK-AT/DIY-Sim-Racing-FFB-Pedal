#include "GripReader.h"
#include "Main.h"

// Use HSPI to avoid conflict with the ADC on the default SPI bus (FSPI).
// ESP32-S3: FSPI=0 (default, used by ADC), HSPI=1.
#define GRIP_SPI_BUS HSPI

static constexpr uint32_t GRIP_SPI_FREQ = 2000000;  // 2 MHz — well within 74HC165 specs at 3.3V

void GripReader::setup(uint8_t cs, uint8_t sck, uint8_t miso, uint8_t numBytes) {
    if (numBytes == 0 || numBytes > MAX_BYTES) return;

    _csPin = cs;
    _numBytes = numBytes;

    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);  // deassert (active low)

    _spi = new SPIClass(GRIP_SPI_BUS);
    // MOSI not used (shift registers are read-only), pass -1
    _spi->begin(sck, miso, -1, -1);

    _ready = true;
}

void GripReader::poll() {
    if (!_ready) return;

    // Warthog grip: CS directly controls CE (clock enable), active low.
    // PL (parallel load) is active low but directly controlled by a separate
    // pull mechanism on the connector.
    //
    // Sequence: CS LOW (enable clock) → clock out data → CS HIGH.
    _spi->beginTransaction(SPISettings(GRIP_SPI_FREQ, MSBFIRST, SPI_MODE0));

    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);

    bool allZeros = true;
    for (uint8_t i = 0; i < _numBytes; i++) {
        uint8_t raw = _spi->transfer(0x00);
        if (raw != 0x00) allZeros = false;
        _data[i] = ~raw;
    }

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    // All 0x00 raw = floating MISO (grip disconnected) → zero all buttons
    if (allZeros) {
        memset(_data, 0, sizeof(_data));
    }
}

bool GripReader::isPressed(uint8_t bitIndex) const {
    uint8_t byteIdx = bitIndex / 8;
    uint8_t bitIdx = 7 - (bitIndex % 8);  // MSB first
    if (byteIdx >= _numBytes) return false;
    return (_data[byteIdx] >> bitIdx) & 1;
}

uint8_t GripReader::getRawByte(uint8_t byteIndex) const {
    if (byteIndex >= _numBytes) return 0;
    return _data[byteIndex];
}

