#pragma once

#include <stdint.h>
#include <SPI.h>

/// Reads button states from daisy-chained 74HC165 shift registers via SPI.
/// The Warthog grip uses 3 shift registers (24 bits).
/// Call setup() once, then poll() at ~100 Hz to update cached button states.
class GripReader {
public:
    static constexpr uint8_t MAX_BYTES = 4;   // max supported chain length
    static constexpr uint8_t MAX_BITS = MAX_BYTES * 8;
    static constexpr uint8_t DEBOUNCE_COUNT = 3;  // consecutive matching polls before state changes

    /// Initialize the SPI bus and CS pin.
    /// @param cs   chip select GPIO
    /// @param sck  clock GPIO
    /// @param miso data-in GPIO (QH of last 74HC165)
    /// @param numBytes number of shift registers in chain
    void setup(uint8_t cs, uint8_t sck, uint8_t miso, uint8_t numBytes);

    /// Read the shift register chain and update cached button states.
    void poll();

    /// Returns true if the given bit is currently pressed (active low → inverted).
    /// @param bitIndex 0-based bit index across the full chain (0 = MSB of first byte)
    bool isPressed(uint8_t bitIndex) const;

    /// Returns the raw byte at the given index (after inversion).
    uint8_t getRawByte(uint8_t byteIndex) const;

    /// Returns true if setup() has been called successfully.
    bool isReady() const { return _ready; }

    /// Returns the total number of bits in the shift register chain.
    uint8_t getNumBits() const { return _numBytes * 8; }

private:
    SPIClass *_spi = nullptr;
    uint8_t _csPin = 0;
    uint8_t _numBytes = 0;
    uint8_t _data[MAX_BYTES] = {};       // debounced output
    uint8_t _raw[MAX_BYTES] = {};        // latest SPI read (inverted)
    uint8_t _counters[MAX_BITS] = {};    // per-bit debounce counters
    bool _ready = false;
};
