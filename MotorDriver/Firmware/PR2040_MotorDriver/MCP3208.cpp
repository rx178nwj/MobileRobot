#include "MCP3208.h"

MCP3208::MCP3208(uint8_t csPin, SPIClass& spi)
    : _csPin(csPin), _spi(spi)
{}

void MCP3208::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    // SPI pins (RX/SCK/TX) are configured by the caller via SPI.setRX/setSCK/setTX
    // before begin() is called.  We just initialise the peripheral here.
    _spi.begin();
}

int16_t MCP3208::read(uint8_t channel) {
    if (channel > 7) return -1;

    // Build 24-bit command:
    //   [0] = 0b00000110 | (channel >> 2)  → leading zeros, start=1, SGL=1, D2
    //   [1] = (channel & 0x03) << 6        → D1, D0, 6 don't-care bits
    //   [2] = 0x00                         → 8 more clocks to shift out result LSB
    uint8_t tx0 = 0x06 | (channel >> 2);
    uint8_t tx1 = (channel & 0x03) << 6;

    _spi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);

    _spi.transfer(tx0);
    uint8_t hi = _spi.transfer(tx1) & 0x0F;  // lower 4 bits = B11..B8
    uint8_t lo = _spi.transfer(0x00);         // B7..B0

    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();

    return (int16_t)((uint16_t)hi << 8 | lo);
}

void MCP3208::readAll(int16_t out[8]) {
    for (uint8_t ch = 0; ch < 8; ch++) {
        out[ch] = read(ch);
    }
}
