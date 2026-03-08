#pragma once
#include <Arduino.h>
#include <SPI.h>

// MCP3208 — 8-channel, 12-bit SPI ADC driver
//
// Wiring (SPI0):
//   MISO = SPI0_RX_PIN  (GPIO4)
//   CS   = SPI0_CSN_PIN (GPIO5)  — software CS
//   SCK  = SPI0_SCK_PIN (GPIO6)
//   MOSI = SPI0_TX_PIN  (GPIO7)
//
// Single-ended channel selection (SGL/DIFF = 1):
//   3-byte transfer:
//     TX[0] = 0x06 | (channel >> 2)   // start=1, SGL=1, D2
//     TX[1] = (channel & 0x03) << 6   // D1, D0, 6 dummy bits
//     TX[2] = 0x00                    // 8 more clock cycles
//   Result = (RX[1] & 0x0F) << 8 | RX[2]   → 12-bit value 0..4095

class MCP3208 {
public:
    MCP3208(uint8_t csPin, SPIClass& spi = SPI);

    void begin();

    // Read single channel (0..7). Returns 0..4095, or -1 on error.
    int16_t read(uint8_t channel);

    // Read all 8 channels into out[0..7].
    void readAll(int16_t out[8]);

private:
    uint8_t   _csPin;
    SPIClass& _spi;

    static constexpr uint32_t SPI_CLK_HZ = 1000000;  // 1 MHz (safe at 3.3 V)
};
