#pragma once

#include <Arduino.h>
#include <Wire.h>

// =============================================================================
// S-5851A Digital Temperature Sensor Driver
// =============================================================================
// - I2C interface, 12-bit temperature sensor
// - Resolution: 0.0625°C (1 LSB)
// - Operating range: -30°C to +100°C (typical)
// - AD0=GND, AD1=GND → I2C address = 0x48
// =============================================================================

class S5851A {
public:
  // I2C Address when AD0=GND, AD1=GND
  static constexpr uint8_t I2C_ADDR = 0x48;

  // Register addresses
  static constexpr uint8_t REG_TEMPERATURE = 0x00;  // Temperature register (12-bit, read-only)
  static constexpr uint8_t REG_CONFIG      = 0x01;  // Configuration register
  static constexpr uint8_t REG_T_LOW       = 0x02;  // Low temperature threshold
  static constexpr uint8_t REG_T_HIGH      = 0x03;  // High temperature threshold

  S5851A(TwoWire& wire = Wire, uint8_t addr = I2C_ADDR);

  // Initialize sensor
  bool begin();

  // Read temperature in Celsius
  // Returns temperature value or NAN on error
  float readTemperature();

  // Read raw 12-bit temperature value
  // Returns raw value (signed 12-bit) or INT16_MIN on error
  int16_t readRaw();

  // Check if sensor is connected
  bool isConnected();

private:
  TwoWire* _wire;
  uint8_t _addr;

  // Read 16-bit register (big-endian)
  bool readRegister(uint8_t reg, uint16_t& value);
};
