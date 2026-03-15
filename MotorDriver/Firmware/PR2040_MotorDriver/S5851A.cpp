#include "S5851A.h"

S5851A::S5851A(TwoWire& wire, uint8_t addr)
  : _wire(&wire), _addr(addr) {
}

bool S5851A::begin() {
  // Check if sensor is present
  if (!isConnected()) {
    return false;
  }

  // S-5851A is ready to use after power-on, no special initialization needed
  // Default configuration is usually fine (continuous conversion mode)

  return true;
}

float S5851A::readTemperature() {
  int16_t raw = readRaw();

  if (raw == INT16_MIN) {
    return NAN;  // Error
  }

  // S-5851A uses 12-bit temperature in the upper 12 bits of 16-bit register
  // Temperature = raw * 0.0625°C
  // Sign-extend 12-bit to 16-bit, then convert to temperature
  int16_t temp12bit = raw >> 4;  // Upper 12 bits

  // Check if negative (12-bit sign bit)
  if (temp12bit & 0x800) {
    temp12bit |= 0xF000;  // Sign extend to 16-bit
  }

  float temperature = temp12bit * 0.0625f;

  return temperature;
}

int16_t S5851A::readRaw() {
  uint16_t value;

  if (!readRegister(REG_TEMPERATURE, value)) {
    return INT16_MIN;  // Error
  }

  return (int16_t)value;
}

bool S5851A::isConnected() {
  _wire->beginTransmission(_addr);
  uint8_t error = _wire->endTransmission();
  return (error == 0);
}

bool S5851A::readRegister(uint8_t reg, uint16_t& value) {
  // Write register address
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  uint8_t error = _wire->endTransmission(false);  // Repeated start

  if (error != 0) {
    return false;
  }

  // Read 2 bytes (big-endian: MSB first)
  uint8_t bytesRead = _wire->requestFrom(_addr, (uint8_t)2);

  if (bytesRead != 2) {
    return false;
  }

  uint8_t msb = _wire->read();
  uint8_t lsb = _wire->read();

  value = ((uint16_t)msb << 8) | lsb;

  return true;
}
