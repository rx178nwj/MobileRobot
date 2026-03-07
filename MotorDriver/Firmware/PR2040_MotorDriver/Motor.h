#pragma once
#include <Arduino.h>

// TB67H451 H-bridge motor driver
//
// Control logic (IN/IN mode):
//   Forward : IN1 = PWM,  IN2 = LOW
//   Reverse : IN1 = LOW,  IN2 = PWM
//   Brake   : IN1 = HIGH, IN2 = HIGH  (short brake)
//   Coast   : IN1 = LOW,  IN2 = LOW   (free run)
//
// PWM duty range: -PWM_DUTY_MAX .. +PWM_DUTY_MAX  (Config.h)

class Motor {
public:
    Motor(uint8_t pinIn1, uint8_t pinIn2);

    // Call once in setup() after analogWriteFreq / analogWriteRange
    void begin();

    // duty: -PWM_DUTY_MAX to +PWM_DUTY_MAX
    //   > 0 : forward
    //   < 0 : reverse
    //   = 0 : coast
    void setDuty(int32_t duty);

    // Coast: both inputs LOW (free run)
    void coast();

    // Brake: both inputs HIGH (short brake)
    void brake();

    int32_t getDuty() const { return _duty; }

private:
    uint8_t  _pinIn1;
    uint8_t  _pinIn2;
    int32_t  _duty;
};
