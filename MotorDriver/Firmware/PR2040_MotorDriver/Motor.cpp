#include "Motor.h"
#include "Config.h"

Motor::Motor(uint8_t pinIn1, uint8_t pinIn2)
    : _pinIn1(pinIn1), _pinIn2(pinIn2), _duty(0)
{}

void Motor::begin() {
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);
}

void Motor::setDuty(int32_t duty) {
    // Clamp to valid range
    if (duty >  PWM_DUTY_MAX) duty =  PWM_DUTY_MAX;
    if (duty < -PWM_DUTY_MAX) duty = -PWM_DUTY_MAX;
    _duty = duty;

    if (duty > 0) {
        // Forward: PWM on IN1, IN2 = LOW
        digitalWrite(_pinIn2, LOW);
        analogWrite(_pinIn1, (int)duty);
    } else if (duty < 0) {
        // Reverse: IN1 = LOW, PWM on IN2
        digitalWrite(_pinIn1, LOW);
        analogWrite(_pinIn2, (int)(-duty));
    } else {
        coast();
    }
}

void Motor::coast() {
    _duty = 0;
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);
}

void Motor::brake() {
    _duty = 0;
    digitalWrite(_pinIn1, HIGH);
    digitalWrite(_pinIn2, HIGH);
}
