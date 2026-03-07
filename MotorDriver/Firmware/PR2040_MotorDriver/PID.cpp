#include "PID.h"

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
    : _kp(kp), _ki(ki), _kd(kd),
      _target(0.0f), _integral(0.0f), _prevMeasured(0.0f),
      _outMin(outMin), _outMax(outMax), _initialized(false)
{}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setOutputLimits(float outMin, float outMax) {
    _outMin = outMin;
    _outMax = outMax;
}

float PID::update(float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = _target - measured;

    // --- Proportional ---
    float pOut = _kp * error;

    // --- Integral with anti-windup (back-calculation) ---
    _integral += error * dt;
    float iOut = _ki * _integral;
    // Clamp integral contribution and back-calculate to prevent windup
    if (iOut > _outMax) {
        iOut      = _outMax;
        _integral = (_ki != 0.0f) ? _outMax / _ki : 0.0f;
    } else if (iOut < _outMin) {
        iOut      = _outMin;
        _integral = (_ki != 0.0f) ? _outMin / _ki : 0.0f;
    }

    // --- Derivative on measurement (no kick on setpoint step) ---
    float dOut = 0.0f;
    if (_initialized) {
        dOut = -_kd * (measured - _prevMeasured) / dt;
    }
    _prevMeasured = measured;
    _initialized  = true;

    // --- Sum and clamp output ---
    float output = pOut + iOut + dOut;
    if (output > _outMax) output = _outMax;
    if (output < _outMin) output = _outMin;

    return output;
}

void PID::reset() {
    _integral     = 0.0f;
    _prevMeasured = 0.0f;
    _initialized  = false;
}
