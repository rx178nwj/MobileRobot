#include "WheelController.h"

WheelController::WheelController(uint8_t in1Pin, uint8_t in2Pin,
                                 uint8_t encAPin, uint8_t encBPin)
    : _motor(in1Pin, in2Pin),
      _encoder(encAPin, encBPin),
      _mode(ControlMode::DIRECT),
      _velPID(1.0f, 0.05f, 0.0f,
              -(float)PWM_DUTY_MAX, (float)PWM_DUTY_MAX),
      _posKp(0.5f),
      _maxVelCps(2000.0f),
      _posTarget(0),
      _prevCount(0),
      _velocity(0.0f)
{}

void WheelController::begin(uint8_t encoderIndex) {
    _motor.begin();
    _encoder.begin(encoderIndex);
    _prevCount = _encoder.getCount();
}

void WheelController::setMode(ControlMode mode) {
    if (_mode != mode) {
        _velPID.reset();
        _velocity  = 0.0f;
        _prevCount = _encoder.getCount();
    }
    _mode = mode;
}

void WheelController::setDuty(int32_t duty) {
    if (_mode != ControlMode::DIRECT) {
        _velPID.reset();
    }
    _mode = ControlMode::DIRECT;
    _motor.setDuty(duty);
}

void WheelController::setVelocityTarget(float cps) {
    if (_mode != ControlMode::VELOCITY) {
        _velPID.reset();
        _prevCount = _encoder.getCount();
    }
    _mode = ControlMode::VELOCITY;
    _velPID.setTarget(cps);
}

void WheelController::setPositionTarget(int32_t counts) {
    if (_mode != ControlMode::POSITION) {
        _velPID.reset();
        _prevCount = _encoder.getCount();
    }
    _mode = ControlMode::POSITION;
    _posTarget = counts;
}

void WheelController::setVelocityPID(float kp, float ki, float kd) {
    _velPID.setGains(kp, ki, kd);
    _velPID.reset();
}

void WheelController::setPositionGains(float posKp, float maxVelCps) {
    _posKp     = posKp;
    _maxVelCps = maxVelCps;
}

void WheelController::update(uint32_t dtMs) {
    if (dtMs == 0) return;

    float dt = dtMs * 0.001f;  // ms -> seconds

    // --- Velocity measurement ---
    int32_t currCount = _encoder.getCount();
    float rawVel = (float)(currCount - _prevCount) / dt;
    _prevCount = currCount;

    // Low-pass filter
    _velocity = VEL_ALPHA * _velocity + (1.0f - VEL_ALPHA) * rawVel;

    // --- Control ---
    switch (_mode) {

    case ControlMode::DIRECT:
        // Duty is set externally via setDuty(); nothing to do here.
        break;

    case ControlMode::VELOCITY: {
        float duty = _velPID.update(_velocity, dt);
        _motor.setDuty((int32_t)duty);
        break;
    }

    case ControlMode::POSITION: {
        int32_t posError = _posTarget - currCount;

        if (posError >= -POS_DEADBAND && posError <= POS_DEADBAND) {
            // Within deadband: stop and reset inner loop
            _motor.coast();
            _velPID.reset();
        } else {
            // Outer P: convert position error to velocity target
            float velTarget = _posKp * (float)posError;
            if (velTarget >  _maxVelCps) velTarget =  _maxVelCps;
            if (velTarget < -_maxVelCps) velTarget = -_maxVelCps;

            // Inner PI: velocity -> duty
            _velPID.setTarget(velTarget);
            float duty = _velPID.update(_velocity, dt);
            _motor.setDuty((int32_t)duty);
        }
        break;
    }
    }
}

void WheelController::resetEncoder() {
    _encoder.resetCount();
    _prevCount = 0;
    if (_mode == ControlMode::POSITION) {
        _posTarget = 0;
        _velPID.reset();
    }
}

void WheelController::stop() {
    _velPID.reset();
    _mode = ControlMode::DIRECT;
    _motor.coast();
}

void WheelController::brake() {
    _velPID.reset();
    _mode = ControlMode::DIRECT;
    _motor.brake();
}
