#pragma once
#include <Arduino.h>

// Discrete PID controller
//
// Features:
//   - Proportional on error
//   - Integral with anti-windup (back-calculation clamping)
//   - Derivative on measurement (avoids kick on setpoint change)
//   - Configurable output limits

class PID {
public:
    PID(float kp     = 1.0f,
        float ki     = 0.0f,
        float kd     = 0.0f,
        float outMin = -1000.0f,
        float outMax =  1000.0f);

    void  setGains(float kp, float ki, float kd);
    void  setOutputLimits(float outMin, float outMax);

    void  setTarget(float target) { _target = target; }
    float getTarget()       const { return _target; }

    // dt: elapsed time in seconds (must be > 0)
    float update(float measured, float dt);

    // Reset integrator and derivative state (call on mode switch)
    void  reset();

private:
    float _kp, _ki, _kd;
    float _target;
    float _integral;
    float _prevMeasured;
    float _outMin, _outMax;
    bool  _initialized;
};
