#pragma once
#include <Arduino.h>
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "Config.h"

// Control mode
enum class ControlMode : uint8_t {
    DIRECT   = 0,  // Direct PWM duty (-PWM_DUTY_MAX .. +PWM_DUTY_MAX)
    VELOCITY = 1,  // Velocity PID   (target: counts/sec)
    POSITION = 2,  // Position cascade: outer-P -> velocity-PI -> duty
};

// Per-wheel controller
//
// Motor control loop (runs at CONTROL_INTERVAL_MS):
//
//   DIRECT   : duty set directly, no feedback
//
//   VELOCITY : vel_error = target_vel - measured_vel
//              duty = PID(vel_error)
//
//   POSITION : pos_error = target_pos - current_pos
//              vel_target = clamp(posKp * pos_error, ±maxVelCps)
//              duty = PID_velocity(vel_target - measured_vel)   [inner PI]
//
// Default PID gains:
//   Velocity : Kp=1.0  Ki=0.05  Kd=0.0
//   Position : posKp=0.5  maxVelCps=2000  (inner vel PID same as above)

class WheelController {
public:
    // Embed Motor and Encoder; encoderIndex (0-3) passed to begin()
    WheelController(uint8_t in1Pin, uint8_t in2Pin,
                    uint8_t encAPin, uint8_t encBPin);

    // Call once in setup(). encoderIndex must be unique across all instances (0-3).
    void begin(uint8_t encoderIndex);

    // --- Mode ---
    void        setMode(ControlMode mode);
    ControlMode getMode() const { return _mode; }

    // --- DIRECT ---
    void    setDuty(int32_t duty);

    // --- VELOCITY ---
    // target: counts per second (signed)
    void  setVelocityTarget(float cps);
    float getVelocityTarget() const { return _velPID.getTarget(); }
    float getVelocity()       const { return _velocity; }
    int32_t getVelocityInt()  const { return (int32_t)_velocity; }

    // --- POSITION ---
    // target: absolute encoder count
    void    setPositionTarget(int32_t counts);
    int32_t getPositionTarget() const { return _posTarget; }
    int32_t getPosition()       const { return _encoder.getCount(); }

    // --- PID configuration ---
    // Velocity inner loop (used in both VELOCITY and POSITION modes)
    void setVelocityPID(float kp, float ki, float kd);
    // Position outer loop
    void setPositionGains(float posKp, float maxVelCps);

    float getPosKp()      const { return _posKp; }
    float getMaxVelCps()  const { return _maxVelCps; }

    // --- Control update (call from loop() at CONTROL_INTERVAL_MS) ---
    void update(uint32_t dtMs);

    // --- Encoder ---
    void resetEncoder();  // reset encoder count to zero

    // --- Safety ---
    void stop();   // coast, switches to DIRECT
    void brake();  // active brake, switches to DIRECT

private:
    Motor   _motor;
    Encoder _encoder;

    ControlMode _mode;

    // Velocity PID (inner loop for both VELOCITY and POSITION)
    PID _velPID;

    // Position outer loop
    float   _posKp;
    float   _maxVelCps;
    int32_t _posTarget;

    // Within this many counts of target → stop motor (prevents oscillation)
    static constexpr int32_t POS_DEADBAND = 5;

    // Velocity measurement
    int32_t _prevCount;
    float   _velocity;   // low-pass filtered velocity (counts/sec)

    // Low-pass filter coefficient: 0 = no filtering, 1 = never updates
    static constexpr float VEL_ALPHA = 0.4f;
};
