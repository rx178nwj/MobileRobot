#!/usr/bin/env python3
"""
Motor Control Service — Velocity PI + Feedforward

Receives velocity commands via WebSocket and controls motors via USB Serial.

Control architecture:
  cmd_vel → target_cps → [PI + FF] → duty_cycle → PR2040 (DIRECT mode)
                              ↑
                     actual_cps (encoder, 100Hz)

The PI + Feedforward controller handles motor dead zones automatically:
  output = Kff * target_cps          ← feedforward (ballpark duty estimate)
           + Kp  * error             ← proportional correction
           + Ki  * integral(error)   ← steady-state error elimination
  + dead_zone_kickstart              ← ensures |output| >= min_duty when moving

Communication:
  Protocol: WebSocket (JSON)
  Port:     8001
  Message:  {"type": "cmd_vel", "linear": 0.2, "angular": 0.5}
"""

import asyncio
import websockets
import json
import logging
import math
import time
from hardware.pr2040_usb_driver import PR2040USBDriver


# ---------------------------------------------------------------------------
# Velocity PI + Feedforward controller (per wheel)
# ---------------------------------------------------------------------------

class WheelVelocityController:
    """
    Single-wheel velocity PI controller with calibrated feedforward.

    Control law:
        ff     = ff_offset + ff_gain * |target_cps|   (offset model)
        output = sign(target) * ff + Kp * error + Ki * integral(error)
        duty   = sign(output) * max(min_duty, |output|)   if target != 0
               = 0                                         if target ≈ 0

    Offset feedforward model  (calibrated by duty_sweep.py):
        duty = ff_offset + ff_gain * cps
             = 402      + 0.0595  * cps
        Calibration points (measured):
          duty=460 → 853 cps   duty=500 → 1685 cps
          duty=550 → 2683 cps  duty=700 → 4910 cps
          duty=800 → 6700 cps

    Dead zone kickstart (safety net):
        If the FF+PI output is still below min_duty, clamp to min_duty.
        min_duty = 480  (measured dead zone threshold + margin)

    Anti-windup:
        Integral clamped to ±max_integral to prevent runaway.
    """

    def __init__(
        self,
        kp: float           = 0.05,
        ki: float           = 0.08,
        ff_offset: float    = 402.0,   # calibrated dead zone offset [duty]
        ff_gain: float      = 0.0595,  # calibrated slope [duty/cps]
        min_duty: int       = 480,     # measured dead zone threshold + margin
        max_duty: int       = 1000,
        max_integral: float = 400.0,   # anti-windup limit [duty units]
    ):
        self.kp = kp
        self.ki = ki
        self.ff_offset = ff_offset
        self.ff_gain   = ff_gain
        self.min_duty  = min_duty
        self.max_duty  = max_duty
        self.max_integral = max_integral

        self._integral  = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, target_cps: float, actual_cps: float, dt: float) -> int:
        """
        Compute duty cycle (-1000..+1000).

        Args:
            target_cps: Desired wheel velocity [counts/sec]
            actual_cps: Measured wheel velocity [counts/sec] from encoder
            dt:         Time since last call [seconds]

        Returns:
            int: Duty cycle for DIRECT mode (-1000 to +1000)
        """
        # Stop immediately when target is near zero
        if abs(target_cps) < 1.0:
            self.reset()
            return 0

        if dt <= 0.0:
            return 0

        sign  = 1 if target_cps > 0 else -1
        error = target_cps - actual_cps

        # Integral with anti-windup
        self._integral += error * dt
        self._integral  = max(-self.max_integral,
                              min(self.max_integral, self._integral))

        # Calibrated offset feedforward: duty = ff_offset + ff_gain * |target|
        ff     = self.ff_offset + self.ff_gain * abs(target_cps)
        output = sign * ff + self.kp * error + self.ki * self._integral

        # Dead zone kickstart (safety net): ensure |duty| >= min_duty
        if abs(output) < self.min_duty:
            output = sign * self.min_duty

        return int(max(-self.max_duty, min(self.max_duty, output)))


# ---------------------------------------------------------------------------
# Motor Control Service
# ---------------------------------------------------------------------------

class MotorControlService:
    """
    Motor control service with WebSocket interface.

    WebSocket handler: receives cmd_vel → sets _target_left/right_cps
    pid_loop (50Hz):   reads encoder velocities → PI+FF → set_wheel_duties
    """

    def __init__(self, config: dict):
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Robot parameters
        self.wheel_base      = config.get('wheel_base', 0.16)      # [m]
        self.wheel_radius    = config.get('wheel_radius', 0.033)    # [m]
        self.encoder_cpr     = config.get('encoder_cpr', 827.2)     # counts/rev (calibrated)
        self.max_linear_vel  = config.get('max_linear_velocity', 0.5)   # [m/s]
        self.max_angular_vel = config.get('max_angular_velocity', 2.0)  # [rad/s]
        self.cmd_timeout     = config.get('cmd_vel_timeout', 0.5)   # [s]

        # WebSocket server
        self.host = config.get('host', '0.0.0.0')
        self.port = config.get('port', 8001)

        # State
        self.last_cmd_time   = time.time()
        self.current_linear  = 0.0
        self.current_angular = 0.0

        # PID velocity targets [counts/sec]; updated by WebSocket, consumed by pid_loop
        self._target_left_cps  = 0.0
        self._target_right_cps = 0.0

        # PI + FF controllers for left / right wheels
        pid_cfg = config.get('pid', {})
        ctrl_args = dict(
            kp           = pid_cfg.get('kp',          0.05),
            ki           = pid_cfg.get('ki',          0.08),
            ff_offset    = pid_cfg.get('ff_offset',   402.0),
            ff_gain      = pid_cfg.get('ff_gain',     0.0595),
            min_duty     = pid_cfg.get('min_duty',    480),
            max_duty     = pid_cfg.get('max_duty',    1000),
            max_integral = pid_cfg.get('max_integral', 400.0),
        )
        self.left_ctrl  = WheelVelocityController(**ctrl_args)
        self.right_ctrl = WheelVelocityController(**ctrl_args)

        # PID loop period [s]
        self._pid_dt = config.get('pid_dt', 0.02)   # 50 Hz

        # Hardware
        if 'driver' in config:
            self.driver = config['driver']
        else:
            self.driver = PR2040USBDriver(port=config.get('usb_port', '/dev/ttyACM0'))

        self.logger.info(
            f"MotorControlService: wheel_base={self.wheel_base}m "
            f"radius={self.wheel_radius}m cpr={self.encoder_cpr} | "
            f"kp={ctrl_args['kp']} ki={ctrl_args['ki']} "
            f"ff_offset={ctrl_args['ff_offset']:.0f} ff_gain={ctrl_args['ff_gain']:.4f} "
            f"min_duty={ctrl_args['min_duty']}"
        )

    # ------------------------------------------------------------------
    # Kinematics helpers
    # ------------------------------------------------------------------

    def _velocity_to_cps(self, vel_mps: float) -> float:
        """Convert linear velocity [m/s] → wheel velocity [counts/sec]."""
        circumference = 2.0 * math.pi * self.wheel_radius
        return (vel_mps / circumference) * self.encoder_cpr

    def _cmd_vel_to_wheel_cps(self, linear: float, angular: float):
        """
        Differential drive kinematics → (left_cps, right_cps).

        v_left  = v - ω * L/2
        v_right = v + ω * L/2
        """
        linear  = max(-self.max_linear_vel,  min(self.max_linear_vel,  linear))
        angular = max(-self.max_angular_vel, min(self.max_angular_vel, angular))

        left_cps  = self._velocity_to_cps(linear - angular * self.wheel_base / 2.0)
        right_cps = self._velocity_to_cps(linear + angular * self.wheel_base / 2.0)
        return left_cps, right_cps

    # ------------------------------------------------------------------
    # Public motor control interface
    # ------------------------------------------------------------------

    def set_motor_velocities(self, linear: float, angular: float):
        """Update velocity targets. The pid_loop applies them to hardware."""
        left_cps, right_cps = self._cmd_vel_to_wheel_cps(linear, angular)
        self._target_left_cps  = left_cps
        self._target_right_cps = right_cps
        self.last_cmd_time     = time.time()
        self.current_linear    = linear
        self.current_angular   = angular
        self.logger.debug(
            f"target: linear={linear:.3f} angular={angular:.3f} "
            f"→ left={left_cps:.1f} right={right_cps:.1f} cps"
        )

    def stop_motors(self):
        """Immediately stop all motors and reset PID state."""
        self._target_left_cps  = 0.0
        self._target_right_cps = 0.0
        self.left_ctrl.reset()
        self.right_ctrl.reset()
        self.driver.stop_all()
        self.current_linear  = 0.0
        self.current_angular = 0.0
        self.logger.info("Motors stopped")

    # ------------------------------------------------------------------
    # PID loop (asyncio task, 50 Hz)
    # ------------------------------------------------------------------

    async def pid_loop(self):
        """
        Velocity PI + Feedforward control loop.

        Wheel layout (4WD differential, mirrored right-side mounting):
          Left : wheels 0, 2   — velocity sign: positive = forward
          Right: wheels 1, 3   — negate duty AND velocity for mirrored mount

        Encoder velocities are read from the PR2040's 100Hz auto-status
        cache (no serial round-trip per iteration).
        """
        self.logger.info(f"PID loop started at {1/self._pid_dt:.0f} Hz")
        prev_time = time.time()

        while True:
            await asyncio.sleep(self._pid_dt)
            now = time.time()
            dt  = now - prev_time
            prev_time = now

            # Safety timeout → stop
            if (now - self.last_cmd_time) > self.cmd_timeout:
                if self._target_left_cps != 0.0 or self._target_right_cps != 0.0:
                    self.stop_motors()
                    self.logger.warning("cmd_vel timeout — motors stopped")
                continue

            # Read actual velocities from cached PR2040 status (non-blocking)
            status = self.driver.get_status()
            vel = status['velocities']           # (v0, v1, v2, v3) [cps, int32]
            actual_left  =  float(vel[0])        # wheel 0: left front
            actual_right = -float(vel[1])        # wheel 1: right front (mirrored → negate)

            # PI + FF
            left_duty  = self.left_ctrl.compute(
                self._target_left_cps,  actual_left,  dt)
            right_duty = self.right_ctrl.compute(
                self._target_right_cps, actual_right, dt)

            # Apply (right side negated for mirrored mounting)
            self.driver.set_wheel_duties(
                (left_duty, -right_duty, left_duty, -right_duty))

            self.logger.debug(
                f"PID tgt=({self._target_left_cps:.0f},{self._target_right_cps:.0f}) "
                f"act=({actual_left:.0f},{actual_right:.0f}) "
                f"duty=({left_duty},{right_duty})"
            )

    # ------------------------------------------------------------------
    # WebSocket handler
    # ------------------------------------------------------------------

    async def handle_client(self, websocket):
        client_addr = websocket.remote_address
        self.logger.info(f"Client connected: {client_addr}")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get('type') == 'cmd_vel':
                        linear  = data.get('linear',  0.0)
                        angular = data.get('angular', 0.0)
                        self.set_motor_velocities(linear, angular)
                        await websocket.send(json.dumps({
                            'type': 'ack',
                            'linear':  self.current_linear,
                            'angular': self.current_angular,
                        }))
                    elif data.get('type') == 'stop':
                        self.stop_motors()
                        await websocket.send(json.dumps({'type': 'ack', 'stopped': True}))
                    else:
                        self.logger.warning(f"Unknown message type: {data.get('type')}")
                except json.JSONDecodeError as e:
                    self.logger.error(f"Invalid JSON: {e}")
                except Exception as e:
                    self.logger.error(f"Error processing message: {e}")
        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client disconnected: {client_addr}")
        finally:
            self.stop_motors()

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    async def run(self):
        self.logger.info("Starting motor control service...")
        asyncio.create_task(self.pid_loop())
        async with websockets.serve(self.handle_client, self.host, self.port):
            self.logger.info(f"WebSocket listening on ws://{self.host}:{self.port}")
            await asyncio.Future()

    def shutdown(self):
        self.logger.info("Shutting down motor control service...")
        self.driver.stop_all()
        if 'driver' not in self.config:
            self.driver.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

async def main():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    config = {
        # Robot dimensions
        'wheel_base':           0.16,    # [m]
        'wheel_radius':         0.033,   # [m]
        'encoder_cpr':          827.2,   # counts/rev (calibrated)
        'max_linear_velocity':  0.5,     # [m/s]
        'max_angular_velocity': 2.0,     # [rad/s]
        'cmd_vel_timeout':      0.5,     # [s]

        # WebSocket
        'host': '0.0.0.0',
        'port': 8001,
        'usb_port': '/dev/ttyACM0',

        # PID loop
        'pid_dt': 0.02,  # 50 Hz

        # PI + Feedforward gains  (calibrated by duty_sweep.py 2026-03-29)
        #
        # Calibration data (measured):
        #   duty=460 → 853 cps (0.21 m/s)   duty=500 → 1685 cps (0.42 m/s)
        #   duty=550 → 2683 cps (0.67 m/s)  duty=700 → 4910 cps (1.23 m/s)
        #   duty=800 → 6700 cps (1.68 m/s)
        # Linear fit: duty = ff_offset + ff_gain * cps
        #                  = 402       + 0.0595  * cps
        #
        # Verification: 0.5 m/s = 1994 cps → duty = 402 + 0.0595*1994 = 521 ✓
        #
        # Tuning guide:
        #   ff_offset  : dead zone offset. Raise if motor overshoots at startup.
        #   ff_gain    : slope. Raise if motor is too slow; lower if too fast.
        #   kp         : proportional. Lower if oscillating.
        #   ki         : integral. Raise if steady-state error remains.
        #   min_duty   : safety kickstart. Must be >= dead zone threshold (460).
        #
        'pid': {
            'ff_offset':    402.0,   # calibrated dead zone offset [duty]
            'ff_gain':      0.0595,  # calibrated slope [duty/cps]
            'kp':           0.05,
            'ki':           0.08,
            'min_duty':     480,     # calibrated dead zone threshold + margin
            'max_duty':     1000,
            'max_integral': 400.0,
        },
    }

    service = MotorControlService(config)
    try:
        await service.run()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt")
    finally:
        service.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
