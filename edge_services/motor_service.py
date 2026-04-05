#!/usr/bin/env python3
"""
Motor Control Service — Firmware Velocity PID

Receives velocity commands via WebSocket and controls motors via USB Serial.
Velocity tracking is handled entirely by the PR2040 firmware (MODE_VELOCITY).

Control architecture:
  cmd_vel (m/s) → kinematics → target_cps → PR2040 firmware PID
                                                   ↑
                                         encoder feedback (internal to PR2040)

PR2040 firmware velocity PID gains (calibrated 2026-03-29):
  kp = 0.10 [duty/cps]
  ki = 0.40 [duty/(cps·s)]
  kd = 0.00
  Dead zone overcome in ~0.33s; 0.5 m/s reached in ~1.8s; steady error ±0.01 m/s.

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


class MotorControlService:
    """Motor control service. Sends CPS targets; firmware handles PID."""

    def __init__(self, config: dict):
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Robot parameters
        self.wheel_base      = config.get('wheel_base', 0.16)
        self.wheel_radius    = config.get('wheel_radius', 0.033)
        self.encoder_cpr     = config.get('encoder_cpr', 827.2)
        self.max_linear_vel  = config.get('max_linear_velocity', 0.5)
        self.max_angular_vel = config.get('max_angular_velocity', 2.0)
        self.cmd_timeout     = config.get('cmd_vel_timeout', 0.5)

        # WebSocket server
        self.host = config.get('host', '0.0.0.0')
        self.port = config.get('port', 8001)

        # State
        self.last_cmd_time   = time.time()
        self.current_linear  = 0.0
        self.current_angular = 0.0

        # Hardware (init programs firmware PID gains + MODE_VELOCITY)
        if 'driver' in config:
            self.driver = config['driver']
        else:
            self.driver = PR2040USBDriver(port=config.get('usb_port', '/dev/ttyACM0'))

        self.logger.info(
            f"MotorControlService ready: "
            f"wheel_base={self.wheel_base}m radius={self.wheel_radius}m "
            f"cpr={self.encoder_cpr} | firmware velocity PID"
        )

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def _velocity_to_cps(self, vel_mps: float) -> float:
        """Convert linear velocity [m/s] → wheel velocity [counts/sec]."""
        return (vel_mps / (2.0 * math.pi * self.wheel_radius)) * self.encoder_cpr

    def _cmd_vel_to_wheel_cps(self, linear: float, angular: float):
        """
        Differential drive kinematics → (left_cps, right_cps).
        v_left  = v - ω·L/2
        v_right = v + ω·L/2
        """
        linear  = max(-self.max_linear_vel,  min(self.max_linear_vel,  linear))
        angular = max(-self.max_angular_vel, min(self.max_angular_vel, angular))
        left_cps  = self._velocity_to_cps(linear - angular * self.wheel_base / 2.0)
        right_cps = self._velocity_to_cps(linear + angular * self.wheel_base / 2.0)
        return left_cps, right_cps

    # ------------------------------------------------------------------
    # Motor control
    # ------------------------------------------------------------------

    def set_motor_velocities(self, linear: float, angular: float):
        """Convert cmd_vel to CPS and send to PR2040 firmware PID."""
        left_cps, right_cps = self._cmd_vel_to_wheel_cps(linear, angular)

        # Right side: mirrored mounting → negate
        self.driver.set_wheel_velocities(
            (left_cps, -right_cps, left_cps, -right_cps))

        self.last_cmd_time   = time.time()
        # Store clamped values so ACK reflects actual commanded velocity
        self.current_linear  = max(-self.max_linear_vel,  min(self.max_linear_vel,  linear))
        self.current_angular = max(-self.max_angular_vel, min(self.max_angular_vel, angular))
        self.logger.debug(
            f"cmd_vel: linear={linear:.3f} angular={angular:.3f} "
            f"→ left={left_cps:.1f} right={right_cps:.1f} cps"
        )

    def stop_motors(self):
        """Immediately stop all motors."""
        self.driver.stop_all()
        self.current_linear  = 0.0
        self.current_angular = 0.0
        self.logger.info("Motors stopped")

    # ------------------------------------------------------------------
    # Safety monitor
    # ------------------------------------------------------------------

    async def safety_monitor(self):
        """Stop motors if no cmd_vel received within timeout."""
        while True:
            await asyncio.sleep(0.1)
            if (time.time() - self.last_cmd_time) > self.cmd_timeout:
                if self.current_linear != 0.0 or self.current_angular != 0.0:
                    self.stop_motors()
                    self.logger.warning("cmd_vel timeout — motors stopped")

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
                        self.set_motor_velocities(
                            data.get('linear',  0.0),
                            data.get('angular', 0.0))
                        await websocket.send(json.dumps({
                            'type':    'ack',
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
                    self.logger.error(f"Error: {e}")
        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client disconnected: {client_addr}")
        finally:
            self.stop_motors()

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    async def run(self):
        self.logger.info("Starting motor control service...")
        asyncio.create_task(self.safety_monitor())
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
        'wheel_base':           0.16,    # [m]
        'wheel_radius':         0.033,   # [m]
        'encoder_cpr':          827.2,   # counts/rev (calibrated)
        'max_linear_velocity':  0.5,     # [m/s]
        'max_angular_velocity': 2.0,     # [rad/s]
        'cmd_vel_timeout':      0.5,     # [s]
        'host':     '0.0.0.0',
        'port':     8001,
        'usb_port': '/dev/ttyACM0',
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
