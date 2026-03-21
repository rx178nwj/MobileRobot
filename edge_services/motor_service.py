#!/usr/bin/env python3
"""
Motor Control Service

Receives velocity commands via WebSocket and controls motors via USB Serial.

Features:
- WebSocket server for velocity commands
- Differential drive kinematics
- Safety timeout (auto-stop)
- Real-time motor control

Communication:
- Protocol: WebSocket (JSON)
- Default port: 8001
- Message format:
  {
    "type": "cmd_vel",
    "linear": 0.2,   # m/s
    "angular": 0.5   # rad/s
  }
"""

import asyncio
import websockets
import json
import logging
import math
import time
from typing import Optional
from hardware.pr2040_usb_driver import PR2040USBDriver


class MotorControlService:
    """Motor control service with WebSocket interface"""

    def __init__(self, config: dict):
        """
        Initialize motor control service

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Robot parameters
        self.wheel_base = config.get('wheel_base', 0.16)  # meters
        self.wheel_radius = config.get('wheel_radius', 0.033)  # meters
        self.encoder_cpr = config.get('encoder_cpr', 720)  # counts per revolution
        self.max_linear_vel = config.get('max_linear_velocity', 0.5)  # m/s
        self.max_angular_vel = config.get('max_angular_velocity', 2.0)  # rad/s
        self.cmd_timeout = config.get('cmd_vel_timeout', 0.5)  # seconds

        # WebSocket server config
        self.host = config.get('host', '0.0.0.0')
        self.port = config.get('port', 8001)

        # State
        self.last_cmd_time = time.time()
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Hardware - USB Serial connection (shared driver or new instance)
        if 'driver' in config:
            self.driver = config['driver']
        else:
            usb_port = config.get('usb_port', '/dev/ttyACM0')
            self.driver = PR2040USBDriver(port=usb_port)

        self.logger.info(f"Motor Control Service initialized")
        self.logger.info(f"Wheel base: {self.wheel_base}m, radius: {self.wheel_radius}m")
        self.logger.info(f"WebSocket server: {self.host}:{self.port}")

    def velocity_to_cps(self, velocity_mps: float) -> float:
        """
        Convert linear velocity (m/s) to encoder counts per second

        Args:
            velocity_mps: Velocity in m/s

        Returns:
            float: Encoder counts per second
        """
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        rps = velocity_mps / wheel_circumference  # Revolutions per second
        cps = rps * self.encoder_cpr  # Counts per second
        return cps

    def cmd_vel_to_wheel_velocities(self, linear: float, angular: float) -> tuple:
        """
        Convert cmd_vel (linear, angular) to individual wheel velocities

        Differential drive kinematics:
        v_left = v - (omega * L) / 2
        v_right = v + (omega * L) / 2

        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)

        Returns:
            tuple: (left_cps, right_cps) in counts/sec
        """
        # Clamp velocities
        linear = max(-self.max_linear_vel, min(self.max_linear_vel, linear))
        angular = max(-self.max_angular_vel, min(self.max_angular_vel, angular))

        # Calculate wheel velocities
        left_vel = linear - (angular * self.wheel_base / 2.0)
        right_vel = linear + (angular * self.wheel_base / 2.0)

        # Convert to encoder counts/sec
        left_cps = self.velocity_to_cps(left_vel)
        right_cps = self.velocity_to_cps(right_vel)

        return (left_cps, right_cps)

    def cps_to_duty(self, cps: float) -> int:
        """Convert counts/sec to duty cycle (-1000 to +1000).
        Empirical: duty=800 → ~5771 cps. Dead zone below duty≈400.
        Uses offset mapping: duty = MIN_DUTY + cps * (MAX_DUTY - MIN_DUTY) / MAX_CPS
        """
        if abs(cps) < 1.0:
            return 0
        MIN_DUTY = 400.0   # Minimum duty to overcome static friction
        MAX_DUTY = 1000.0
        MAX_CPS  = 7214.0  # Empirical max counts/sec at duty=1000
        duty = MIN_DUTY + abs(cps) * (MAX_DUTY - MIN_DUTY) / MAX_CPS
        duty = min(MAX_DUTY, duty)
        return int(math.copysign(duty, cps))

    def set_motor_velocities(self, linear: float, angular: float):
        """
        Set motor velocities from cmd_vel using DIRECT duty mode.

        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
        """
        # Convert to wheel velocities (counts/sec)
        left_cps, right_cps = self.cmd_vel_to_wheel_velocities(linear, angular)

        # Convert to duty cycles
        left_duty  =  self.cps_to_duty(left_cps)
        right_duty = -self.cps_to_duty(right_cps)   # mirrored mounting

        # For 4-wheel differential drive:
        # Left side: wheels 0 and 2, Right side: wheels 1 and 3
        self.driver.set_wheel_duties((left_duty, right_duty, left_duty, right_duty))

        # Update state
        self.current_linear = linear
        self.current_angular = angular
        self.last_cmd_time = time.time()

        self.logger.debug(
            f"cmd_vel: linear={linear:.3f} angular={angular:.3f} "
            f"-> left={left_cps:.1f}cps right={right_cps:.1f}cps"
        )

    def stop_motors(self):
        """Stop all motors (coast stop)."""
        self.driver.stop_all()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.logger.info("Motors stopped")

    async def safety_monitor(self):
        """Safety monitor task - stops motors if no command received"""
        while True:
            await asyncio.sleep(0.1)  # Check every 100ms

            time_since_cmd = time.time() - self.last_cmd_time

            if time_since_cmd > self.cmd_timeout:
                # No command received - stop for safety
                if self.current_linear != 0.0 or self.current_angular != 0.0:
                    self.stop_motors()
                    self.logger.warning("Safety timeout - motors stopped")

    async def handle_client(self, websocket):
        """
        Handle WebSocket client connection

        Args:
            websocket: WebSocket connection
        """
        client_addr = websocket.remote_address
        self.logger.info(f"Client connected: {client_addr}")

        try:
            async for message in websocket:
                try:
                    # Parse JSON message
                    data = json.loads(message)

                    if data.get('type') == 'cmd_vel':
                        linear = data.get('linear', 0.0)
                        angular = data.get('angular', 0.0)

                        # Driver is non-blocking (background reader thread)
                        self.set_motor_velocities(linear, angular)

                        response = {
                            'type': 'ack',
                            'linear': self.current_linear,
                            'angular': self.current_angular
                        }
                        await websocket.send(json.dumps(response))

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
            # Stop motors when client disconnects
            self.stop_motors()

    async def run(self):
        """Run the motor control service"""
        self.logger.info("Starting motor control service...")

        # Start safety monitor
        safety_task = asyncio.create_task(self.safety_monitor())

        # Start WebSocket server
        async with websockets.serve(self.handle_client, self.host, self.port):
            self.logger.info(f"WebSocket server listening on ws://{self.host}:{self.port}")
            await asyncio.Future()  # Run forever

    def shutdown(self):
        """Shutdown service"""
        self.logger.info("Shutting down motor control service...")
        self.driver.stop_all()  # Hard stop on shutdown
        if 'driver' not in self.config:
            self.driver.close()


async def main():
    """Main entry point"""
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Load configuration
    config = {
        'wheel_base': 0.16,              # meters
        'wheel_radius': 0.033,           # meters
        'encoder_cpr': 827.2,            # counts per revolution (calibrated)
        'max_linear_velocity': 0.5,      # m/s
        'max_angular_velocity': 2.0,     # rad/s
        'cmd_vel_timeout': 0.5,          # seconds
        'host': '0.0.0.0',
        'port': 8001,
        'usb_port': '/dev/ttyACM0'       # USB serial port
    }

    # Create and run service
    service = MotorControlService(config)

    try:
        await service.run()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received")
    finally:
        service.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
