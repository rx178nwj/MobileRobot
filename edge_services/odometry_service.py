#!/usr/bin/env python3
"""
Odometry Service

Reads wheel encoders and publishes odometry data via WebSocket.

Features:
- Encoder reading from PR2040 motor driver via USB Serial
- Odometry calculation (position, velocity)
- WebSocket broadcaster
- Differential drive kinematics

Communication:
- Protocol: WebSocket (JSON)
- Default port: 8002
- Message format:
  {
    "type": "odom",
    "timestamp": 1234567.89,
    "pose": {
      "x": 0.0,      # meters
      "y": 0.0,      # meters
      "theta": 0.0   # radians
    },
    "twist": {
      "linear": 0.0,  # m/s
      "angular": 0.0  # rad/s
    },
    "encoders": [0, 0, 0, 0]
  }
"""

import asyncio
import websockets
import json
import logging
import math
import time
from typing import Set
from hardware.pr2040_usb_driver import PR2040USBDriver


class OdometryService:
    """Odometry calculation and broadcasting service"""

    def __init__(self, config: dict):
        """
        Initialize odometry service

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Robot parameters
        self.wheel_base = config.get('wheel_base', 0.16)  # meters
        self.wheel_radius = config.get('wheel_radius', 0.033)  # meters
        self.encoder_cpr = config.get('encoder_cpr', 720)  # counts per revolution
        self.publish_rate = config.get('publish_rate', 50.0)  # Hz

        # WebSocket server config
        self.host = config.get('host', '0.0.0.0')
        self.port = config.get('port', 8002)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Previous encoder values
        self.prev_encoders = [0, 0, 0, 0]
        self.prev_time = time.time()

        # Connected clients
        self.clients: Set[websockets.WebSocketServerProtocol] = set()

        # Hardware - USB Serial connection (shared driver or new instance)
        if 'driver' in config:
            self.driver = config['driver']
        else:
            usb_port = config.get('usb_port', '/dev/ttyACM0')
            self.driver = PR2040USBDriver(port=usb_port)

        self.logger.info("Odometry Service initialized")
        self.logger.info(f"Wheel base: {self.wheel_base}m, radius: {self.wheel_radius}m")
        self.logger.info(f"Publish rate: {self.publish_rate}Hz")
        self.logger.info(f"WebSocket server: {self.host}:{self.port}")

    def counts_to_meters(self, counts: int) -> float:
        """
        Convert encoder counts to distance in meters

        Args:
            counts: Encoder counts

        Returns:
            float: Distance in meters
        """
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        meters_per_count = wheel_circumference / self.encoder_cpr
        return counts * meters_per_count

    def calculate_odometry(self, encoders: tuple) -> dict:
        """
        Calculate odometry from encoder readings

        Differential drive kinematics:
        distance = (left_dist + right_dist) / 2
        delta_theta = (right_dist - left_dist) / wheel_base

        Args:
            encoders: Tuple of (enc0, enc1, enc2, enc3) counts

        Returns:
            dict: Odometry data
        """
        current_time = time.time()
        dt = current_time - self.prev_time

        if dt == 0:
            return self._create_odom_message()

        # Calculate encoder deltas
        # Left side: average of wheels 0 and 2
        # Right side: average of wheels 1 and 3
        left_enc = (encoders[0] + encoders[2]) / 2.0
        right_enc = (encoders[1] + encoders[3]) / 2.0

        prev_left = (self.prev_encoders[0] + self.prev_encoders[2]) / 2.0
        prev_right = (self.prev_encoders[1] + self.prev_encoders[3]) / 2.0

        delta_left = left_enc - prev_left
        delta_right = right_enc - prev_right

        # Convert to meters
        left_dist = self.counts_to_meters(delta_left)
        right_dist = self.counts_to_meters(delta_right)

        # Calculate robot motion
        distance = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_base

        # Update pose
        self.x += distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities
        self.linear_vel = distance / dt
        self.angular_vel = delta_theta / dt

        # Update previous values
        self.prev_encoders = list(encoders)
        self.prev_time = current_time

        return self._create_odom_message()

    def _create_odom_message(self) -> dict:
        """Create odometry message"""
        return {
            'type': 'odom',
            'timestamp': time.time(),
            'pose': {
                'x': self.x,
                'y': self.y,
                'theta': self.theta
            },
            'twist': {
                'linear': self.linear_vel,
                'angular': self.angular_vel
            },
            'encoders': self.prev_encoders
        }

    def reset_odometry(self):
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.prev_time = time.time()
        self.logger.info("Odometry reset")

    async def odometry_publisher(self):
        """Publish odometry at fixed rate"""
        interval = 1.0 / self.publish_rate

        while True:
            # Driver is non-blocking (background reader thread)
            try:
                encoders = self.driver.read_all_encoders()

                # Calculate odometry
                odom_msg = self.calculate_odometry(encoders)

                # Broadcast to all connected clients
                if self.clients:
                    message = json.dumps(odom_msg)
                    disconnected = set()

                    for client in self.clients:
                        try:
                            await client.send(message)
                        except websockets.exceptions.ConnectionClosed:
                            disconnected.add(client)

                    # Remove disconnected clients
                    self.clients -= disconnected

            except Exception as e:
                self.logger.error(f"Error in odometry publisher: {e}")

            await asyncio.sleep(interval)

    async def handle_client(self, websocket):
        """
        Handle WebSocket client connection

        Args:
            websocket: WebSocket connection
        """
        client_addr = websocket.remote_address
        self.logger.info(f"Client connected: {client_addr}")

        # Add client to set
        self.clients.add(websocket)

        try:
            # Send initial odometry
            odom_msg = self._create_odom_message()
            await websocket.send(json.dumps(odom_msg))

            # Handle incoming messages (commands)
            async for message in websocket:
                try:
                    data = json.loads(message)

                    if data.get('type') == 'reset':
                        self.reset_odometry()
                        self.driver.reset_encoders()
                        await websocket.send(json.dumps({'type': 'ack', 'reset': True}))

                    elif data.get('type') == 'get_status':
                        status = self.driver.get_status()
                        await websocket.send(json.dumps({'type': 'status', 'data': status}))

                except json.JSONDecodeError as e:
                    self.logger.error(f"Invalid JSON: {e}")
                except Exception as e:
                    self.logger.error(f"Error processing message: {e}")

        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client disconnected: {client_addr}")
        finally:
            # Remove client from set
            self.clients.discard(websocket)

    async def run(self):
        """Run the odometry service"""
        self.logger.info("Starting odometry service...")

        # Start odometry publisher task
        publisher_task = asyncio.create_task(self.odometry_publisher())

        # Start WebSocket server
        async with websockets.serve(self.handle_client, self.host, self.port):
            self.logger.info(f"WebSocket server listening on ws://{self.host}:{self.port}")
            await asyncio.Future()  # Run forever

    def shutdown(self):
        """Shutdown service"""
        self.logger.info("Shutting down odometry service...")
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
        'wheel_base': 0.16,      # meters
        'wheel_radius': 0.033,   # meters
        'encoder_cpr': 827.2,    # counts per revolution (calibrated)
        'publish_rate': 50.0,    # Hz
        'host': '0.0.0.0',
        'port': 8002,
        'usb_port': '/dev/ttyACM0'  # USB serial port
    }

    # Create and run service
    service = OdometryService(config)

    try:
        await service.run()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received")
    finally:
        service.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
