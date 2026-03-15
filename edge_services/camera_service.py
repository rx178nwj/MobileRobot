#!/usr/bin/env python3
"""
Camera Service

Captures images from Raspberry Pi Camera Module 3 and streams via WebSocket.

Features:
- Camera capture (OpenCV/v4l2)
- JPEG compression
- WebSocket streaming
- Fixed focus mode (autofocus disabled for SLAM stability)

Communication:
- Protocol: WebSocket (Binary)
- Default port: 8003
- Message format: Binary JPEG data with header
  Header (8 bytes): [timestamp (4B float), width (2B), height (2B)]
  Body: JPEG compressed image data
"""

import asyncio
import websockets
import logging
import time
import struct
from typing import Set, Optional
import subprocess

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logging.warning("OpenCV not available - camera service disabled")


class CameraService:
    """Camera capture and streaming service"""

    def __init__(self, config: dict):
        """
        Initialize camera service

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Camera config
        self.device = config.get('device', '/dev/video0')
        self.width = config.get('width', 640)
        self.height = config.get('height', 480)
        self.fps = config.get('fps', 30)
        self.quality = config.get('jpeg_quality', 80)
        self.focus_value = config.get('focus_value', 450)  # Fixed focus

        # WebSocket server config
        self.host = config.get('host', '0.0.0.0')
        self.port = config.get('port', 8003)

        # State
        self.camera: Optional[cv2.VideoCapture] = None
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.frame_count = 0

        self.logger.info("Camera Service initialized")
        self.logger.info(f"Device: {self.device}, Resolution: {self.width}x{self.height}")
        self.logger.info(f"FPS: {self.fps}, JPEG quality: {self.quality}%")
        self.logger.info(f"WebSocket server: {self.host}:{self.port}")

    def disable_autofocus(self):
        """
        Disable camera autofocus and set fixed focus

        Camera Module 3 has autofocus which is problematic for SLAM.
        This sets the camera to fixed focus mode using v4l2-ctl.
        """
        try:
            # Disable continuous autofocus
            subprocess.run([
                'v4l2-ctl', '-d', self.device,
                '--set-ctrl=focus_automatic_continuous=0'
            ], check=True, capture_output=True)

            # Set fixed focus value
            subprocess.run([
                'v4l2-ctl', '-d', self.device,
                f'--set-ctrl=focus_absolute={self.focus_value}'
            ], check=True, capture_output=True)

            self.logger.info(f"Camera autofocus disabled, fixed focus set to {self.focus_value}")

        except subprocess.CalledProcessError as e:
            self.logger.error(f"Failed to set camera focus: {e}")
        except FileNotFoundError:
            self.logger.warning("v4l2-ctl not found - install v4l-utils package")

    def init_camera(self) -> bool:
        """
        Initialize camera

        Returns:
            bool: True if successful
        """
        if not CV2_AVAILABLE:
            self.logger.error("OpenCV not available")
            return False

        try:
            # Disable autofocus before opening camera
            self.disable_autofocus()

            # Open camera
            self.camera = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

            if not self.camera.isOpened():
                self.logger.error(f"Failed to open camera: {self.device}")
                return False

            # Set camera parameters
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

            # Verify settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.camera.get(cv2.CAP_PROP_FPS))

            self.logger.info(
                f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps"
            )

            return True

        except Exception as e:
            self.logger.error(f"Failed to initialize camera: {e}")
            return False

    def capture_frame(self) -> Optional[bytes]:
        """
        Capture and compress a frame

        Returns:
            Optional[bytes]: JPEG compressed frame with header, or None on error
        """
        if self.camera is None or not self.camera.isOpened():
            return None

        try:
            # Capture frame
            ret, frame = self.camera.read()

            if not ret or frame is None:
                self.logger.warning("Failed to capture frame")
                return None

            # Compress to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)

            if not ret:
                self.logger.warning("Failed to encode JPEG")
                return None

            # Create header: timestamp (4B float), width (2B), height (2B)
            timestamp = time.time()
            header = struct.pack('<fHH', timestamp, self.width, self.height)

            # Combine header + JPEG data
            message = header + jpeg.tobytes()

            self.frame_count += 1

            return message

        except Exception as e:
            self.logger.error(f"Error capturing frame: {e}")
            return None

    async def camera_publisher(self):
        """Publish camera frames at configured FPS"""
        interval = 1.0 / self.fps

        while True:
            if not self.clients:
                # No clients connected - skip frame capture
                await asyncio.sleep(interval)
                continue

            # Capture frame
            frame_data = self.capture_frame()

            if frame_data is None:
                await asyncio.sleep(interval)
                continue

            # Broadcast to all connected clients
            disconnected = set()

            for client in self.clients:
                try:
                    await client.send(frame_data)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
                except Exception as e:
                    self.logger.error(f"Error sending frame: {e}")
                    disconnected.add(client)

            # Remove disconnected clients
            self.clients -= disconnected

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
            # Send camera info
            info = {
                'type': 'camera_info',
                'width': self.width,
                'height': self.height,
                'fps': self.fps,
                'format': 'jpeg'
            }
            await websocket.send(str(info))

            # Wait for client to disconnect
            async for message in websocket:
                # Handle commands if needed
                pass

        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client disconnected: {client_addr}")
        finally:
            # Remove client from set
            self.clients.discard(websocket)

    async def run(self):
        """Run the camera service"""
        self.logger.info("Starting camera service...")

        # Initialize camera
        if not self.init_camera():
            self.logger.error("Failed to initialize camera - service not started")
            return

        # Start camera publisher task
        publisher_task = asyncio.create_task(self.camera_publisher())

        # Start WebSocket server
        async with websockets.serve(self.handle_client, self.host, self.port):
            self.logger.info(f"WebSocket server listening on ws://{self.host}:{self.port}")
            await asyncio.Future()  # Run forever

    def shutdown(self):
        """Shutdown service"""
        self.logger.info("Shutting down camera service...")
        if self.camera is not None:
            self.camera.release()
            self.logger.info(f"Total frames captured: {self.frame_count}")


async def main():
    """Main entry point"""
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Load configuration
    config = {
        'device': '/dev/video0',
        'width': 640,
        'height': 480,
        'fps': 30,
        'jpeg_quality': 80,
        'focus_value': 450,  # Fixed focus (adjust based on distance)
        'host': '0.0.0.0',
        'port': 8003
    }

    # Create and run service
    service = CameraService(config)

    try:
        await service.run()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received")
    finally:
        service.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
