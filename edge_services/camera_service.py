#!/usr/bin/env python3
"""
Camera Service

Captures images from Raspberry Pi Camera Module 3 (IMX708) via picamera2/libcamera
and streams via WebSocket.

Communication:
- Protocol: WebSocket (Binary)
- Default port: 8003
- Message format: Binary JPEG data with header
  Header (8 bytes): [timestamp (4B float LE), width (2B LE), height (2B LE)]
  Body: JPEG compressed image data
"""

import asyncio
import json
import websockets
import logging
import time
import struct
from typing import Set, Optional
import io

try:
    from picamera2 import Picamera2
    import numpy as np
    import cv2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False
    logging.warning("picamera2 not available — install: sudo apt install python3-picamera2")


class CameraService:
    """Camera capture and streaming service using picamera2 (libcamera)"""

    def __init__(self, config: dict):
        self.config = config
        self.logger = logging.getLogger(__name__)

        self.width   = config.get('width', 640)
        self.height  = config.get('height', 480)
        self.fps     = config.get('fps', 30)
        self.quality = config.get('jpeg_quality', 80)
        self.host    = config.get('host', '0.0.0.0')
        self.port    = config.get('port', 8003)

        self._cam: Optional[Picamera2] = None
        self.clients: Set = set()
        self.frame_count = 0

        self.logger.info("Camera Service initialized (picamera2/libcamera)")
        self.logger.info(f"Resolution: {self.width}x{self.height} @ {self.fps}fps")
        self.logger.info(f"WebSocket server: {self.host}:{self.port}")

    def init_camera(self) -> bool:
        if not PICAMERA2_AVAILABLE:
            self.logger.error("picamera2 not available")
            return False
        try:
            self._cam = Picamera2()
            cfg = self._cam.create_video_configuration(
                main={"size": (self.width, self.height), "format": "BGR888"},
                controls={"AfMode": 0, "LensPosition": 2.0},  # fixed focus
            )
            self._cam.configure(cfg)
            self._cam.start()
            time.sleep(0.5)  # warm-up
            self.logger.info(
                f"Camera started: {self.width}x{self.height} "
                f"(IMX708 via libcamera)"
            )
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize camera: {e}")
            return False

    def capture_frame(self) -> Optional[bytes]:
        if self._cam is None:
            return None
        try:
            frame = self._cam.capture_array("main")  # BGR numpy array
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                return None
            header = struct.pack('<fHH', time.time(), self.width, self.height)
            self.frame_count += 1
            return header + jpeg.tobytes()
        except Exception as e:
            self.logger.error(f"Error capturing frame: {e}")
            return None

    async def camera_publisher(self):
        """Publish camera frames at fixed rate using deadline-based scheduling."""
        interval = 1.0 / self.fps
        next_time = time.time() + interval

        while True:
            if not self.clients:
                now = time.time()
                sleep_sec = next_time - now
                if sleep_sec > 0:
                    await asyncio.sleep(sleep_sec)
                next_time += interval
                continue

            frame_data = await asyncio.get_event_loop().run_in_executor(
                None, self.capture_frame
            )
            if frame_data is not None:
                disconnected = set()
                for client in list(self.clients):
                    try:
                        await client.send(frame_data)
                    except websockets.exceptions.ConnectionClosed:
                        disconnected.add(client)
                    except Exception as e:
                        self.logger.error(f"Error sending frame: {e}")
                        disconnected.add(client)
                self.clients -= disconnected

            # Sleep until next deadline (absorbs capture + send time)
            now = time.time()
            sleep_sec = next_time - now
            if sleep_sec > 0:
                await asyncio.sleep(sleep_sec)
            next_time += interval

    async def handle_client(self, websocket):
        client_addr = websocket.remote_address
        self.logger.info(f"Client connected: {client_addr}")
        self.clients.add(websocket)
        try:
            info = {
                'type': 'camera_info',
                'width': self.width,
                'height': self.height,
                'fps': self.fps,
                'format': 'jpeg'
            }
            await websocket.send(json.dumps(info))
            async for _ in websocket:
                pass
        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client disconnected: {client_addr}")
        finally:
            self.clients.discard(websocket)

    async def run(self):
        self.logger.info("Starting camera service...")
        if not self.init_camera():
            self.logger.error("Failed to initialize camera — service not started")
            return
        publisher_task = asyncio.create_task(self.camera_publisher())
        async with websockets.serve(self.handle_client, self.host, self.port):
            self.logger.info(f"WebSocket server listening on ws://{self.host}:{self.port}")
            await asyncio.Future()

    def shutdown(self):
        self.logger.info("Shutting down camera service...")
        if self._cam is not None:
            self._cam.stop()
            self._cam.close()
            self.logger.info(f"Total frames captured: {self.frame_count}")


async def main():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    config = {
        'width': 640,
        'height': 480,
        'fps': 30,
        'jpeg_quality': 80,
        'host': '0.0.0.0',
        'port': 8003,
    }
    service = CameraService(config)
    try:
        await service.run()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received")
    finally:
        service.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
