#!/usr/bin/env python3
"""
Camera WebSocket Bridge Node  [Raspberry Pi / Edge side]

Receives binary JPEG frames from edge_services/camera_service (WebSocket :8003)
and publishes directly as CompressedImage — no decode/re-encode → minimal CPU &
WiFi bandwidth.

Published topics (consumed by Ubuntu Server via Zenoh):
  /camera/image_raw/compressed  (sensor_msgs/CompressedImage)  JPEG
  /camera/camera_info           (sensor_msgs/CameraInfo)

Header format from camera_service.py:
  [timestamp (4B float LE), width (2B LE), height (2B LE)] + JPEG bytes
"""

import asyncio
import struct
import threading

import websockets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


class WsCameraBridge(Node):
    def __init__(self):
        super().__init__('ws_camera_bridge')

        # Parameters — match camera_service.py config
        self.declare_parameter('ws_uri', 'ws://localhost:8003')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        # Approximate intrinsics — update after running camera calibration
        self.declare_parameter('fx', 500.0)
        self.declare_parameter('fy', 500.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        ws_uri    = self.get_parameter('ws_uri').value
        frame_id  = self.get_parameter('frame_id').value
        width     = self.get_parameter('width').value
        height    = self.get_parameter('height').value
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        self._ws_uri   = ws_uri
        self._frame_id = frame_id

        # Publishers
        self._compressed_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', 10)
        self._info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10)

        # Static camera_info — stamp updated each frame
        self._camera_info = CameraInfo()
        self._camera_info.header.frame_id = frame_id
        self._camera_info.width  = width
        self._camera_info.height = height
        self._camera_info.distortion_model = 'plumb_bob'
        self._camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self._camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        threading.Thread(target=self._run, daemon=True).start()
        self.get_logger().info(f'WsCameraBridge → {ws_uri}')

    def _run(self):
        asyncio.run(self._ws_loop())

    async def _ws_loop(self):
        while rclpy.ok():
            try:
                async with websockets.connect(self._ws_uri) as ws:
                    self.get_logger().info('Camera WebSocket connected')
                    async for message in ws:
                        if not isinstance(message, bytes) or len(message) < 8:
                            continue
                        # Parse header — JPEG data starts at byte 8
                        _ts, _w, _h = struct.unpack('<fHH', message[:8])
                        jpeg_bytes = message[8:]

                        now = self.get_clock().now().to_msg()

                        # Publish CompressedImage directly (no decode needed)
                        comp = CompressedImage()
                        comp.header.stamp    = now
                        comp.header.frame_id = self._frame_id
                        comp.format          = 'jpeg'
                        comp.data            = jpeg_bytes
                        self._compressed_pub.publish(comp)

                        # Publish camera_info with matching stamp
                        self._camera_info.header.stamp = now
                        self._info_pub.publish(self._camera_info)

            except Exception as e:
                self.get_logger().error(f'Camera WS error: {e}')
                await asyncio.sleep(3.0)


def main(args=None):
    rclpy.init(args=args)
    node = WsCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
