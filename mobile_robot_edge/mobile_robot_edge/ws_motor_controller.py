#!/usr/bin/env python3
"""
WebSocket Motor Controller Node

This node subscribes to /cmd_vel and forwards commands to edge_services
via WebSocket.

WebSocket Connection: ws://localhost:8001 (edge_services/motor_service.py)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import json
import threading
from queue import Queue


class WebSocketMotorController(Node):
    def __init__(self):
        super().__init__('ws_motor_controller')

        # Declare parameters
        self.declare_parameter('ws_uri', 'ws://localhost:8001')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Get parameters
        self.ws_uri = self.get_parameter('ws_uri').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Command queue for thread-safe communication
        self.cmd_queue = Queue()

        # WebSocket connection status
        self.ws_connected = False

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Start WebSocket client in separate thread
        self.ws_thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        self.ws_thread.start()

        self.get_logger().info('WebSocket Motor Controller initialized')
        self.get_logger().info(f'Subscribing to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Connecting to: {self.ws_uri}')

    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel subscription"""
        # Create command message for edge service
        cmd = {
            'type': 'cmd_vel',
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }

        # Add to queue for WebSocket thread
        self.cmd_queue.put(cmd)

        if not self.ws_connected:
            self.get_logger().warn('WebSocket not connected, command queued')

    def run_websocket_client(self):
        """Run WebSocket client in separate thread"""
        asyncio.run(self.websocket_client())

    async def websocket_client(self):
        """WebSocket client coroutine"""
        while True:
            try:
                self.get_logger().info(f'Connecting to {self.ws_uri}...')
                async with websockets.connect(self.ws_uri) as websocket:
                    self.ws_connected = True
                    self.get_logger().info('WebSocket connected')

                    while True:
                        try:
                            # Check if there's a command to send
                            if not self.cmd_queue.empty():
                                cmd = self.cmd_queue.get()
                                await websocket.send(json.dumps(cmd))

                                # Receive response
                                response = await asyncio.wait_for(
                                    websocket.recv(),
                                    timeout=1.0
                                )
                                resp_data = json.loads(response)

                                if resp_data.get('status') != 'ok':
                                    self.get_logger().warn(
                                        f'Motor command failed: {resp_data.get("message")}'
                                    )

                            else:
                                # No command, just wait a bit
                                await asyncio.sleep(0.01)

                        except asyncio.TimeoutError:
                            self.get_logger().warn('Response timeout from motor service')
                        except websockets.exceptions.ConnectionClosed:
                            self.get_logger().warn('WebSocket connection closed')
                            self.ws_connected = False
                            break
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f'JSON decode error: {e}')
                        except Exception as e:
                            self.get_logger().error(f'Error in WebSocket loop: {e}')

            except Exception as e:
                self.get_logger().error(f'WebSocket connection failed: {e}')
                self.ws_connected = False
                await asyncio.sleep(5.0)  # Retry after 5 seconds


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketMotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
