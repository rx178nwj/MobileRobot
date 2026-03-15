#!/usr/bin/env python3
"""
WebSocket Odometry Publisher Node

This node receives odometry data from edge_services via WebSocket
and publishes to ROS 2 topics:
1. /odom topic (nav_msgs/Odometry)
2. TF transform: odom -> base_footprint

WebSocket Connection: ws://localhost:8002 (edge_services/odometry_service.py)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import asyncio
import websockets
import json
import math
import threading


class WebSocketOdometryPublisher(Node):
    def __init__(self):
        super().__init__('ws_odometry_publisher')

        # Declare parameters
        self.declare_parameter('ws_uri', 'ws://localhost:8002')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')

        # Get parameters
        self.ws_uri = self.get_parameter('ws_uri').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # WebSocket connection
        self.ws_connected = False
        self.ws_thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        self.ws_thread.start()

        self.get_logger().info(f'WebSocket Odometry Publisher initialized')
        self.get_logger().info(f'Connecting to: {self.ws_uri}')

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
                            # Receive odometry data from edge service
                            message = await websocket.recv()
                            data = json.loads(message)

                            if data.get('type') == 'odom':
                                self.publish_odometry(data)

                        except websockets.exceptions.ConnectionClosed:
                            self.get_logger().warn('WebSocket connection closed')
                            self.ws_connected = False
                            break
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f'JSON decode error: {e}')
                        except Exception as e:
                            self.get_logger().error(f'Error processing message: {e}')

            except Exception as e:
                self.get_logger().error(f'WebSocket connection failed: {e}')
                self.ws_connected = False
                await asyncio.sleep(5.0)  # Retry after 5 seconds

    def publish_odometry(self, data):
        """
        Publish odometry from edge service data

        Expected data format:
        {
            "type": "odom",
            "timestamp": 1234567.89,
            "pose": {
                "x": 0.0,
                "y": 0.0,
                "theta": 0.0
            },
            "twist": {
                "linear": 0.0,
                "angular": 0.0
            },
            "encoders": [0, 0, 0, 0]
        }
        """
        try:
            # Extract pose
            pose = data.get('pose', {})
            x = pose.get('x', 0.0)
            y = pose.get('y', 0.0)
            theta = pose.get('theta', 0.0)

            # Extract twist
            twist = data.get('twist', {})
            linear_vel = twist.get('linear', 0.0)
            angular_vel = twist.get('angular', 0.0)

            # Create timestamp
            current_time = self.get_clock().now()

            # Create quaternion from theta
            q = self.quaternion_from_euler(0, 0, theta)

            # Publish TF: odom -> base_footprint
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

            # Publish odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame_id

            # Pose
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = q

            # Velocity
            odom.twist.twist.linear.x = linear_vel
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = angular_vel

            # Set covariance (placeholder values)
            odom.pose.covariance[0] = 0.01  # x
            odom.pose.covariance[7] = 0.01  # y
            odom.pose.covariance[35] = 0.01  # theta
            odom.twist.covariance[0] = 0.01  # vx
            odom.twist.covariance[35] = 0.01  # vtheta

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketOdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
