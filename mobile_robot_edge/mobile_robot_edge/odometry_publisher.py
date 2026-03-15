#!/usr/bin/env python3
"""
Odometry Publisher Node

This node reads wheel encoder data and publishes:
1. /odom topic (nav_msgs/Odometry) - Robot pose and velocity
2. TF transform: odom -> base_footprint

Hardware Interface: PR2040 Motor Driver via I2C
- I2C Address: 0x60
- Reads encoder counts from 4 wheels
- Calculates odometry using differential drive kinematics
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Declare parameters
        self.declare_parameter('wheel_base', 0.16)  # Distance between left/right wheels (m)
        self.declare_parameter('wheel_radius', 0.033)  # Wheel radius (m)
        self.declare_parameter('encoder_cpr', 720)  # Encoder counts per revolution
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('i2c_address', 0x60)  # Motor driver I2C address

        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.i2c_address = self.get_parameter('i2c_address').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous encoder counts
        self.prev_left_enc = 0
        self.prev_right_enc = 0
        self.prev_time = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_odometry)

        # Initialize hardware interface
        self.init_hardware()

        self.get_logger().info('Odometry Publisher initialized')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m, Wheel radius: {self.wheel_radius}m')

    def init_hardware(self):
        """
        Initialize I2C connection to motor driver

        TODO: Implement actual I2C initialization
        Example:
            import smbus2
            self.i2c_bus = smbus2.SMBus(1)  # I2C bus 1
            # Test connection
            try:
                self.i2c_bus.read_byte(self.i2c_address)
                self.get_logger().info(f'Connected to motor driver at 0x{self.i2c_address:02X}')
            except:
                self.get_logger().error('Failed to connect to motor driver')
        """
        self.get_logger().warn('Hardware interface not implemented - using dummy data')

    def read_encoders(self):
        """
        Read encoder counts from motor driver via I2C

        Returns:
            tuple: (left_encoder_count, right_encoder_count)

        TODO: Implement actual I2C reading
        Motor driver register map (from PR2040_MotorDriver):
            REG_ENC0 = 0x10  # Encoder 0 (left front)
            REG_ENC1 = 0x11  # Encoder 1 (right front)
            REG_ENC2 = 0x12  # Encoder 2 (left rear)
            REG_ENC3 = 0x13  # Encoder 3 (right rear)

        Each register returns 4 bytes (int32, little-endian)

        Example implementation:
            import struct
            # Read left encoder (average of front and rear left)
            left_front = self.read_i2c_register(0x10, 4)
            left_rear = self.read_i2c_register(0x12, 4)
            left_enc = (struct.unpack('<i', left_front)[0] +
                       struct.unpack('<i', left_rear)[0]) // 2

            # Read right encoder (average of front and rear right)
            right_front = self.read_i2c_register(0x11, 4)
            right_rear = self.read_i2c_register(0x13, 4)
            right_enc = (struct.unpack('<i', right_front)[0] +
                        struct.unpack('<i', right_rear)[0]) // 2

            return (left_enc, right_enc)
        """
        # DUMMY DATA - Replace with actual encoder reading
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        # Simulate encoder increments (no motion)
        left_enc = self.prev_left_enc
        right_enc = self.prev_right_enc

        return (left_enc, right_enc)

    def read_i2c_register(self, register, num_bytes):
        """
        Read data from I2C register

        TODO: Implement actual I2C register read
        Example:
            self.i2c_bus.write_byte(self.i2c_address, register)
            data = self.i2c_bus.read_i2c_block_data(self.i2c_address, register, num_bytes)
            return bytes(data)
        """
        return bytes(num_bytes)

    def publish_odometry(self):
        """Calculate and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        if dt == 0:
            return

        # Read encoder counts
        left_enc, right_enc = self.read_encoders()

        # Calculate encoder deltas
        delta_left = left_enc - self.prev_left_enc
        delta_right = right_enc - self.prev_right_enc

        # Convert encoder counts to distance (meters)
        meters_per_count = (2.0 * math.pi * self.wheel_radius) / self.encoder_cpr
        left_distance = delta_left * meters_per_count
        right_distance = delta_right * meters_per_count

        # Differential drive kinematics
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # Update robot pose
        self.x += distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities
        linear_velocity = distance / dt
        angular_velocity = delta_theta / dt

        # Create quaternion from yaw
        q = self.quaternion_from_euler(0, 0, self.theta)

        # Publish TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Velocity
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        # Set covariance (placeholder values)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.01  # theta
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.01  # vtheta

        self.odom_pub.publish(odom)

        # Update previous values
        self.prev_left_enc = left_enc
        self.prev_right_enc = right_enc
        self.prev_time = current_time

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
    node = OdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
