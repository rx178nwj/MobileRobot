#!/usr/bin/env python3
"""
Motor Controller Node

This node subscribes to /cmd_vel and controls the robot's motors via I2C.

Hardware Interface: PR2040 Motor Driver via I2C
- I2C Address: 0x60
- Control mode: Velocity control (VELOCITY mode)
- Converts /cmd_vel (linear, angular) to wheel velocities
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import struct


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters
        self.declare_parameter('wheel_base', 0.16)  # Distance between wheels (m)
        self.declare_parameter('wheel_radius', 0.033)  # Wheel radius (m)
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 2.0)  # rad/s
        self.declare_parameter('encoder_cpr', 720)  # Encoder counts per revolution
        self.declare_parameter('i2c_address', 0x60)  # Motor driver I2C address
        self.declare_parameter('cmd_vel_timeout', 0.5)  # Timeout for cmd_vel (seconds)

        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Safety timer - stop motors if no cmd_vel received
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        # Initialize hardware
        self.init_hardware()

        self.get_logger().info('Motor Controller initialized')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m, Max linear vel: {self.max_linear_vel}m/s')

    def init_hardware(self):
        """
        Initialize I2C connection and set motor control mode

        TODO: Implement actual I2C initialization
        Example:
            import smbus2
            self.i2c_bus = smbus2.SMBus(1)  # I2C bus 1

            # Test connection
            try:
                device_id = self.read_i2c_register(0xFF, 2)
                if device_id == bytes([0x20, 0x40]):
                    self.get_logger().info('Motor driver connected')
                else:
                    self.get_logger().error('Invalid device ID')
            except:
                self.get_logger().error('Failed to connect to motor driver')
                return

            # Set all motors to VELOCITY control mode (mode=1)
            REG_SET_MODE_ALL = 0x06
            mode_data = [REG_SET_MODE_ALL, 1, 1, 1, 1]  # All 4 motors to velocity mode
            self.i2c_bus.write_i2c_block_data(self.i2c_address, mode_data[0], mode_data[1:])
            self.get_logger().info('Motors set to velocity control mode')
        """
        self.get_logger().warn('Hardware interface not implemented - using dummy mode')

    def cmd_vel_callback(self, msg):
        """
        Process /cmd_vel command and send to motors

        Args:
            msg (Twist): Velocity command (linear.x, angular.z)
        """
        # Update last command time
        self.last_cmd_time = self.get_clock().now()

        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Clamp velocities to max values
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        # Convert to wheel velocities using differential drive kinematics
        # v_left = v - (omega * L) / 2
        # v_right = v + (omega * L) / 2
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert m/s to encoder counts/sec
        left_cps = self.velocity_to_cps(left_vel)
        right_cps = self.velocity_to_cps(right_vel)

        # Send velocity commands to motors
        self.set_wheel_velocities(left_cps, right_cps)

        # Debug logging
        self.get_logger().debug(
            f'cmd_vel: linear={linear_vel:.3f} angular={angular_vel:.3f} '
            f'-> left={left_cps:.1f}cps right={right_cps:.1f}cps'
        )

    def velocity_to_cps(self, velocity_mps):
        """
        Convert linear velocity (m/s) to encoder counts per second

        Args:
            velocity_mps (float): Velocity in m/s

        Returns:
            float: Encoder counts per second
        """
        # velocity = (2 * pi * r * cps) / cpr
        # cps = (velocity * cpr) / (2 * pi * r)
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        rps = velocity_mps / wheel_circumference  # Revolutions per second
        cps = rps * self.encoder_cpr  # Counts per second
        return cps

    def set_wheel_velocities(self, left_cps, right_cps):
        """
        Send velocity commands to motor driver via I2C

        Args:
            left_cps (float): Left wheel velocity (counts/sec)
            right_cps (float): Right wheel velocity (counts/sec)

        TODO: Implement actual I2C write
        Motor driver register map (from PR2040_MotorDriver):
            REG_SET_VEL_ALL = 0x08  # Set all 4 wheel velocities (16 bytes)
            Data format: float x4 (little-endian)

        Example implementation for 4-wheel robot (differential drive):
            import struct

            REG_SET_VEL_ALL = 0x08

            # For differential drive with 4 wheels:
            # Left side: wheels 0 and 2
            # Right side: wheels 1 and 3
            velocities = struct.pack('<ffff',
                                    left_cps,   # Wheel 0 (left front)
                                    right_cps,  # Wheel 1 (right front)
                                    left_cps,   # Wheel 2 (left rear)
                                    right_cps)  # Wheel 3 (right rear)

            # Write to I2C
            data = [REG_SET_VEL_ALL] + list(velocities)
            self.i2c_bus.write_i2c_block_data(self.i2c_address, data[0], data[1:])

        Alternative - Set individual wheels using REG_SET_VEL_SINGLE (0x09):
            # Format: [register, index, float velocity]
            left_data = struct.pack('<Bf', 0, left_cps)   # Wheel 0
            right_data = struct.pack('<Bf', 1, right_cps)  # Wheel 1
            # ... repeat for wheels 2 and 3
        """
        # DUMMY - Replace with actual I2C write
        pass

    def safety_check(self):
        """
        Safety timer callback - stops motors if no cmd_vel received within timeout
        """
        current_time = self.get_clock().now()
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9

        if time_since_cmd > self.cmd_vel_timeout:
            # No command received recently - stop motors for safety
            self.set_wheel_velocities(0.0, 0.0)

    def stop_motors(self):
        """Emergency stop - immediately stops all motors"""
        self.set_wheel_velocities(0.0, 0.0)
        self.get_logger().info('Motors stopped')

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


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
