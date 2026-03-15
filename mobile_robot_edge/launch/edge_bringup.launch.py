#!/usr/bin/env python3
"""
Edge Bringup Launch File

This launch file starts all edge-side nodes for the mobile robot:
1. Camera node (Raspberry Pi Camera Module 3 with fixed focus)
2. Odometry publisher (wheel encoders via I2C)
3. Motor controller (cmd_vel subscriber via I2C)

Usage:
    ros2 launch mobile_robot_edge edge_bringup.launch.py

Hardware:
    - Raspberry Pi Camera Module 3
    - PR2040 Motor Driver (I2C address 0x60)
    - 4 DC motors with encoders
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('mobile_robot_edge').find('mobile_robot_edge')

    # Camera autofocus disable script (run once at startup)
    disable_autofocus = ExecuteProcess(
        cmd=[
            'bash',
            PathJoinSubstitution([
                FindPackageShare('mobile_robot_edge'),
                'mobile_robot_edge',
                'disable_camera_autofocus.sh'
            ])
        ],
        name='disable_camera_autofocus',
        output='screen',
    )

    # Camera launch (includes v4l2_camera + image compression)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mobile_robot_edge'),
                'launch',
                'camera.launch.py'
            ])
        ])
    )

    # Odometry Publisher Node
    odometry_node = Node(
        package='mobile_robot_edge',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        parameters=[{
            'wheel_base': 0.16,      # meters
            'wheel_radius': 0.033,   # meters
            'encoder_cpr': 720,      # counts per revolution
            'publish_rate': 50.0,    # Hz
            'i2c_address': 0x60,     # Motor driver I2C address
        }],
        # Remap if needed
        remappings=[
            ('/odom', '/odom'),
        ]
    )

    # Motor Controller Node
    motor_controller_node = Node(
        package='mobile_robot_edge',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'wheel_base': 0.16,              # meters
            'wheel_radius': 0.033,           # meters
            'max_linear_velocity': 0.5,      # m/s
            'max_angular_velocity': 2.0,     # rad/s
            'encoder_cpr': 720,              # counts per revolution
            'i2c_address': 0x60,             # Motor driver I2C address
            'cmd_vel_timeout': 0.5,          # seconds
        }],
        # Remap if needed (e.g., to receive from navigation stack)
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    # Robot State Publisher (publishes static transforms)
    # This describes the robot's physical structure
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': get_robot_description(),
            'publish_frequency': 30.0,
        }]
    )

    return LaunchDescription([
        # First, disable camera autofocus
        disable_autofocus,

        # Wait a bit for camera settings to apply, then start camera
        TimerAction(
            period=1.0,
            actions=[camera_launch]
        ),

        # Start robot state publisher
        robot_state_publisher_node,

        # Start odometry publisher
        odometry_node,

        # Start motor controller
        motor_controller_node,
    ])


def get_robot_description():
    """
    Generate URDF robot description

    This is a minimal URDF that defines the robot's coordinate frames.
    For a complete robot description, create a separate URDF/xacro file.

    Frame tree:
        base_footprint (on ground, between wheels)
        └── base_link (robot center)
            ├── camera_link
            │   └── camera_optical_frame
            └── [wheel links, sensor links, etc.]
    """
    urdf = '''<?xml version="1.0"?>
<robot name="mobile_robot">

  <!-- Base footprint (virtual frame on ground) -->
  <link name="base_footprint"/>

  <!-- Base link (robot center) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.15 0.12 0.08"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.04" rpy="0 0 0"/>
  </joint>

  <!-- Camera link -->
  <link name="camera_link"/>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <!-- Mount camera at front, 5cm high, tilted down 10 degrees -->
    <origin xyz="0.075 0 0.01" rpy="0 0.175 0"/>
  </joint>

  <!-- Camera optical frame (standard ROS camera frame) -->
  <link name="camera_optical_frame"/>

  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <!-- Rotate to optical frame convention: x=right, y=down, z=forward -->
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

</robot>
'''
    return urdf
