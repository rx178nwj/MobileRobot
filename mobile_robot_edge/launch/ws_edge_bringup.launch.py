#!/usr/bin/env python3
"""
WebSocket Edge Bringup Launch File

This launch file starts all edge nodes that communicate with edge_services
via WebSocket:
1. ws_motor_controller - Receives /cmd_vel and sends to motor_service
2. ws_odometry_publisher - Receives odometry from odometry_service and publishes to ROS 2

Prerequisites:
- edge_services must be running on the host:
  cd /home/pi/MobileRobot/edge_services
  python3 launch_all.py

WebSocket Endpoints:
- Motor control: ws://localhost:8001
- Odometry: ws://localhost:8002
- Camera: ws://localhost:8003 (handled by camera.launch.py)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    motor_ws_uri_arg = DeclareLaunchArgument(
        'motor_ws_uri',
        default_value='ws://localhost:8001',
        description='WebSocket URI for motor service'
    )

    odom_ws_uri_arg = DeclareLaunchArgument(
        'odom_ws_uri',
        default_value='ws://localhost:8002',
        description='WebSocket URI for odometry service'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic name for velocity commands'
    )

    # WebSocket Motor Controller Node
    ws_motor_controller = Node(
        package='mobile_robot_edge',
        executable='ws_motor_controller',
        name='ws_motor_controller',
        output='screen',
        parameters=[{
            'ws_uri': LaunchConfiguration('motor_ws_uri'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    # WebSocket Odometry Publisher Node
    ws_odometry_publisher = Node(
        package='mobile_robot_edge',
        executable='ws_odometry_publisher',
        name='ws_odometry_publisher',
        output='screen',
        parameters=[{
            'ws_uri': LaunchConfiguration('odom_ws_uri'),
            'frame_id': 'odom',
            'child_frame_id': 'base_footprint',
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    # Robot State Publisher for TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': get_robot_description(),
        }],
    )

    # LiderModule Node (XIAO ESP32-C3 + T-mini Pro + STS3215 tilt servo)
    # Reads USB CDC binary protocol from /dev/ttyACM0
    # Publishes: /scan (LaserScan), /lider/pointcloud2, /lider/imu
    lider_module_node = Node(
        package='mobile_robot_edge',
        executable='lider_module_node',
        name='lider_module_node',
        output='screen',
        parameters=[{
            'port':                    '/dev/ttyACM0',
            'scan_frame_id':           'laser_frame',
            'imu_frame_id':            'imu_link',
            'min_range_m':             0.02,
            'max_range_m':             12.0,
            'laser_scan_bins':         720,
            'slice_timeout_s':         4.0,
            'pointcloud_interval_s':   30.0,
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([
        motor_ws_uri_arg,
        odom_ws_uri_arg,
        cmd_vel_topic_arg,
        ws_motor_controller,
        ws_odometry_publisher,
        robot_state_publisher,
        lider_module_node,
    ])


def get_robot_description():
    """
    Get robot URDF description

    TODO: Create proper URDF file
    For now, return minimal URDF with basic links
    """
    urdf = """<?xml version="1.0"?>
<robot name="mobile_robot">
  <!-- Base footprint (ground projection) -->
  <link name="base_footprint"/>

  <!-- Base link (robot center) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.16 0.1"/>
      </geometry>
      <origin xyz="0 0 0.05"/>
    </visual>
  </link>

  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.033"/>
  </joint>

  <!-- Camera link -->
  <link name="camera_link"/>

  <joint name="base_link_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- LiderModule laser frame -->
  <link name="laser_frame"/>

  <joint name="base_link_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.05 0 0.08" rpy="0 0 0"/>
  </joint>

  <!-- IMU frame (MPU6050 on LiderModule) -->
  <link name="imu_link"/>

  <joint name="laser_to_imu" type="fixed">
    <parent link="laser_frame"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""
    return urdf
