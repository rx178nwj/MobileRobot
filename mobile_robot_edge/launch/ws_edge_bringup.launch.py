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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


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
    pointcloud_interval_arg = DeclareLaunchArgument(
        'pointcloud_interval_s',
        default_value='6.0',
        description='3D pointcloud interval seconds'
    )
    scan_priority_cycles_arg = DeclareLaunchArgument(
        'scan_priority_cycles_before_3d',
        default_value='15',
        description='Number of 2D scan cycles before allowing 3D sweep'
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
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('mobile_robot_edge'),
            'urdf',
            'mobile_robot.urdf.xacro',
        ]),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
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
            'port':                    '/dev/lider_module',
            'scan_frame_id':           'laser_frame',
            'imu_frame_id':            'imu_link',
            'min_range_m':             0.02,
            'max_range_m':             12.0,
            'laser_scan_bins':         720,
            'slice_timeout_s':         6.0,
            'pointcloud_interval_s':   LaunchConfiguration('pointcloud_interval_s'),
            'sweep_tilt_min_deg':      -45.0,
            'sweep_tilt_max_deg':      45.0,
            'sweep_tilt_step_deg':     2.0,
            'scan_priority_cycles_before_3d': LaunchConfiguration('scan_priority_cycles_before_3d'),
            'min_quality':             0,
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([
        motor_ws_uri_arg,
        odom_ws_uri_arg,
        cmd_vel_topic_arg,
        pointcloud_interval_arg,
        scan_priority_cycles_arg,
        ws_motor_controller,
        ws_odometry_publisher,
        robot_state_publisher,
        lider_module_node,
    ])
