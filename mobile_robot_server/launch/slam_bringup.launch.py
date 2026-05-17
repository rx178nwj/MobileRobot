#!/usr/bin/env python3
"""
SLAM Bringup — 3D LiDAR ICP SLAM (RTAB-Map)

前提: ws_edge_bringup.launch.py が起動済みで以下が利用可能であること
  - /odom (nav_msgs/Odometry, 50 Hz)
  - TF: odom → base_footprint → base_link

このlaunchが追加するもの:
  ① static TF: base_link → lidar_base_link (取付オフセット)
  ② lidar_pointcloud_bridge: USB CDC → /lidar/points (PointCloud2)
  ③ rtabmap: /lidar/points + /odom → /map + TF(map→odom)

Usage:
  ros2 launch mobile_robot_server slam_bringup.launch.py

  # LiDAR ポートを指定する場合
  ros2 launch mobile_robot_server slam_bringup.launch.py lidar_port:=/dev/ttyACM1

  # ロケーションモード (既存マップで自己位置推定のみ)
  ros2 launch mobile_robot_server slam_bringup.launch.py localization:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_server = get_package_share_directory('mobile_robot_server')
    rtabmap_cfg = os.path.join(pkg_server, 'config', 'rtabmap_lidar_3d_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyACM1',
        description='ESP32 LiDAR module USB CDC device path'
    )
    tilt_min = DeclareLaunchArgument(
        'tilt_min', default_value='-30.0',
        description='Scan tilt start angle [deg]'
    )
    tilt_max = DeclareLaunchArgument(
        'tilt_max', default_value='0.0',
        description='Scan tilt end angle [deg]'
    )
    tilt_step = DeclareLaunchArgument(
        'tilt_step', default_value='3.0',
        description='Scan tilt step [deg]'
    )
    # LiDAR mounting position on robot body (measure from actual hardware)
    lidar_x = DeclareLaunchArgument(
        'lidar_x', default_value='0.0',
        description='lidar_base_link X offset from base_link [m] — measure from hardware'
    )
    lidar_y = DeclareLaunchArgument(
        'lidar_y', default_value='0.0',
        description='lidar_base_link Y offset from base_link [m]'
    )
    lidar_z = DeclareLaunchArgument(
        'lidar_z', default_value='0.12',
        description='lidar_base_link Z offset from base_link [m] (approx 12cm above base)'
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )
    localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='true=localization only (requires existing map DB), false=SLAM'
    )
    database_path = DeclareLaunchArgument(
        'database_path', default_value='/maps/rtabmap.db',
        description='Path to RTAB-Map database file'
    )

    # ── ① Static TF: base_link → lidar_base_link ────────────────────────────
    # このフレームはチルト軸の取付点を表す。実機計測後に lidar_x/y/z を更新すること。
    static_tf_lidar_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_base',
        output='screen',
        arguments=[
            LaunchConfiguration('lidar_x'),
            LaunchConfiguration('lidar_y'),
            LaunchConfiguration('lidar_z'),
            '0', '0', '0',          # roll pitch yaw = 0
            'base_link',
            'lidar_base_link',
        ],
    )

    # ── ② lidar_pointcloud_bridge ─────────────────────────────────────────────
    lidar_bridge = Node(
        package='mobile_robot_server',
        executable='lidar_pointcloud_bridge',
        name='lidar_pointcloud_bridge',
        output='screen',
        parameters=[{
            'port':        LaunchConfiguration('lidar_port'),
            'tilt_min':    LaunchConfiguration('tilt_min'),
            'tilt_max':    LaunchConfiguration('tilt_max'),
            'tilt_step':   LaunchConfiguration('tilt_step'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    # ── ③ RTAB-Map (SLAM モード) ─────────────────────────────────────────────
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('localization')),
        parameters=[
            rtabmap_cfg,
            {
                'use_sim_time':    LaunchConfiguration('use_sim_time'),
                'database_path':   LaunchConfiguration('database_path'),
                'Mem/IncrementalMemory': 'true',
            },
        ],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('grid_map',   '/map'),
        ],
    )

    # ── ③' RTAB-Map (ロケーションモード) ────────────────────────────────────
    rtabmap_loc = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(LaunchConfiguration('localization')),
        parameters=[
            rtabmap_cfg,
            {
                'use_sim_time':    LaunchConfiguration('use_sim_time'),
                'database_path':   LaunchConfiguration('database_path'),
                'Mem/IncrementalMemory': 'false',  # 地図を更新しない
                'Mem/InitWMWithAllNodes': 'true',   # 全ノードをワーキングメモリに読み込む
            },
        ],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('grid_map',   '/map'),
        ],
    )

    return LaunchDescription([
        lidar_port,
        tilt_min,
        tilt_max,
        tilt_step,
        lidar_x,
        lidar_y,
        lidar_z,
        use_sim_time,
        localization,
        database_path,
        static_tf_lidar_base,
        lidar_bridge,
        rtabmap_slam,
        rtabmap_loc,
    ])
