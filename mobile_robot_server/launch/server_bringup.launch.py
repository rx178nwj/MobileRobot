#!/usr/bin/env python3
"""
Mobile Robot Server Bringup Launch File
Ubuntu Server Side — ROS 2 Jazzy

Data flow:
  [Raspberry Pi / Edge]  →  (rmw_zenoh_cpp / WiFi)  →  [This PC / Server]

  /camera/image_raw/compressed  ─┐
  /camera/camera_info            │  (from Pi)
  /odom                          │
  TF: odom → base_footprint      ┘

  ① image_transport/republish
        /camera/image_raw/compressed  →  /camera/image_raw

  ② rtabmap_slam/rtabmap  (Monocular RGB + Odometry mode)
        /camera/image_raw + /camera/camera_info + /odom
        →  /map (OccupancyGrid)
        →  TF: map → odom

  ③ Nav2 (navigation_launch.py)
        /map + /odom + /goal_pose  →  /cmd_vel

  ④ llm_nav_controller
        /camera/image_raw + /llm_command  →  /cmd_vel or /goal_pose

  /cmd_vel  →  (rmw_zenoh_cpp)  →  Raspberry Pi  →  motors

Usage:
  # Set API key (OpenAI GPT-4o)
  export OPENAI_API_KEY="sk-..."

  # Build
  cd ~/ros2_ws && colcon build --packages-select mobile_robot_server
  source install/setup.bash

  # Launch
  ros2 launch mobile_robot_server server_bringup.launch.py

  # Send LLM command (another terminal)
  ros2 topic pub /llm_command std_msgs/msg/String \\
    '{data: "{\"mode\": \"explore\"}"}' --once

  ros2 topic pub /llm_command std_msgs/msg/String \\
    '{data: "{\"mode\": \"navigate\", \"command\": \"go to the door\"}"}' --once
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_server = get_package_share_directory('mobile_robot_server')
    pkg_nav2   = get_package_share_directory('nav2_bringup')

    rtabmap_cfg = os.path.join(pkg_server, 'config', 'rtabmap_params.yaml')
    nav2_cfg    = os.path.join(pkg_server, 'config', 'nav2_params.yaml')

    # ---- Launch arguments ----
    use_sim_time    = DeclareLaunchArgument('use_sim_time',    default_value='false')
    llm_interval    = DeclareLaunchArgument('llm_interval',    default_value='2.0')
    linear_speed    = DeclareLaunchArgument('linear_speed',    default_value='0.25')
    angular_speed   = DeclareLaunchArgument('angular_speed',   default_value='0.5')
    delete_db       = DeclareLaunchArgument(
        'delete_db_on_start', default_value='true',
        description='Delete RTAB-Map DB on start (true=mapping, false=localization only)')

    # ------------------------------------------------------------------
    # ① Image decompression
    #    image_transport republish: compressed → raw
    #    Input:  /camera/image_raw/compressed
    #    Output: /camera/image_raw
    # ------------------------------------------------------------------
    image_decompress = Node(
        package='image_transport',
        executable='republish',
        name='image_decompress',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/image_raw/compressed'),
            ('out',           '/camera/image_raw'),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # ------------------------------------------------------------------
    # ② RTAB-Map — Monocular Visual SLAM
    #
    #    Mode: RGB + Odometry (no depth)
    #    - /camera/image_raw  →  visual feature extraction & loop closure
    #    - /odom              →  scale reference & motion prior
    #    Publishes:
    #    - /map               →  Nav2 global costmap
    #    - TF map → odom      →  self-localization
    # ------------------------------------------------------------------
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_cfg,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            # monocular mode: map rgb/* to camera topics
            ('rgb/image',       '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom',            '/odom'),
        ],
        arguments=['--delete_db_on_start'],  # always fresh map on launch
    )

    # ------------------------------------------------------------------
    # ③ Nav2 — Path planning & navigation
    #    navigation_launch.py starts:
    #      planner_server, controller_server, bt_navigator,
    #      behavior_server, velocity_smoother, lifecycle_manager
    #    Does NOT start map_server (map comes from RTAB-Map /map topic)
    # ------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_cfg,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items(),
    )

    # ------------------------------------------------------------------
    # ④ LLM Navigation Controller
    #    Subscribes: /camera/image_raw, /llm_command
    #    Publishes:  /cmd_vel (direct), /goal_pose (→ Nav2)
    # ------------------------------------------------------------------
    llm_nav = Node(
        package='mobile_robot_server',
        executable='llm_nav_controller',
        name='llm_nav_controller',
        output='screen',
        parameters=[{
            'openai_api_key': os.environ.get('OPENAI_API_KEY', ''),
            'model':          'gpt-4o',
            'llm_interval':   LaunchConfiguration('llm_interval'),
            'linear_speed':   LaunchConfiguration('linear_speed'),
            'angular_speed':  LaunchConfiguration('angular_speed'),
            'use_sim_time':   LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time,
        llm_interval,
        linear_speed,
        angular_speed,
        delete_db,
        image_decompress,
        rtabmap,
        nav2,
        llm_nav,
    ])
