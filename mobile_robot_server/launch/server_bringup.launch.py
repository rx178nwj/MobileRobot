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

  ⓪ ydlidar_ros2_driver  (YDLIDAR T-mini Plus)
        /dev/ttyUSB0  →  /scan (LaserScan, 360°, 10 Hz)

  ① image_transport/republish
        /camera/image_raw/compressed  →  /camera/image_raw

  ② slam_toolbox  (2D LiDAR SLAM)
        /scan (YDLIDAR T-mini Plus) + /odom
        →  /map (OccupancyGrid, 5cm)
        →  TF: map → odom

  ③ Nav2 (navigation_launch.py)
        /map + /odom + /goal_pose  →  /cmd_vel

  ④ llm_nav_controller  (YOLO + Ollama)
        /camera/image_raw  →  YOLO (RTX 4060)  →  scene text
        scene text + /llm_command  →  Ollama qwen3.5:9b  →  /cmd_vel or /goal_pose

  /cmd_vel  →  (rmw_zenoh_cpp)  →  Raspberry Pi  →  motors

AI Pipeline (this host):
  Edge camera frame
    └→ YOLO (yolov8s.pt, RTX 4060 GPU)
         └→ object list + positions (text)
              └→ Ollama qwen3.5:9b (localhost:11434)
                   └→ navigation action (JSON)

Usage:
  # Start Ollama (if not running)
  ollama serve &
  ollama run qwen3.5:9b  # preload model

  # Build
  cd ~/ros2_ws && colcon build --packages-select mobile_robot_server
  source install/setup.bash

  # Launch
  ros2 launch mobile_robot_server server_bringup.launch.py

  # Override LLM settings
  ros2 launch mobile_robot_server server_bringup.launch.py \\
    llm_base_url:=http://localhost:11434/v1 llm_model:=qwen3.5:9b

  # Send LLM command (another terminal)
  ros2 topic pub /llm_command std_msgs/msg/String \\
    '{data: "{\"mode\": \"explore\"}"}' --once

  ros2 topic pub /llm_command std_msgs/msg/String \\
    '{data: "{\"mode\": \"navigate\", \"command\": \"go to the door\"}"}' --once
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_server  = get_package_share_directory('mobile_robot_server')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')

    slam_cfg    = os.path.join(pkg_server,  'config', 'slam_toolbox_params.yaml')
    nav2_cfg    = os.path.join(pkg_server,  'config', 'nav2_params.yaml')
    lidar_cfg   = os.path.join(pkg_ydlidar, 'params', 'Tmini.yaml')

    # ---- Launch arguments ----
    use_sim_time    = DeclareLaunchArgument('use_sim_time',    default_value='false')
    llm_interval    = DeclareLaunchArgument('llm_interval',    default_value='2.0')
    linear_speed    = DeclareLaunchArgument('linear_speed',    default_value='0.25')
    angular_speed   = DeclareLaunchArgument('angular_speed',   default_value='0.5')
    delete_db       = DeclareLaunchArgument(
        'delete_db_on_start', default_value='true',
        description='(unused) kept for backward compat')

    lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='YDLIDAR T-mini Plus serial port')

    llm_base_url = DeclareLaunchArgument(
        'llm_base_url',
        default_value=os.environ.get('LLM_BASE_URL', 'http://localhost:11434/v1'),
        description='Ollama / OpenAI-compatible API base URL (default: local Ollama)')

    llm_model = DeclareLaunchArgument(
        'llm_model',
        default_value=os.environ.get('LLM_MODEL', 'qwen3.5:9b-nav'),
        description='Ollama model name (default: qwen3.5:9b-nav, thinking disabled)')

    yolo_model = DeclareLaunchArgument(
        'yolo_model',
        default_value=os.environ.get('YOLO_MODEL', 'yolov8s.pt'),
        description='YOLO model file (yolov8n.pt / yolov8s.pt / yolov8m.pt)')

    # ------------------------------------------------------------------
    # Edge bridge nodes  (connect to edge_services via WebSocket)
    # ------------------------------------------------------------------
    ws_motor = Node(
        package='mobile_robot_edge',
        executable='ws_motor_controller',
        name='ws_motor_controller',
        output='screen',
        parameters=[{'ws_uri': 'ws://localhost:8001'}],
        respawn=True, respawn_delay=5.0,
    )

    ws_odom = Node(
        package='mobile_robot_edge',
        executable='ws_odometry_publisher',
        name='ws_odometry_publisher',
        output='screen',
        parameters=[{
            'ws_uri': 'ws://localhost:8002',
            'frame_id': 'odom',
            'child_frame_id': 'base_footprint',
        }],
        respawn=True, respawn_delay=5.0,
    )

    ws_camera = Node(
        package='mobile_robot_edge',
        executable='ws_camera_bridge',
        name='ws_camera_bridge',
        output='screen',
        parameters=[{
            'ws_uri': 'ws://localhost:8003',
            'frame_id': 'camera_optical_frame',
            'width': 640,
            'height': 480,
        }],
        respawn=True, respawn_delay=5.0,
    )

    robot_urdf = (
        '<?xml version="1.0"?>'
        '<robot name="mobile_robot">'
        '<link name="base_footprint"/>'
        '<link name="base_link"/>'
        '<joint name="base_footprint_joint" type="fixed">'
        '<parent link="base_footprint"/><child link="base_link"/>'
        '<origin xyz="0 0 0.033"/>'
        '</joint>'
        '<link name="laser_frame"/>'
        '<joint name="laser_joint" type="fixed">'
        '<parent link="base_link"/><child link="laser_frame"/>'
        '<origin xyz="0.05 0 0.08"/>'
        '</joint>'
        '<link name="camera_optical_frame"/>'
        '<joint name="camera_joint" type="fixed">'
        '<parent link="base_link"/><child link="camera_optical_frame"/>'
        '<origin xyz="0.10 0 0.10" rpy="-1.5708 0 -1.5708"/>'
        '</joint>'
        '</robot>'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ------------------------------------------------------------------
    # ⓪ YDLIDAR T-mini Plus
    #    /scan (sensor_msgs/LaserScan) @ 10 Hz, 360°, 0.03–12 m
    # ------------------------------------------------------------------
    ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[
            lidar_cfg,
            {'port': LaunchConfiguration('lidar_port')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

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
        remappings=[
            ('in/compressed', '/camera/image_raw/compressed'),
            ('out',           '/camera/image_raw'),
        ],
        parameters=[{
            'in_transport':  'compressed',
            'out_transport': 'raw',
            'use_sim_time':  LaunchConfiguration('use_sim_time'),
        }],
    )

    # ------------------------------------------------------------------
    # ② slam_toolbox — 2D LiDAR SLAM
    #
    #    /scan (YDLIDAR T-mini Plus, BEST_EFFORT) + /odom
    #    Publishes:
    #    - /map               →  Nav2 global costmap (OccupancyGrid 5cm)
    #    - TF map → odom      →  self-localization
    # ------------------------------------------------------------------
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_cfg,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('/scan', '/scan')],
    )

    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,  # disable bond — slam_toolbox heartbeat can lag under TF load
        }],
    )

    # ------------------------------------------------------------------
    # ③ Nav2 — Path planning & navigation (no docking_server)
    #    Inline node definitions to exclude docking_server, which crashes
    #    in Jazzy when dock_plugins is empty.
    # ------------------------------------------------------------------
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_cfg,
        root_key='',
        param_rewrites={},
        convert_types=True,
    )
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    autostart = True

    nav2 = GroupAction([
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': autostart,
                'bond_timeout': 30.0,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'route_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }],
        ),
    ])

    # ------------------------------------------------------------------
    # ④ LLM Navigation Controller
    #    Subscribes: /camera/image_raw, /llm_command
    #    Publishes:  /cmd_vel (direct), /goal_pose (→ Nav2)
    #    Pipeline: YOLO yolov8s.pt → scene text → Ollama qwen3.5:9b → JSON action
    # ------------------------------------------------------------------
    llm_nav = Node(
        package='mobile_robot_server',
        executable='llm_nav_controller',
        name='llm_nav_controller',
        output='screen',
        parameters=[{
            'openai_api_key':      os.environ.get('OPENAI_API_KEY', 'ollama'),
            'llm_base_url':        LaunchConfiguration('llm_base_url'),
            'model':               LaunchConfiguration('llm_model'),
            'llm_interval':        LaunchConfiguration('llm_interval'),
            'linear_speed':        LaunchConfiguration('linear_speed'),
            'angular_speed':       LaunchConfiguration('angular_speed'),
            'use_sim_time':        LaunchConfiguration('use_sim_time'),
            # YOLO + Ollama pipeline: disable direct image send, enable YOLO text pipeline
            'vision_enabled':      False,
            'use_object_detection': True,
            'yolo_model':          LaunchConfiguration('yolo_model'),
            'yolo_conf':           0.3,
        }],
    )

    # ------------------------------------------------------------------
    # ⑤ Autonomous Mapping
    #    Subscribes: /map, /odom, /mapping/stop
    #                /camera/image_raw/compressed (optional)
    #    Action client: navigate_to_pose
    #    Publishes: /mapping/status
    # ------------------------------------------------------------------
    autonomous_mapping = Node(
        package='mobile_robot_server',
        executable='autonomous_mapping',
        name='autonomous_mapping',
        output='screen',
        parameters=[{
            'llm_base_url':        LaunchConfiguration('llm_base_url'),
            'llm_model':           LaunchConfiguration('llm_model'),
            'openai_api_key':      os.environ.get('OPENAI_API_KEY', 'ollama'),
            'stuck_timeout':       60.0,
            'nav_timeout':         60.0,
            'min_frontier_size':   10,
            'use_sim_time':        LaunchConfiguration('use_sim_time'),
        }],
    )

    # ------------------------------------------------------------------
    # ⑥ Foxglove Bridge
    #    WebSocket :8765 → Foxglove Studio (browser)
    #    Advertises all active ROS2 topics
    # ------------------------------------------------------------------
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port':                8765,
            'address':             '0.0.0.0',
            'tls':                 False,
            'topic_whitelist':     ['.*'],
            'use_sim_time':        LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time,
        llm_interval,
        linear_speed,
        angular_speed,
        delete_db,
        lidar_port,
        llm_base_url,
        llm_model,
        yolo_model,
        robot_state_pub,
        ws_motor,
        ws_odom,
        ws_camera,
        ydlidar,
        image_decompress,
        slam,
        lifecycle_manager_slam,
        nav2,
        llm_nav,
        autonomous_mapping,
        foxglove_bridge,
    ])
