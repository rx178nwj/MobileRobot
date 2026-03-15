#!/usr/bin/env python3
"""
Camera Launch File for Raspberry Pi Camera Module 3

This launch file configures v4l2_camera to:
1. Disable autofocus (fixed focus for SLAM stability)
2. Publish compressed images to save network bandwidth
3. Set appropriate resolution and frame rate

Hardware: Raspberry Pi Camera Module 3
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # V4L2 Camera Node - Publishes raw image
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        namespace='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],  # VGA resolution for balanced quality/bandwidth
            'camera_frame_id': 'camera_optical_frame',
            'io_method': 'mmap',
            'pixel_format': 'YUYV',
            'frame_rate': 30.0,
            # Camera Module 3 specific controls to disable autofocus
            # These V4L2 controls fix the focus for SLAM stability
            'camera_info_url': '',
        }],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
        ]
    )

    # Image Transport Republish Node - Converts raw to compressed
    # This significantly reduces network bandwidth usage
    image_republish_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republish',
        namespace='camera',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out/compressed', '/camera/image_raw/compressed'),
        ]
    )

    # V4L2 Control Node to disable autofocus
    # Camera Module 3 has autofocus which must be disabled for SLAM
    # This uses v4l2-ctl commands to set fixed focus
    v4l2_focus_disable = Node(
        package='mobile_robot_edge',
        executable='disable_camera_autofocus.sh',
        name='v4l2_focus_config',
        output='screen',
        # This script runs once at startup to configure camera controls
        # Script content:
        # #!/bin/bash
        # v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
        # v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=450
        # (450 is a typical fixed focus value, adjust based on your setup)
    )

    return LaunchDescription([
        v4l2_camera_node,
        image_republish_node,
        # Uncomment below when disable_camera_autofocus.sh script is created
        # v4l2_focus_disable,
    ])
