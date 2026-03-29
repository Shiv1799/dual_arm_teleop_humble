#!/usr/bin/env python3
"""
Launch file: dual gesture tracking (webcam + MediaPipe 2-hand)
and dual MoveIt Servo bridge.

Use this when you already have the dual_arm_panda_moveit_config demo
and two servo nodes running separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('dual_panda_gesture_bringup')
    workspace_bounds_file = os.path.join(
        bringup_pkg, 'config', 'dual_workspace_bounds.yaml')

    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='Webcam device index')
    smoothing_arg = DeclareLaunchArgument(
        'smoothing_factor', default_value='0.4',
        description='EMA smoothing for hand position')
    velocity_gain_arg = DeclareLaunchArgument(
        'velocity_gain', default_value='1.5',
        description='Proportional gain for velocity control')

    dual_tracker_node = Node(
        package='dual_panda_gesture_bringup',
        executable='dual_gesture_tracker',
        name='dual_gesture_tracker',
        output='screen',
        parameters=[
            workspace_bounds_file,
            {
                'camera_id': LaunchConfiguration('camera_id'),
                'smoothing_factor': LaunchConfiguration('smoothing_factor'),
            },
        ],
    )

    dual_bridge_node = Node(
        package='dual_panda_gesture_bringup',
        executable='dual_gesture_bridge',
        name='dual_gesture_bridge',
        output='screen',
        parameters=[
            workspace_bounds_file,
            {
                'velocity_gain': LaunchConfiguration('velocity_gain'),
            },
        ],
    )

    return LaunchDescription([
        camera_id_arg,
        smoothing_arg,
        velocity_gain_arg,
        dual_tracker_node,
        dual_bridge_node,
    ])
