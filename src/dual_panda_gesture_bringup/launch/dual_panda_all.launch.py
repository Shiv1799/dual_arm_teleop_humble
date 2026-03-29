#!/usr/bin/env python3
"""
Unified launch: Dual Panda + ros2_control + gesture tracker + bridge + RViz.
Gripper controllers are spawned by the bridge node itself via subprocess.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dual_config_pkg = get_package_share_directory(
        'dual_arm_panda_moveit_config')
    bringup_pkg = get_package_share_directory(
        'dual_panda_gesture_bringup')

    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0')
    velocity_gain_arg = DeclareLaunchArgument(
        'velocity_gain', default_value='1.5')
    smoothing_arg = DeclareLaunchArgument(
        'smoothing_factor', default_value='0.4')

    # 1. Demo launch (try to disable its RViz)
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_config_pkg, 'launch', 'demo.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items(),
    )

    # 2. Our RViz with RobotModel display
    rviz_config = os.path.join(
        bringup_pkg, 'config', 'gesture_control.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_gesture',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # 3. Gesture tracker + bridge (delayed 5s for controllers)
    workspace_bounds_file = os.path.join(
        bringup_pkg, 'config', 'dual_workspace_bounds.yaml')

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

    delayed_gesture = TimerAction(
        period=5.0,
        actions=[dual_tracker_node, dual_bridge_node],
    )

    return LaunchDescription([
        camera_id_arg,
        velocity_gain_arg,
        smoothing_arg,
        demo_launch,
        rviz_node,
        delayed_gesture,
    ])
