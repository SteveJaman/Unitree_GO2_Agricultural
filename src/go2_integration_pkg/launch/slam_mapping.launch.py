#!/usr/bin/env python3
"""
slam_mapping.launch.py

Launches slam_toolbox in online async mode for building a 2D map.

Requirements:
  - /scan topic must be publishing (from pointcloud_to_scan.py)
  - TF tree must be complete (map -> odom -> base_link)

Usage:
  ros2 launch go2_integration_pkg slam_mapping.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='LaserScan topic for SLAM'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Map frame name'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Odometry frame name'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Robot base frame name'),
    ]

    banner = LogInfo(msg=(
        '\n'
        '================================================================\n'
        ' go2_integration_pkg: slam_mapping.launch.py\n'
        ' Starting slam_toolbox for online 2D mapping.\n'
        ' In RViz, add:\n'
        '   - Map         -> /map\n'
        '   - LaserScan   -> /scan\n'
        '   - TF          -> /tf, /tf_static\n'
        '================================================================\n'
    ))

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': LaunchConfiguration('odom_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 30.0,
            'minimum_time_interval': 0.2,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True,
        }],
    )

    return LaunchDescription(args + [banner, slam_toolbox])