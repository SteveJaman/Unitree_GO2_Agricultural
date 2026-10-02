#!/usr/bin/env python3
"""
mapping.launch.py

Launches the industry-standard mapping node on top of Point-LIO.

Run AFTER the autonomy stack is up (system_real_robot_ethernet.sh or
system_simulation.sh), because Point-LIO must be publishing
/cloud_registered and /state_estimation.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_out = str(Path.home() / 'go2_maps')

    args = [
        DeclareLaunchArgument(
            'cloud_topic', default_value='/registered_scan',
            description='Aligned point cloud from Point-LIO'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/state_estimation',
            description='6-DOF pose from Point-LIO'),
        DeclareLaunchArgument(
            'output_dir', default_value=default_out,
            description='Where to save maps'),
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Fuse RGB from /camera/image/raw'),
        DeclareLaunchArgument(
            'save_interval_s', default_value='0.0',
            description='Auto-save every N seconds (0 disables)'),
    ]

    banner = LogInfo(msg=(
        '\n'
        '================================================================\n'
        ' go2_integration_pkg: mapping.launch.py\n'
        ' Consuming /cloud_registered + /state_estimation from Point-LIO.\n'
        ' In RViz, add:\n'
        '   - Map         -> /map/occupancy\n'
        '   - PointCloud2 -> /map/points\n'
        ' On Ctrl+C, writes map.png + map.ply + mesh.obj to ~/go2_maps/\n'
        '================================================================\n'
    ))

    node = Node(
        package='go2_integration_pkg',
        executable='map_node.py',
        name='map_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'output_dir': LaunchConfiguration('output_dir'),
            'use_camera': LaunchConfiguration('use_camera'),
            'save_interval_s': LaunchConfiguration('save_interval_s'),
        }],
    )

    return LaunchDescription(args + [banner, node])