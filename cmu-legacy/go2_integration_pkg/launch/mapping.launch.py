#!/usr/bin/env python3
"""
mapping.launch.py

Launches the mapping node with optional RGB fusion.

The RGB fusion pipeline:
  - Subscribes to /camera/image/raw and /camera/camera_info
  - Looks up TF from map -> camera_color_optical_frame
  - Projects LiDAR points into the image
  - Attaches RGB to the accumulated cloud

Note: use_camera=true requires the camera and TF tree to be publishing.
In Unity simulation, enable it. On the real robot over Ethernet, the
front camera stream may not be available unless the WebRTC SDK is
running or a RealSense is attached.
"""

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
            'image_topic', default_value='/camera/image/raw',
            description='RGB camera image'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info',
            description='Camera intrinsics'),
        DeclareLaunchArgument(
            'output_dir', default_value=default_out,
            description='Where to save maps'),
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Enable RGB fusion'),
        DeclareLaunchArgument(
            'save_interval_s', default_value='0.0',
            description='Auto-save interval (0 disables)'),
    ]

    banner = LogInfo(msg=(
        '\n'
        '================================================================\n'
        ' go2_integration_pkg: mapping.launch.py\n'
        ' Consuming /registered_scan + /state_estimation from Point-LIO.\n'
        ' In RViz, add:\n'
        '   - Map         -> /map/occupancy\n'
        '   - PointCloud2 -> /map/points\n'
        ' On Ctrl+C: saves map.png + map.ply + mesh.obj\n'
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
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_dir': LaunchConfiguration('output_dir'),
            'use_camera': LaunchConfiguration('use_camera'),
            'save_interval_s': LaunchConfiguration('save_interval_s'),
        }],
    )

    return LaunchDescription(args + [banner, node])