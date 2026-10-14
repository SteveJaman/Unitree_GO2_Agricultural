#!/usr/bin/env python3
"""
localization.launch.py

Launches AMCL for localisation against a pre-built map, plus the
map_server to load the saved map.

Requirements:
  - /scan topic must be publishing
  - TF tree must be complete (map -> odom -> base_link)
  - A saved map (YAML + PGM) must exist

Usage:
  ros2 launch go2_integration_pkg localization.launch.py \
      map_file:=$HOME/go2_maps/my_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'map_file', default_value=os.path.expanduser('~/go2_maps/map.yaml'),
            description='Full path to the saved map YAML'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='LaserScan topic'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
    ]

    banner = LogInfo(msg=(
        '\n'
        '================================================================\n'
        ' go2_integration_pkg: localization.launch.py\n'
        ' Starting map_server + AMCL.\n'
        ' Set the initial pose in RViz:\n'
        '   - Click "2D Pose Estimate"\n'
        '   - Click and drag on the map where the robot is\n'
        '================================================================\n'
    ))

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map_file'),
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_link',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 30.0,
            'laser_min_range': 0.2,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription(args + [banner, map_server, amcl, lifecycle_manager])