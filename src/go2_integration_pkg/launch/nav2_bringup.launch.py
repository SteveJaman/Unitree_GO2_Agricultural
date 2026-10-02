#!/usr/bin/env python3
"""
nav2_bringup.launch.py

Launches the Nav2 navigation stack for autonomous path planning and
obstacle avoidance.

Requirements:
  - /scan topic must be publishing
  - TF tree must be complete (map -> odom -> base_link)
  - A map must be loaded (from localization.launch.py)
  - /cmd_vel must be consumable by the robot

Usage:
  ros2 launch go2_integration_pkg nav2_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure and activate Nav2 nodes'),
    ]

    banner = LogInfo(msg=(
        '\n'
        '================================================================\n'
        ' go2_integration_pkg: nav2_bringup.launch.py\n'
        ' Starting Nav2 navigation stack.\n'
        ' In RViz, use "2D Goal Pose" to send the robot to a target.\n'
        '================================================================\n'
    ))

    # Common Nav2 parameters
    nav2_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'yaml_filename': '',
    }

    # --- Controller Server ---
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'controller_frequency': 20.0,
            'min_x_velocity_threshold': 0.001,
            'min_y_velocity_threshold': 0.5,
            'min_theta_velocity_threshold': 0.001,
            'failure_tolerance': 0.3,
            'progress_checker_plugin': 'progress_checker',
            'goal_checker_plugins': ['general_goal_checker'],
            'controller_plugins': ['FollowPath'],
            'progress_checker': {
                'plugin': 'nav2_controller::SimpleProgressChecker',
                'required_movement_radius': 0.5,
                'movement_time_allowance': 10.0,
            },
            'general_goal_checker': {
                'stateful': True,
                'plugin': 'nav2_controller::SimpleGoalChecker',
                'xy_goal_tolerance': 0.25,
                'yaw_goal_tolerance': 0.25,
            },
            'FollowPath': {
                'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                'desired_linear_vel': 0.3,
                'lookahead_dist': 0.6,
                'min_lookahead_dist': 0.3,
                'max_lookahead_dist': 0.9,
                'transform_tolerance': 0.2,
                'use_velocity_scaled_lookahead_dist': False,
                'min_approach_linear_velocity': 0.05,
                'approach_velocity_scaling_dist': 0.6,
                'use_collision_detection': True,
                'max_allowed_time_to_collision_up_to_carrot': 1.0,
                'use_regulated_linear_velocity_scaling': True,
                'use_cost_regulated_linear_velocity_scaling': False,
                'cost_scaling_dist': 0.6,
                'cost_scaling_gain': 1.0,
                'inflation_cost_scaling_factor': 3.0,
                'regulated_linear_scaling_min_radius': 0.9,
                'regulated_linear_scaling_min_speed': 0.25,
                'use_rotate_to_heading': True,
                'allow_reversing': False,
                'rotate_to_heading_min_angle': 0.785,
                'max_angular_accel': 3.2,
                'max_robot_pose_search_dist': 10.0,
            },
        }],
    )

    # --- Planner Server ---
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'expected_planner_frequency': 1.0,
            'planner_plugins': ['GridBased'],
            'GridBased': {
                'plugin': 'nav2_navfn_planner::NavfnPlanner',
                'tolerance': 0.5,
                'use_astar': True,
                'allow_unknown': True,
            },
        }],
    )

    # --- Behavior Server ---
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'costmap_topic': 'local_costmap/costmap_raw',
            'footprint_topic': 'local_costmap/published_footprint',
            'cycle_frequency': 10.0,
            'behavior_plugins': ['spin', 'backup', 'drive_on_heading', 'wait'],
            'spin': {
                'plugin': 'nav2_behaviors::Spin',
            },
            'backup': {
                'plugin': 'nav2_behaviors::BackUp',
            },
            'drive_on_heading': {
                'plugin': 'nav2_behaviors::DriveOnHeading',
            },
            'wait': {
                'plugin': 'nav2_behaviors::Wait',
            },
            'local_frame': 'odom',
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'transform_tolerance': 0.1,
            'simulate_ahead_time': 2.0,
            'max_rotational_vel': 1.0,
            'min_rotational_vel': 0.4,
            'rotational_acc_lim': 3.2,
        }],
    )

    # --- BT Navigator ---
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'odom_topic': '/odom',
            'bt_loop_duration': 10,
            'default_server_timeout': 20,
            'navigators': ['navigate_to_pose', 'navigate_through_poses'],
            'navigate_to_pose': {
                'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator',
            },
            'navigate_through_poses': {
                'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator',
            },
        }],
    )

    # --- Recovery Server ---
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_frame': 'odom',
            'robot_base_frame': 'base_link',
            'transform_tolerance': 0.1,
            'simulate_ahead_time': 2.0,
            'max_rotational_vel': 1.0,
            'min_rotational_vel': 0.4,
            'rotational_acc_lim': 3.2,
        }],
    )

    # --- Lifecycle Manager ---
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'recoveries_server',
            ],
        }],
    )

    return LaunchDescription(args + [
        banner,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        recoveries_server,
        lifecycle_manager,
    ])