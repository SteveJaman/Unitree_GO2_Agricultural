#!/usr/bin/env python3
"""
integrated_robot.launch.py

Starts the integration layer between go2_robot_sdk (WebRTC driver) and
autonomy_stack_go2. Assumes both are already running in separate terminals.

Terminal 1:  ros2 launch go2_robot_sdk robot.launch.py
Terminal 2:  ros2 launch vehicle_simulator system_real_robot.launch
Terminal 3:  ros2 launch go2_integration_pkg integrated_robot.launch.py

The relay nodes connect:
  /robot0/point_cloud2 -> /utlidar/cloud   (PointCloud2, BEST_EFFORT)
  /cmd_vel_stamped     -> /cmd_vel         (TwistStamped -> Twist)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ---------------- Launch arguments ----------------
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/robot0/point_cloud2',
        description='PointCloud2 topic published by go2_robot_sdk',
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/utlidar/cloud',
        description='PointCloud2 topic expected by point_lio_unilidar',
    )
    queue_arg = DeclareLaunchArgument(
        'queue_depth',
        default_value='5',
        description='QoS history depth for the relay',
    )

    # ---------------- Cloud relay node ----------------
    cloud_relay = Node(
        package='go2_integration_pkg',
        executable='cloud_relay_node.py',
        name='cloud_relay_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_topic':  LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'queue_depth':  LaunchConfiguration('queue_depth'),
        }],
    )

    # ---------------- Twist bridge node ----------------
    cmd_vel_bridge = Node(
        package='go2_integration_pkg',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_topic':  '/cmd_vel_stamped',
            'output_topic': '/cmd_vel',
        }],
    )

    # ---------------- Startup banner ----------------
    banner = LogInfo(msg=(
        '\n'
        '================================================================\n'
        ' go2_integration_pkg: integrated_robot.launch.py\n'
        ' Relays:\n'
        '   /robot0/point_cloud2 -> /utlidar/cloud   (BEST_EFFORT)\n'
        '   /cmd_vel_stamped     -> /cmd_vel         (TwistStamped -> Twist)\n'
        '\n'
        ' Assumes already running:\n'
        '   - go2_robot_sdk      (WebRTC driver)\n'
        '   - point_lio_unilidar (SLAM)\n'
        '   - local_planner / terrain_analysis\n'
        '================================================================\n'
    ))

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        queue_arg,
        banner,
        cloud_relay,
        cmd_vel_bridge,
    ])
