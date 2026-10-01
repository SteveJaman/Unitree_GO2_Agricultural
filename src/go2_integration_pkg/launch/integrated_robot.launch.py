#!/usr/bin/env python3
"""
integrated_robot.launch.py

Single entry point that starts the integration layer. It does NOT launch the
WebRTC driver (go2_robot_sdk) or the autonomy stack directly, because they
each have their own launch files and their own overlays. Instead, this
launch assumes both are already running in separate terminals and only
starts the bridging nodes.

Typical usage:

  Terminal 1:  launch go2_robot_sdk      (WebRTC driver)
  Terminal 2:  launch autonomy_stack_go2 (SLAM + planner)
  Terminal 3:  ros2 launch go2_integration_pkg integrated_robot.launch.py

The relay node will then connect /point_cloud2 -> /utlidar/cloud.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ---------------- Launch arguments ----------------
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/point_cloud2',
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

    # ---------------- Startup banner ----------------
    banner = LogInfo(msg=(
        '\n'
        '================================================================\n'
        ' go2_integration_pkg: integrated_robot.launch.py\n'
        ' Relaying /point_cloud2 -> /utlidar/cloud (BEST_EFFORT)\n'
        ' Make sure the following are already running:\n'
        '   - go2_robot_sdk     (WebRTC driver)\n'
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
    ])