"""Frontier exploration in the CMU Unity sim, using the real go2-nav2-wifi Nav2.

  CMU Unity sim ──cmu_bridge──► go2_nav2 lidar3d_planning  (planner + global costmap)
                                go2_nav2 lidar3d_controller (RPP + smoother + goal_pose_nav)
                                frontier_explorer.py        (scripts/nodes, same params as wifi_nav.sh)

  ros2 launch go2_sim_bridge sim_explore.launch.py
  ros2 launch go2_sim_bridge sim_explore.launch.py autostart:=true rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = LaunchConfiguration
    here = get_package_share_directory('go2_sim_bridge')
    go2_nav2 = get_package_share_directory('go2_nav2')

    def include(path, **launch_args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=launch_args.items())

    explorer = Node(
        package='go2_explore_panel', executable='frontier_explorer.py', name='frontier_explorer',
        output='screen', parameters=[os.path.join(here, 'config', 'explorer_sim.yaml')])

    # Same as clicking "Auto Explore: START" in the RViz panel.
    start_explore = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/explore/resume', 'std_msgs/msg/Bool',
             '{data: true}'],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(here, 'config', 'sim_explore.rviz')],
        condition=IfCondition(cfg('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('launch_unity', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='enable exploration without the RViz button'),
        DeclareLaunchArgument('autostart_delay', default_value='20.0'),

        # lidar3d_planning.launch.py reads this at launch time to switch to the
        # navigation profile (footprint, Smac lattice) that mapping.sh --3d --nav uses.
        SetEnvironmentVariable('GO2_LIDAR3D_NAV', '1'),

        include(os.path.join(here, 'launch', 'cmu_unity_sim.launch.py'),
                launch_unity=cfg('launch_unity')),
        include(os.path.join(go2_nav2, 'launch', 'lidar3d_planning.launch.py')),
        include(os.path.join(go2_nav2, 'launch', 'lidar3d_controller.launch.py')),
        rviz,
        # Let Nav2 reach active and the costmap fill before the explorer looks.
        TimerAction(period=8.0, actions=[explorer]),
        TimerAction(period=cfg('autostart_delay'), actions=[start_explore],
                    condition=IfCondition(cfg('autostart'))),
    ])
