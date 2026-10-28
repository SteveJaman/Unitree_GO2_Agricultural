"""VLM semantic costmap node.

  ros2 launch go2_vlm vlm_costmap.launch.py profile:=sim backend:=mock
  ros2 launch go2_vlm vlm_costmap.launch.py profile:=robot backend:=ollama model:=moondream

Needs the KeepoutFilter in go2-nav2-wifi's lidar3d_planner.yaml (filters:
[keepout_filter]); without it the mask is published but Nav2 ignores it.
The speed limit works without any Nav2 change.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = LaunchConfiguration
    here = get_package_share_directory('go2_vlm')
    params = PythonExpression(["'", os.path.join(here, 'config', 'vlm_'), "' + '",
                               cfg('profile'), "' + '.yaml'"])
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim', description='sim or robot'),
        DeclareLaunchArgument('backend', default_value='mock', description='mock or ollama'),
        DeclareLaunchArgument('model', default_value='moondream'),
        DeclareLaunchArgument('ollama_url', default_value='http://127.0.0.1:11434'),
        DeclareLaunchArgument('queries_file',
                              default_value=os.path.join(here, 'config', 'queries.yaml')),
        Node(package='go2_vlm', executable='semantic_costmap', name='vlm_semantic_costmap',
             output='screen',
             parameters=[params, {'backend': cfg('backend'), 'model': cfg('model'),
                                  'ollama_url': cfg('ollama_url'),
                                  'queries_file': cfg('queries_file')}]),
    ])
