"""CMU Unity simulator + bridge to the go2-nav2-wifi topic/TF contract.

Starts only the simulator side of autonomy_stack_go2 (Unity binary,
ros_tcp_endpoint, vehicleSimulator, camera repub). CMU's own local planner,
terrain analysis and RViz are left out: navigation comes from go2-nav2-wifi.

  ros2 launch go2_sim_bridge cmu_unity_sim.launch.py
  ros2 launch go2_sim_bridge cmu_unity_sim.launch.py launch_unity:=false   # Unity already running
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CMU_STACK_DIR = os.environ.get('CMU_STACK_DIR', os.path.expanduser('~/files/autonomy_stack_go2'))
DEFAULT_UNITY = os.path.join(
    CMU_STACK_DIR, 'src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64')


def generate_launch_description():
    cfg = LaunchConfiguration
    args = [
        DeclareLaunchArgument('launch_unity', default_value='true'),
        DeclareLaunchArgument('unity_binary', default_value=DEFAULT_UNITY),
        DeclareLaunchArgument('vehicleX', default_value='0.0'),
        DeclareLaunchArgument('vehicleY', default_value='0.0'),
        DeclareLaunchArgument('vehicleYaw', default_value='0.0'),
        DeclareLaunchArgument('max_vx', default_value='0.5'),
        DeclareLaunchArgument('max_wz', default_value='1.0'),
    ]

    unity = ExecuteProcess(
        cmd=[cfg('unity_binary')], output='log', condition=IfCondition(cfg('launch_unity')))

    # ROS-TCP-Endpoint can crash with rclpy InvalidHandle when Unity
    # re-registers a topic it already registered (seen on first connect).
    # Unity reconnects on its own, so just bring the endpoint back.
    endpoint = Node(
        package='ros_tcp_endpoint', executable='default_server_endpoint', name='endpoint',
        output='log', respawn=True, respawn_delay=1.0,
        parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}])

    # Same parameters as autonomy_stack_go2 vehicle_simulator.launch, but its
    # TwistStamped /cmd_vel input is moved to /sim/cmd_vel so it does not
    # collide with Nav2's Twist /cmd_vel. cmu_bridge converts between them.
    vehicle_sim = Node(
        package='vehicle_simulator', executable='vehicleSimulator', name='vehicleSimulator',
        output='screen',
        remappings=[('/cmd_vel', '/sim/cmd_vel')],
        parameters=[{
            'sensorOffsetX': 0.0, 'sensorOffsetY': 0.0, 'vehicleHeight': 0.366,
            'vehicleX': cfg('vehicleX'), 'vehicleY': cfg('vehicleY'), 'vehicleZ': 0.0,
            'terrainZ': 0.0, 'vehicleYaw': cfg('vehicleYaw'),
            'terrainVoxelSize': 0.05, 'groundHeightThre': 0.1,
            'adjustZ': False, 'terrainRadiusZ': 1.0, 'minTerrainPointNumZ': 5,
            'smoothRateZ': 0.5, 'adjustIncl': False, 'terrainRadiusIncl': 2.0,
            'minTerrainPointNumIncl': 200, 'smoothRateIncl': 0.5,
            'InclFittingThre': 0.2, 'maxIncl': 30.0,
        }])

    # Unity camera -> raw images (/camera/image/raw), useful for VLM work later.
    image_repub = Node(
        package='vehicle_simulator', executable='sim_image_repub', name='sim_image_repub',
        output='log', parameters=[{
            'camera_in_topic': '/camera/image/compressed',
            'camera_raw_out_topic': '/camera/image/raw',
            'sem_in_topic': '/camera/semantic_image/compressed',
            'sem_raw_out_topic': '/camera/semantic_image/raw',
            'depth_in_topic': '/camera/depth/compressed',
            'depth_raw_out_topic': '/camera/depth/raw',
        }])

    bridge = Node(
        package='go2_sim_bridge', executable='cmu_bridge', name='go2_cmu_sim_bridge',
        output='screen', parameters=[{'max_vx': cfg('max_vx'), 'max_wz': cfg('max_wz')}])

    return LaunchDescription(args + [
        unity,
        # Give Unity a moment to come up before the endpoint and model-state
        # messages, as autonomy_stack_go2/system_simulation.sh does.
        TimerAction(period=3.0, actions=[endpoint, vehicle_sim, image_repub, bridge]),
    ])
