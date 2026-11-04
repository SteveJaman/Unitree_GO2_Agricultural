"""D1-550 on the Go2 in MoveIt 2: RViz + move_group + ros2_control.

Mock hardware (default, unchanged):
  ros2 launch d1_moveit_config demo.launch.py
  ros2 launch d1_moveit_config demo.launch.py use_rviz:=false
  then, in another terminal:  ros2 run d1_moveit_config inspect_plant.py

Gazebo Harmonic (gz_ros2_control), camera instead of the gripper:
  ros2 launch d1_moveit_config demo.launch.py hardware:=gazebo end_effector:=camera launch_gazebo:=true
    starts Gazebo with gazebo/worlds/hydroponic_station.sdf (world:=...),
    spawns the arm on its fixed dog stand at spawn_x/spawn_y, then
    robot_state_publisher, move_group and the controllers, all on sim time.
  ros2 launch d1_moveit_config demo.launch.py hardware:=gazebo end_effector:=camera
    Gazebo is already running with this arm in it (the walking Go2 of the
    simulation, SIM_MOVEIT=1: gazebo/sim/start_inside.sh starts this): only
    robot_state_publisher, move_group and the controller spawners.
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

REPO = os.environ.get('GO2_REPO', os.path.expanduser('~/files/Unitree_GO2_Agricultural'))


def moveit_config(hardware, end_effector):
    mappings = {'hardware': hardware, 'end_effector': end_effector,
                'use_mock_hardware': 'false' if hardware == 'gazebo' else 'true'}
    if hardware == 'real':
        controllers = 'config/moveit_controllers_real.yaml'
    elif end_effector == 'camera':
        controllers = 'config/moveit_controllers_camera.yaml'
    else:
        controllers = 'config/moveit_controllers.yaml'
    return (
        MoveItConfigsBuilder('d1_go2', package_name='d1_moveit_config')
        .robot_description(file_path='config/d1_go2.urdf.xacro', mappings=mappings)
        .robot_description_semantic(file_path='config/d1_go2.srdf', mappings={'end_effector': end_effector})
        .trajectory_execution(file_path=controllers)
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )


def gazebo_nodes(context):
    end_effector = LaunchConfiguration('end_effector').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    cfg = moveit_config('gazebo', end_effector)
    sim_time = {'use_sim_time': True}
    nodes = []

    if LaunchConfiguration('launch_gazebo').perform(context) == 'true':
        from ament_index_python.packages import get_package_share_directory
        world = LaunchConfiguration('world').perform(context)
        # the repo's worlds leave the render engine as a placeholder (start_inside.sh fills it)
        text = open(world).read().replace('@RENDER_ENGINE@', 'ogre2')
        out = os.path.join(tempfile.gettempdir(), 'd1_moveit_' + os.path.basename(world))
        open(out, 'w').write(text)
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                       'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {out}'}.items()))
        nodes.append(Node(package='ros_gz_sim', executable='create', output='screen',
                          arguments=['-topic', 'robot_description', '-name', 'd1_go2',
                                     '-x', LaunchConfiguration('spawn_x').perform(context),
                                     '-y', LaunchConfiguration('spawn_y').perform(context), '-z', '0.0']))
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                          arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))

    # gz_ros2_control reads the URDF (with its <ros2_control>) from this node
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                      name='robot_state_publisher', output='screen',
                      parameters=[cfg.robot_description, sim_time]))
    nodes.append(Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[cfg.to_dict(), sim_time]))
    for ctrl in ['joint_state_broadcaster', 'arm_controller'] + (['gripper_controller'] if end_effector == 'gripper' else []):
        nodes.append(Node(package='controller_manager', executable='spawner', output='screen',
                          arguments=[ctrl, '--controller-manager', '/controller_manager',
                                     '--controller-manager-timeout', '300']))
    if use_rviz:
        nodes.append(Node(package='rviz2', executable='rviz2', output='log',
                          arguments=['-d', str(cfg.package_path / 'config' / 'moveit.rviz')],
                          parameters=[cfg.robot_description, cfg.robot_description_semantic,
                                      cfg.planning_pipelines, cfg.robot_description_kinematics, sim_time]))
    return nodes


def mock_nodes(context):
    # Explicit nodes instead of generate_demo_launch(): that helper includes
    # launch/rsp.launch.py, move_group.launch.py, ... from this package, which
    # the MoveIt Setup Assistant would generate but this package does not have.
    end_effector = LaunchConfiguration('end_effector').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    cfg = moveit_config('mock', end_effector)
    controllers_yaml = str(cfg.package_path / 'config' / 'ros2_controllers.yaml')
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[cfg.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[cfg.robot_description, controllers_yaml]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[cfg.to_dict()]),
    ]
    for ctrl in ['joint_state_broadcaster', 'arm_controller'] + (['gripper_controller'] if end_effector == 'gripper' else []):
        nodes.append(Node(package='controller_manager', executable='spawner', output='screen',
                          arguments=[ctrl, '--controller-manager', '/controller_manager']))
    if use_rviz:
        nodes.append(Node(package='rviz2', executable='rviz2', output='log',
                          arguments=['-d', str(cfg.package_path / 'config' / 'moveit.rviz')],
                          parameters=[cfg.robot_description, cfg.robot_description_semantic,
                                      cfg.planning_pipelines, cfg.robot_description_kinematics]))
    return nodes


def real_nodes(context):
    # The real arm: no ros2_control. moveit_d1_bridge.py (started by arm_moveit.sh
    # --real) is the only arm_controller action server and /joint_states publisher.
    end_effector = LaunchConfiguration('end_effector').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    cfg = moveit_config('real', end_effector)
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[cfg.robot_description]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[cfg.to_dict()]),
    ]
    if use_rviz:
        nodes.append(Node(package='rviz2', executable='rviz2', output='log',
                          arguments=['-d', str(cfg.package_path / 'config' / 'moveit.rviz')],
                          parameters=[cfg.robot_description, cfg.robot_description_semantic,
                                      cfg.planning_pipelines, cfg.robot_description_kinematics]))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('hardware', default_value='mock', description='mock, gazebo or real'),
        DeclareLaunchArgument('end_effector', default_value='gripper', description='gripper or camera'),
        DeclareLaunchArgument('launch_gazebo', default_value='false',
                              description='hardware:=gazebo: also start Gazebo and spawn the arm'),
        DeclareLaunchArgument('world', default_value=os.path.join(REPO, 'gazebo', 'worlds', 'hydroponic_station.sdf')),
        DeclareLaunchArgument('spawn_x', default_value='1.892'),
        DeclareLaunchArgument('spawn_y', default_value='0.508'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    def pick(context):
        hardware = LaunchConfiguration('hardware').perform(context)
        if hardware == 'gazebo':
            return gazebo_nodes(context)
        return real_nodes(context) if hardware == 'real' else mock_nodes(context)

    return LaunchDescription(args + [OpaqueFunction(function=pick)])
