"""D1-550 on the Go2 in MoveIt 2 with mock hardware: RViz + move_group + ros2_control.

  ros2 launch d1_moveit_config demo.launch.py
  ros2 launch d1_moveit_config demo.launch.py use_rviz:=false

Then, in another terminal:  ros2 run d1_moveit_config inspect_plant.py
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('d1_go2', package_name='d1_moveit_config')
        .robot_description(file_path='config/d1_go2.urdf.xacro')
        .robot_description_semantic(file_path='config/d1_go2.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
