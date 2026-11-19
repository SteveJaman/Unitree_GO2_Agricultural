"""RViz with MoveIt's MotionPlanning panel for an already running move_group (no other nodes).

  ros2 launch d1_moveit_config rviz.launch.py hardware:=gazebo end_effector:=gripper claw_camera:=true

Same robot description and planning parameters as demo.launch.py, so the planning scene,
the arm's state and planned trajectories show as in demo.launch.py's own RViz. run_sim_demo.sh
--rviz starts it in the simulation container (ROS domain of the arm, sim time).
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_spec = importlib.util.spec_from_file_location(
    "d1_demo_launch", os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo.launch.py"))
_demo = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_demo)
moveit_config = _demo.moveit_config      # the same robot description and MoveIt parameters


def nodes(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    cfg = moveit_config(lc("hardware"), lc("end_effector"), lc("camera_model"), lc("claw_camera"))
    sim_time = {"use_sim_time": lc("hardware") == "gazebo"}
    return [Node(package="rviz2", executable="rviz2", output="log",
                 arguments=["-d", str(cfg.package_path / "config" / "moveit.rviz")],
                 parameters=[cfg.robot_description, cfg.robot_description_semantic, cfg.planning_pipelines,
                             cfg.robot_description_kinematics, sim_time])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="gazebo"),
        DeclareLaunchArgument("end_effector", default_value="gripper"),
        DeclareLaunchArgument("camera_model", default_value="d435i"),
        DeclareLaunchArgument("claw_camera", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
