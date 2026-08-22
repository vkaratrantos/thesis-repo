"""
Headless MoveIt stack for benchmarking.

Same move_group configuration as robot_config/launch/demo.launch.py, minus RViz,
plus a joint_state_publisher. demo.launch.py leaves /joint_states to a separate
bridge node, which is right for driving the real arm but means the planning
scene monitor never gets a current state when nothing else is running -- and the
benchmark needs a well-defined scene to plan against.

Nothing is executed here. move_group only ever plans.

    ros2 launch my_robot_control moveit_headless.launch.py
    ros2 run my_robot_control benchmark_reduced --ros-args \
        -p gen_dir:=<install>/share/my_robot_control/generated
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

MOVEIT_CONFIG_PKG = "robot_config"


def load_file(package, relative_path):
    path = os.path.join(get_package_share_directory(package), relative_path)
    with open(path, "r") as f:
        return f.read()


def load_yaml(package, relative_path):
    path = os.path.join(get_package_share_directory(package), relative_path)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("myarm_300_pi"), "urdf", "myarm_300_pi_thorgripper.urdf"
    )
    if not os.path.exists(urdf_path):
        urdf_path = "/home/vkaratrantos/elephant_robots_ws/src/myarm_300_pi/urdf/myarm_300_pi_thorgripper.urdf"

    robot_description = {"robot_description": xacro.process_file(urdf_path).toxml()}
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            MOVEIT_CONFIG_PKG, "config/myarm_300_pi_thorgripper.srdf"
        )
    }
    kinematics_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/kinematics.yaml")
    ompl_pipeline = {"move_group": load_yaml(MOVEIT_CONFIG_PKG, "config/ompl_planning.yaml")}

    # Copied verbatim from demo.launch.py. move_group segfaults on startup if
    # the controller manager is left unset -- trajectory_execution_manager
    # reloads controllers before it checks whether execution is even enabled --
    # so this has to be here even though the benchmark never executes anything.
    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["arm_controller", "gripper_controller"],
            "arm_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": ["joint1", "joint2", "joint3", "joint4",
                           "joint5", "joint6", "joint7"],
            },
            "gripper_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": ["endeffector_gripper"],
            },
        },
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            parameters=[robot_description],
        ),
        # Publishes a constant /joint_states at the default configuration. The
        # benchmark overrides every arm joint per query anyway; this only exists
        # so the planning scene monitor has a current state to build on.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="log",
            parameters=[robot_description, {"rate": 30}],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            # Reveals which OMPL state space is actually chosen. The selection
            # is logged at DEBUG only, and it is the difference between the
            # constrained state space (what ompl_planning.yaml asks for) and
            # rejection sampling (the slow fallback).
            arguments=["--ros-args", "--log-level",
                       "moveit.ompl_planning.planning_context_manager:=DEBUG"],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_pipeline,
                trajectory_execution,
                moveit_controllers,
                {"use_sim_time": False,
                 "publish_robot_description_semantic": True},
            ],
        ),
    ])
