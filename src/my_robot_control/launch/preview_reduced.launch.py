"""
RViz preview of the reduced-planning pipeline.

Shows the real 7-DOF arm with the swept-volume blob attached at its wrist
centre, animating along paths produced by the reduced planner. The obstacles
are published as markers.

No move_group, no controllers, no robot -- preview_reduced publishes
/joint_states itself.

    ros2 launch my_robot_control preview_reduced.launch.py
    ros2 launch my_robot_control preview_reduced.launch.py transit_z:=0.30
    ros2 launch my_robot_control preview_reduced.launch.py group:=arm_positioning_3dof

What to look for: the gripper and the carried tube stay inside the ring at all
times. That containment is the assumption the whole approach rests on.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "my_robot_control"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    gen_dir = os.path.join(share, "generated")
    preview_urdf = os.path.join(gen_dir, "preview_arm.urdf")
    rviz_config = os.path.join(share, "config", "preview_reduced.rviz")

    if not os.path.exists(preview_urdf):
        raise RuntimeError(
            f"{preview_urdf} not found.\n"
            "Generate it first:\n"
            "  python3 tools/wrist_blob_gen.py --tag carry --tilt 0.30\n"
            "  python3 tools/make_reduced_model.py --geometry mesh\n"
            "then rebuild the package."
        )

    with open(preview_urdf, "r") as f:
        robot_description = {"robot_description": f.read()}

    return LaunchDescription([
        DeclareLaunchArgument("transit_z", default_value="0.24",
                              description="wrist-centre height for the transit legs"),
        DeclareLaunchArgument("group", default_value="arm_positioning",
                              description="arm_positioning (4-DOF) or arm_positioning_3dof"),
        DeclareLaunchArgument("rate", default_value="30.0"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            parameters=[robot_description],
        ),
        Node(
            package=PKG,
            executable="preview_reduced",
            output="screen",
            parameters=[{
                "gen_dir": gen_dir,
                "group": LaunchConfiguration("group"),
                "transit_z": LaunchConfiguration("transit_z"),
                "rate": LaunchConfiguration("rate"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", rviz_config],
        ),
    ])
