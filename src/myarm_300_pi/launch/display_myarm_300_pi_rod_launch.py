from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("myarm_300_pi").find("myarm_300_pi")

    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "myarm_300_pi_rod.urdf"])
    rviz_config_file = PathJoinSubstitution([pkg_share, "config", "display_myarm_300_pi_rod.rviz"])

    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_file])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])

