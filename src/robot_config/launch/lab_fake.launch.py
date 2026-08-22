# =============================================================================
#  ONE-SHOT LAUNCHER -- fake-robot mode
#
#  Replaces the manual sequence:  fake -> start_fake -> move -> GUI
#
#  This file deliberately INCLUDES fake.launch.py and run_simple_move.launch.py
#  instead of copying their contents, so any fix to those two files applies
#  here as well without editing this one.
#
#  Ordering matters:
#    fake_robot.py must own the FollowJointTrajectory action servers before
#    move_group looks for its controllers; simple_move must be subscribed to
#    /gui_commands before the GUI can publish anything that is not dropped.
# =============================================================================
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# my_robot now lives inside the workspace (it is versioned with it), so it is
# located relative to this file rather than via ~/. This file is installed to
# <ws>/install/robot_config/share/robot_config/launch/, hence the walk up.
MY_ROBOT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__),
                 '..', '..', '..', '..', '..', 'my_robot'))


def generate_launch_description():
    robot_config_launch = os.path.join(
        get_package_share_directory('robot_config'), 'launch')
    my_robot_control_launch = os.path.join(
        get_package_share_directory('my_robot_control'), 'launch')

    # 1. Fake hardware: /fake_joint_states + both action servers.
    fake_robot = ExecuteProcess(
        cmd=['python3', os.path.join(MY_ROBOT_DIR, 'fake_robot.py')],
        output='screen',
    )

    # 2. move_group + RViz + robot_state_publisher + joint_state_publisher.
    fake_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_config_launch, 'fake.launch.py')))

    # 3. simple_move -- the /gui_commands bridge. Opens its own gnome-terminal
    #    (prefix is set inside run_simple_move.launch.py). MoveGroupInterface
    #    blocks until move_group answers, so this delay is only to keep the
    #    startup log readable, not a correctness requirement.
    simple_move = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_control_launch, 'run_simple_move.launch.py')))

    # 4. The Tkinter front-end.
    gui = ExecuteProcess(
        cmd=['python3', os.path.join(MY_ROBOT_DIR, 'GUI.py')],
        output='screen',
    )

    return LaunchDescription([
        fake_robot,
        TimerAction(period=2.0,  actions=[fake_stack]),
        TimerAction(period=10.0, actions=[simple_move]),
        TimerAction(period=18.0, actions=[gui]),
    ])
