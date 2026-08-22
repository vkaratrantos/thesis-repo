#!/usr/bin/env bash
# =============================================================================
#  Chemical Synthesis lab -- single entry point (fake-robot mode).
#
#  Step 1 is the important one. A half-dead Gazebo leaves gzserver +
#  move_group + robot_state_publisher running with no visible window; the
#  second move_group then answers /move_action and silently swallows every
#  trajectory, so the fake robot never moves and nothing reports an error.
# =============================================================================
# No `set -u`: ROS's setup.bash references unbound variables
# (AMENT_TRACE_SETUP_FILES) and aborts under it. No `set -e` either --
# pkill exits 1 when nothing matched, which is the normal clean case.

WS=/home/vkaratrantos/elephant_robots_ws
LOG=/home/vkaratrantos/.lab_launcher.log

exec > >(tee -a "$LOG") 2>&1
echo "=============== lab start $(date '+%F %T') ==============="

# --- 1. Clean slate ----------------------------------------------------------
# Note: NOT `pkill -9 -f ros` -- that pattern matches anything with "ros" in
# its command line, including this script's own shell and /opt/ros binaries.
echo "[lab] clearing previous session..."
pkill -INT -f 'ros2 launch'   2>/dev/null
pkill -f 'gripper_mimic.py'   2>/dev/null
# simple_move is launched through `prefix=["gnome-terminal --"]`, so the
# real process is a child of gnome-terminal-server, NOT of ros2 launch.
# Killing the launch leaves it alive; every restart then adds another
# /gui_commands subscriber and the arm repeats each move once per copy.
# -x matches the executable name exactly, so this cannot match this script.
pkill -x simple_move          2>/dev/null
pkill -f 'fake_robot.py'      2>/dev/null
pkill -f 'my_robot/GUI.py'    2>/dev/null   # path-suffix match; works either location
pkill -x gzclient             2>/dev/null
pkill -x gzserver             2>/dev/null
sleep 3
# Anything that ignored SIGINT gets SIGKILL.
pkill -KILL -x gzserver       2>/dev/null
pkill -KILL -x gzclient       2>/dev/null
pkill -KILL -x move_group     2>/dev/null
pkill -KILL -x rviz2          2>/dev/null
pkill -KILL -x simple_move    2>/dev/null
sleep 1

# --- 2. Environment ----------------------------------------------------------
source /opt/ros/humble/setup.bash
source /home/vkaratrantos/ws_moveit2/install/setup.bash
source "$WS/install/setup.bash"
# RViz/Gazebo config parsing breaks under a comma decimal separator.
export LC_NUMERIC="en_US.UTF-8"

# --- 3. Go -------------------------------------------------------------------
cd "$WS"
echo "[lab] launching..."
exec ros2 launch robot_config lab_fake.launch.py
