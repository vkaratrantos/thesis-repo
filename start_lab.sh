#!/usr/bin/env bash
WS=/home/vkaratrantos/elephant_robots_ws
LOG=/home/vkaratrantos/.lab_launcher.log

exec > >(tee -a "$LOG") 2>&1
echo "=============== lab start $(date '+%F %T') ==============="
echo "[lab] clearing previous session..."
pkill -INT -f 'ros2 launch'   2>/dev/null
pkill -f 'gripper_mimic.py'   2>/dev/null
pkill -x simple_move          2>/dev/null
pkill -f 'fake_robot.py'      2>/dev/null
pkill -f 'my_robot/GUI.py'    2>/dev/null
pkill -x gzclient             2>/dev/null
pkill -x gzserver             2>/dev/null
sleep 3
pkill -KILL -x gzserver       2>/dev/null
pkill -KILL -x gzclient       2>/dev/null
pkill -KILL -x move_group     2>/dev/null
pkill -KILL -x rviz2          2>/dev/null
pkill -KILL -x simple_move    2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source /home/vkaratrantos/ws_moveit2/install/setup.bash
source "$WS/install/setup.bash"
export LC_NUMERIC="en_US.UTF-8"

cd "$WS"
echo "[lab] launching..."
exec ros2 launch robot_config lab_fake.launch.py
