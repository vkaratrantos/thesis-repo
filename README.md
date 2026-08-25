# Automated Chemical Synthesis — 7-DOF Liquid-Handling Arm

ROS 2 Humble workspace for a MyArm 300 Pi with a custom gripper, used to
automate liquid-handling sequences (pick a tube, carry it to the mixer, pour,
return it). Planning is MoveIt 2 + MoveIt Task Constructor.

## Layout

| Path | What it is |
|------|-----------|
| `src/myarm_300_pi` | Robot description: URDF, meshes, joint config |
| `src/robot_config` | MoveIt configuration (SRDF, kinematics, OMPL, launch files) |
| `src/my_robot_control` | `simple_move` — the motion node driving the sequences |
| `src/simple_move` | Standalone minimal move example |
| `src/myarm_motion_planning` | Motion-planning experiments |
| `my_robot` | Hardware bridges, fake robot, and the Tkinter GUI |

Modes are mutually exclusive — run exactly one:

| Launch | Hardware interface | Physics |
|--------|--------------------|---------|
| `robot_config lab_fake.launch.py` | `fake_robot.py` action servers | none |
| `robot_config demo_gazebo.launch.py` | `gazebo_ros2_control` | Gazebo |
| `robot_config real.launch.py` | LAN driver / `moveit_bridge_5.py` | real arm |

## Command interface

The GUI publishes `std_msgs/String` on `/gui_commands`; `simple_move` executes
and replies `OK <cmd>` / `FAIL <cmd>` on `/gui_status`.

| Command | Action |
|---------|--------|
| `m<0-6>` | Move to marker N (OMPL transit + Cartesian descent) |
| `TASK <1-5>` | Full MTC pick-and-carry pipeline (needs camera TF) |
| `o` / `c` | Open / close gripper |
| `p` | Pour |
| `h` | Return home |

Without a camera publishing `marker_N` frames, `simple_move` falls back to
hardcoded tube positions; `TASK` requires the TF and will fail without it.
