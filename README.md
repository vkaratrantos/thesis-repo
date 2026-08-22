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

## Dependencies

Built and tested on **Ubuntu 22.04 / ROS 2 Humble**.

This workspace is an overlay on two things that are **not** in this repo:

1. **MoveIt 2 built from source** in `~/ws_moveit2`.
2. **MoveIt Task Constructor**, cloned into the workspace root:

   ```bash
   cd <workspace>
   git clone https://github.com/moveit/moveit_task_constructor.git
   ```

   It is excluded via `.gitignore` because it is unmodified upstream source.

## Build

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run (simulated robot, no hardware)

```bash
./start_lab.sh
```

Starts, in order: the fake robot, MoveIt + RViz, the `simple_move` command
bridge, and the GUI. It kills any previous session first — a leftover
`move_group` or a second `simple_move` will silently steal or duplicate
trajectories.

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

## Known limitations

- Launch files contain absolute `/home/vkaratrantos/...` paths.
- `demo_gazebo.launch.py` does not register the `ompl` planning pipeline, so
  planning from the RViz panel fails there (the GUI path is unaffected).
