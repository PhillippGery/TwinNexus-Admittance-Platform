# bimanual_ur5e_bringup

*Launch, parameter, and visualization scaffolding for safely bringing up both UR5e arms, cameras, and controller stacks.*

## Contents

- `launch/` - ROS 2 launch entry points for workstation, robot, and camera bringup flows
- `config/` - YAML configuration placeholders for controller, camera, and runtime parameters
- `rviz/` - RViz configuration placeholders for operator visualization
- `package.xml` - ROS 2 package manifest for bringup runtime dependencies
- `CMakeLists.txt` - build-system file for installing bringup assets

## Conventions

- Launch files should follow `<system>_<mode>.launch.py`, such as `bimanual_cell_bringup.launch.py`.
- Parameter files should be YAML and grouped by subsystem or environment.
- Left/right arm resources should use explicit namespaces rather than implicit naming.
- Safety-critical startup sequencing belongs here, not in ad hoc scripts.

## UR5e upstream admittance launch

- Keep hardware bringup on the proven UR path (`boot_hw` / `ur_robot_driver ur_control.launch.py`).
- After hardware is up, `launch/ur5e_upstream_admittance.launch.py` unloads the active trajectory controller and then spawns the official ROS 2 `admittance_controller`.
- The upstream admittance parameter file is `config/ur5e_admittance_controller.yaml`.
- This flow currently assumes `tf_prefix:=""`, matching the single-arm interface names exported by the UR driver.

Example commands:

```bash
boot_hw
```

```bash
ros2 launch bimanual_ur5e_bringup ur5e_upstream_admittance.launch.py
```

## Admittance + keyboard TCP teleop

- Keep `boot_hw` as-is.
- Press **Play** on the robot before enabling admittance or teleop.
- Run the admittance controller launch first, then start the Servo teleop launch.
- This uses the standard ROS stack:
  - `ur_moveit_config` + `moveit_servo`
  - `teleop_twist_keyboard`
  - a small bridge in this package from Servo's joint output to `admittance_controller/joint_references`

Example:

```bash
boot_hw
```

```bash
ros2 launch bimanual_ur5e_bringup ur5e_upstream_admittance.launch.py
```

```bash
ros2 launch bimanual_ur5e_bringup ur5e_admittance_servo_teleop.launch.py
```

Then, in a separate interactive terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link -p repeat_rate:=20.0 -p key_timeout:=0.25 -p speed:=0.08 -p turn:=0.25 -r /cmd_vel:=/servo_node/delta_twist_cmds
```

Keyboard usage comes from `teleop_twist_keyboard`:

- `i` / `,` for +X / -X in the chosen command frame
- `J` / `L` (uppercase) for +Y / -Y strafing
- `t` / `b` for +Z / -Z
- lowercase `j` / `l` command yaw, so they will look like circular TCP motion
- `k` stops motion
- `q/z`, `w/x`, `e/c` to scale speeds

For easier first tests, keep `twist_frame_id:=base_link`. If you later want tool-relative motion, set `twist_frame_id:=tool0`.

```bash
ros2 run bimanual_ur5e_bringup admittance_keyboard_teleop.py
```

The keyboard helper publishes small joint-reference steps into the active
`admittance_controller`, so the robot stays compliant while you nudge its
equilibrium pose from the PC. The MoveIt Servo launch above is the preferred path for TCP Cartesian teleop.

Required runtime packages for the upstream controller:

```bash
sudo apt install ros-jazzy-admittance-controller ros-jazzy-kinematics-interface ros-jazzy-kinematics-interface-kdl
```

Additional runtime packages for the Servo teleop path:

```bash
sudo apt install ros-jazzy-moveit-servo ros-jazzy-teleop-twist-keyboard ros-jazzy-ur-moveit-config
```
