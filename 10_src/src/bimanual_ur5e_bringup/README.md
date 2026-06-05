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

Required runtime packages for the upstream controller:

```bash
sudo apt install ros-jazzy-admittance-controller ros-jazzy-kinematics-interface ros-jazzy-kinematics-interface-kdl
```
