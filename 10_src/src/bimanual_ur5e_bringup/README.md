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

## UR5e control-mode launch

- `launch/ur5e_control_mode.launch.py` wraps the upstream UR driver bringup and adds `control_mode:=trajectory|admittance`.
- `control_mode:=trajectory` preserves the normal UR joint controller startup.
- `control_mode:=admittance` starts the driver without activating the trajectory controller, then auto-spawns the upstream ROS 2 `admittance_controller` with a UR5e-specific parameter file.
- `control_mode:=admittance` currently assumes `tf_prefix:=""`, matching the single-arm interface names exported by the UR driver.
- The previous `boot_hw` + `spawn_brain` flow remains available as a legacy path for the custom controller, but the supported admittance bringup path is now the launch file in this package.

Example commands:

```bash
ros2 launch bimanual_ur5e_bringup ur5e_control_mode.launch.py \
  control_mode:=trajectory \
  ur_type:=ur5e \
  robot_ip:=192.168.0.20 \
  kinematics_params_file:=/home/phillippgery/TwinNexus-Admittance-Platform/ur5e_factory_calibration.yaml \
  launch_rviz:=true
```

```bash
ros2 launch bimanual_ur5e_bringup ur5e_control_mode.launch.py \
  control_mode:=admittance \
  ur_type:=ur5e \
  robot_ip:=192.168.0.20 \
  kinematics_params_file:=/home/phillippgery/TwinNexus-Admittance-Platform/ur5e_factory_calibration.yaml \
  launch_rviz:=true
```

Required runtime packages for the upstream controller:

```bash
sudo apt install ros-jazzy-admittance-controller ros-jazzy-kinematics-interface ros-jazzy-kinematics-interface-kdl
```
