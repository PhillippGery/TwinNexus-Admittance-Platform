# 10_src/src

*Package source tree for the bimanual UR5e ROS 2 workspace.*

## Contents

- `bimanual_ur5e_bringup/` - launch and parameter package for arms, cameras, and controllers
- `ur5e_admittance_control/` - C++ control-layer package for admittance integration
- `vla_interface_node/` - Python package for Pi0.5 VLA inference and data-facing interfaces

## Conventions

- New packages should use lowercase snake_case names consistent with ROS 2 package naming.
- Build from `10_src/` with `colcon build --symlink-install`.
- Runtime topics, frames, and controller names should remain namespaced to support left/right arm separation.
- Hardware-specific values such as IPs, calibration references, and controller parameters belong in launch/config files, not hardcoded source.
