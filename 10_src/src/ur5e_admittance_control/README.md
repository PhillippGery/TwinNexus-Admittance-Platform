# ur5e_admittance_control

*ROS 2 controller package for low-level UR5e admittance integration and controller-manager loading.*

## Contents

- `include/admittance_controller/` - public headers for the exported controller plugin
- `src/` - implementation sources
- `config/` - controller parameter YAML files
- `description/` - controller-related description fragments and integration notes
- `test/` - package-local tests and validation scaffolding
- `package.xml` - ROS 2 package manifest for the controller plugin
- `CMakeLists.txt` - `ament_cmake` build for the controller plugin library

## Conventions

- This package is reserved for the control path that must remain suitable for high-frequency operation.
- Public headers live under `include/admittance_controller/` so the exported include path matches the runtime plugin name.
- YAML configs should distinguish left/right arm controller instances explicitly.
- Experimental tooling, notebooks, or non-real-time helpers do not belong in this package.

## Runtime

- Build from `10_src/` with `colcon build --symlink-install`.
- Start the robot hardware and controller manager first, then use the existing `spawn_brain` alias that wraps `ros2 run controller_manager spawner admittance_controller --param-file .../admittance_params.yaml`.
- Before running `spawn_brain`, fully unload any controller that still owns the UR position interfaces (for example `scaled_joint_trajectory_controller`, `joint_trajectory_controller`, or `forward_position_controller`), otherwise controller activation will fail on interface contention.
