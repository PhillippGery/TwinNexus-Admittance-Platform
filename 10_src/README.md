# 10_src

*ROS 2 colcon workspace root for all runtime packages in this project.*

## Contents

- `colcon.meta` - workspace-level colcon metadata placeholder
- `src/` - ROS 2 package source tree

## Conventions

- Build the workspace from this directory with `colcon build --symlink-install`.
- All ROS 2 packages must live under `10_src/src/`.
- C++ packages should use `ament_cmake`; Python packages should use ROS 2 Python packaging conventions.
- Generated folders such as `build/`, `install/`, and `log/` are local artifacts and must not be committed.
