# bimanual_ur5e_bringup

*Launch, parameter, and visualization scaffolding for safely bringing up both UR5e arms, cameras, and controller stacks.*

## Contents

- `launch/` - ROS 2 launch entry points for workstation, robot, and camera bringup flows
- `config/` - YAML configuration placeholders for controller, camera, and runtime parameters
- `rviz/` - RViz configuration placeholders for operator visualization
- `package.xml` - ROS 2 package manifest stub
- `CMakeLists.txt` - build-system stub for an `ament_cmake` package

## Conventions

- Launch files should follow `<system>_<mode>.launch.py`, such as `bimanual_cell_bringup.launch.py`.
- Parameter files should be YAML and grouped by subsystem or environment.
- Left/right arm resources should use explicit namespaces rather than implicit naming.
- Safety-critical startup sequencing belongs here, not in ad hoc scripts.
