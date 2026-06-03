# ur5e_admittance_control

*Placeholder C++ package for the low-level UR5e admittance-control integration layer.*

## Contents

- `include/ur5e_admittance_control/` - public headers for controller wrappers and interfaces
- `src/` - implementation sources
- `config/` - controller parameter YAML files
- `description/` - controller-related description fragments and integration notes
- `test/` - package-local tests and validation scaffolding
- `package.xml` - ROS 2 package manifest stub
- `CMakeLists.txt` - build-system stub for an `ament_cmake` package

## Conventions

- This package is reserved for the control path that must remain suitable for high-frequency operation.
- Public headers live under `include/ur5e_admittance_control/` and should mirror implementation names where practical.
- YAML configs should distinguish left/right arm controller instances explicitly.
- Experimental tooling, notebooks, or non-real-time helpers do not belong in this package.
