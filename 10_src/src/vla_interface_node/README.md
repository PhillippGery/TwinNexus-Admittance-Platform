# vla_interface_node

*Python package scaffold for Pi0.5 VLA inference, teleoperation interfacing, and demonstration logging integration.*

## Contents

- `vla_interface_node/` - Python module namespace
- `resource/` - ROS 2 package resource marker
- `config/` - runtime config placeholders for model, camera, and policy settings
- `models/` - local references or lightweight metadata for model assets
- `test/` - package-local tests
- `package.xml` - ROS 2 package manifest stub
- `setup.py` - setuptools package stub
- `setup.cfg` - installation/script layout stub

## Conventions

- Model weights and large checkpoints should not be committed directly to this package.
- Python modules should stay focused on ROS interfacing, orchestration, and logging boundaries.
- Config files should externalize camera topics, frame IDs, and model/runtime selection.
- Data capture side effects should be explicit and traceable to paths under `30_data/`.
