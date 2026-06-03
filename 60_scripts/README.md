# 60_scripts

*Auxiliary host-side utilities for validation, visualization, and network diagnostics.*

## Contents

- `validation/` - scripts for dataset checks, file integrity, and run sanity checks
- `visualization/` - plotting or playback helpers
- `networking/` - connectivity, latency, and RTDE-related diagnostics

## Conventions

- Scripts here are support tooling, not ROS 2 package runtime nodes.
- Use descriptive verb-first names such as `validate_*`, `plot_*`, or `check_*`.
- Scripts should avoid hidden side effects and make input/output paths explicit.
- Anything required at runtime by ROS 2 should live in a package under `10_src/src/` instead.
