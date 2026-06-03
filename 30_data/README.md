# 30_data

*Storage area for demonstrations, ROS bag captures, and calibration artifacts used by the VLA and robotics stack.*

## Contents

- `demos/` - structured demonstration sessions and trajectory exports
- `rosbags/` - raw or canonical ROS bag recordings
- `calibrations/` - camera intrinsics, extrinsics, and frame-alignment calibration files
- `processed/` - derived datasets, cleaned exports, and analysis-ready transforms

## Conventions

- Demonstration folders should use a sortable naming pattern such as `YYYYMMDD_sessionNN_task-slug/`.
- ROS bag names should remain machine-readable, for example `YYYYMMDD_task-slug_takeNN`.
- Calibration files should encode sensor identity and revision, such as `realsense_left_wrist_intrinsics.yaml`.
- Large data artifacts may need external storage or LFS policies later; until then, keep this tree organized for clear migration.
