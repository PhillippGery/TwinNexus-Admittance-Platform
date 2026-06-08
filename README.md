# TwinNexus Admittance Platform

Repository scaffold for a bimanual UR5e e-Series robotic manipulation system combining ROS 2, low-level Admittance Control, and a Pi0.5 Vision-Language-Action pipeline for teleoperation and demonstration data collection.

## Structure

- `10_src/` - ROS 2 colcon workspace root and package source tree
- `20_docs/` - system architecture, network configuration, and TF/frame documentation
- `30_data/` - demonstrations, ROS bag captures, and calibration artifacts
- `60_scripts/` - host-side utilities for validation, visualization, and networking
- `70_hardware/` - CAD, BOMs, and lab-cell physical integration material
- `80_Docu/` - papers, reports, and analysis deliverables

## WSG32 Gripper Driver

Custom ROS 2 Jazzy support for the Weiss WSG 32 Ethernet gripper lives in `10_src/src/wsg32_driver/`.

- `wsg_tcp.py` - pure Python GCL TCP driver with no ROS dependency. It talks to the gripper over the text command interface on `192.168.1.200:1000`. The key teleoperation method is `move_nonblocking(position_mm)`, which sends `MOVE` and returns immediately.
- `wsg32_node.py` - ROS 2 wrapper node. It subscribes to `~/cmd_pos` (`std_msgs/Float32`, mm), publishes `~/joint_state` (`sensor_msgs/JointState`) for rosbag2 recording, and exposes `~/home` and `~/ack_fault` (`std_srvs/Trigger`). It also applies the `cmd_deadband_mm` filter to suppress GELLO encoder jitter.
- `config/wsg32_params.yaml` - driver parameters.
- `launch/wsg32.launch.py` - single-gripper launch.
- `launch/wsg32_dual.launch.py` - dual-gripper launch.
- `test_connection.py` - standalone hardware smoke test. Run this before launching the ROS node.

One-time hardware prerequisite: enable the GCL text interface in the gripper web UI:

```bash
http://192.168.1.200
```

Navigate to:

```bash
Settings -> Command Interface
```

Network note: the gripper lives on the isolated `192.168.1.X` ghost subnet, separated from the enterprise DHCP network by IP aliasing on `enp128s31f6`.

Recommended first test:

```bash
cd 10_src/src/wsg32_driver
python3 test_connection.py
```

ROS launch:

```bash
cd 10_src
colcon build --packages-select wsg32_driver
source install/setup.bash
ros2 launch wsg32_driver wsg32.launch.py
```

Dual-gripper launch:

```bash
cd 10_src
source install/setup.bash
ros2 launch wsg32_driver wsg32_dual.launch.py
```

## Notes

Most of the repository is still scaffold/configuration material, but active workspace code now includes the ROS 2 Jazzy WSG32 gripper driver under `10_src/src/wsg32_driver/`.
