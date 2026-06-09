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
http://192.168.1.201
```

Navigate to:

```bash
Settings -> Command Interface
```

Network note: the gripper lives on the isolated `192.168.1.X` ghost subnet, separated from the enterprise DHCP network by IP aliasing on `enp128s31f6`.

Validated status:

- GCL text interface confirmed working
- `move_nonblocking()` validated over TCP
- `GETSTATE` feedback polling confirmed on a second TCP connection
- dual-socket architecture (command + feedback) working
- package also published as a standalone repository: `https://github.com/PhillippGery/wsg32_ros2`

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

## Camera System

Three Intel RealSense D415 cameras are configured for the teleoperation and data-collection stack.

| Role | Serial | Topic |
|---|---|---|
| Wrist left | `151322062583` | `/wrist_left/color/image_raw` |
| Wrist right | `151422060684` | `/wrist_right/color/image_raw` |
| Overhead | `TBD` | `/overhead/color/image_raw` |

Launch file:

```bash
ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py
```

Override the overhead serial when assigned:

```bash
ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py overhead_serial:=<serial>
```

Depth is disabled by default to preserve USB3 bandwidth across three simultaneous streams. Enable it explicitly if needed:

```bash
ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py enable_depth:=true
```

## GELLO Teleoperation Bridge

`gello_software` from Stanford is added as a git submodule at `10_src/src/gello_software/`:

```bash
https://github.com/wuphilipp/gello_software
```

A `COLCON_IGNORE` marker is present because the package is installed into the LeRobot venv via `pip`, not built with `colcon`.

The bridge node `gello_bridge.py` in `10_src/src/bimanual_ur5e_bringup/scripts/` publishes GELLO joint positions directly to `/admittance_controller/joint_references` as `trajectory_msgs/JointTrajectoryPoint` at `50 Hz` with a `200 ms` rolling horizon.

Key design decisions:

- rate limiter on the target signal, not on the position error, to prevent velocity faults without suppressing admittance compliance
- `mock_start_delay_s` startup delay holds the frozen center before oscillation begins
- `mock:=true` mode validates the full bridge path without GELLO hardware

Mock validation:

```bash
ros2 run bimanual_ur5e_bringup gello_bridge.py --ros-args -p mock:=true -p mock_amp_rad:=0.02
```

Live mode:

```bash
ros2 run bimanual_ur5e_bringup gello_bridge.py
```

GELLO offset calibration (run once when hardware arrives):

```bash
cd 10_src/src/gello_software
python scripts/gello_get_offset.py \
  --start-joints 0 0 0 0 0 0 \
  --joint-signs 1 1 1 1 1 1 \
  --port /dev/ttyUSB0
```

## ML / Data Collection Environment

LeRobot `0.5.2` is installed in a dedicated virtual environment at `~/lerobot_env` with `--system-site-packages` so it can import `rclpy` from the system ROS 2 Jazzy install.

```bash
source ~/lerobot_env/bin/activate
python3 -c "import lerobot; import rclpy; print('OK')"
```

PyTorch `2.11` with CUDA `12.8` wheels is confirmed working on the RTX `5090` (`sm_120` / Blackwell, `32 GB` VRAM).

`TwinNexusRobot` will be added in a later update to bridge ROS 2 topics into the LeRobot observation format for episode recording.

## Notes

The repository now contains active, validated system components:

- admittance controller running and validated; compliant behavior confirmed under manual push
- WSG32 gripper driver built, tested, and split out as a standalone repository
- three-camera system launch file ready; both wrist-camera serials assigned
- GELLO bridge mock-validated; real hardware integration still pending
- LeRobot + PyTorch environment installed and verified
