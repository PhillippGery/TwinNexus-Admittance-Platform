# TwinNexus Admittance Platform

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B&logoColor=white" />
  <img src="https://img.shields.io/badge/Python-3.12-blue?logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/Hardware-UR5e%20e--Series-orange" />
  <img src="https://img.shields.io/badge/VLA-π0.5%20(openpi)-blueviolet" />
  <img src="https://img.shields.io/badge/GPU-RTX%205090%2032GB-green?logo=nvidia&logoColor=white" />
  <img src="https://img.shields.io/badge/License-MIT-green" />
</p>

A full-stack bimanual manipulation research platform built on two **UR5e e-Series** arms. The core contribution is a custom **`ros2_control` admittance controller** implemented from scratch in C++, which replaces the upstream trajectory controller and makes both arms compliantly backdrivable under external force. On top of compliant hardware control, the platform supports **GELLO teleoperation**, **multi-camera data collection** via LeRobot, and **π0.5 Vision-Language-Action fine-tuning** via openpi — forming a complete pipeline from physical setup to trained visuomotor policy.

---

## Table of Contents

- [Demo](#demo)
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Subsystems](#subsystems)
  - [Custom Admittance Controller](#custom-admittance-controller)
  - [GELLO Teleoperation Bridge](#gello-teleoperation-bridge)
  - [WSG32 Gripper Driver](#wsg32-gripper-driver)
  - [Camera System](#camera-system)
  - [Data Collection](#data-collection)
  - [VLA Fine-Tuning (π0.5)](#vla-fine-tuning-π05)
- [Key Algorithms](#key-algorithms)
- [Project Structure](#project-structure)
- [Installation & Build](#installation--build)
- [Running the System](#running-the-system)
- [Configuration Reference](#configuration-reference)
- [ROS2 Topic Reference](#ros2-topic-reference)
- [Dependencies](#dependencies)

---

## Demo

![Bimanual pick and place demo](figures/bimanual_pick_place_demo.gif)

*Bimanual pick-and-place of a rigid box using the custom admittance controller under GELLO teleoperation. Both arms remain compliantly backdrivable throughout the task.*

---

## Overview

The platform is structured as a layered stack:

| Layer | What it does |
|---|---|
| **Admittance control** | Custom C++ `ros2_control` plugin — arms yield compliantly to external force while tracking a joint reference |
| **Teleoperation** | GELLO bridge streams joint positions at 50 Hz into `admittance_controller/joint_references` |
| **Perception** | Three Intel RealSense D415 cameras (wrist left, wrist right, overhead) stream synchronized RGB video |
| **Data collection** | LeRobot 0.5.2 records multi-camera episodes as HuggingFace datasets |
| **Policy training** | `convert_twinnexus.py` converts LeRobot v3.0 → openpi v2.1 format; π0.5 is LoRA fine-tuned on an RTX 5090 |
| **Inference** | openpi policy server + `vla_interface_node` (ROS 2 client — in progress) |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    TwinNexus Admittance Platform                     │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │                  bimanual_ur5e_bringup                         │  │
│  │  ur5e_TwinNexus_hwl.launch.py   realsense_cameras.launch.py   │  │
│  │  ur5e_upstream_admittance.launch.py   ur5e_control_mode.launch │  │
│  └────────────────────────────────────────────────────────────────┘  │
│                               ▲                                      │
│        ┌──────────────────────┼──────────────────────┐              │
│        │                      │                      │              │
│  ┌─────┴───────────┐  ┌───────┴──────────┐  ┌───────┴──────────┐  │
│  │ur5e_admittance_ │  │  GELLO Bridge    │  │  wsg32_driver    │  │
│  │control (C++)    │  │  gello_bridge.py │  │  wsg_tcp.py      │  │
│  │                 │  │                  │  │  wsg32_node.py   │  │
│  │ Custom plugin:  │  │  GELLO → joint   │  │  WSG32 Ethernet  │  │
│  │ ControllerIface │  │  references      │  │  gripper control │  │
│  │ M·ẍ + D·ẋ + K·x│  │  @ 50 Hz         │  └──────────────────┘  │
│  │  = F_ext        │  └──────────────────┘                         │
│  └─────────────────┘                                                │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                   Camera System                               │   │
│  │  RealSense D415 × 3  (wrist_left / wrist_right / overhead)   │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                               │                                      │
│                               ▼                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              Data & Policy Pipeline (70_vla/)                 │   │
│  │  LeRobot 0.5.2 → twinnexus_record.py → HuggingFace dataset   │   │
│  │  convert_twinnexus.py (v3.0 → v2.1) → openpi fine-tune       │   │
│  │  train.sh (π0.5 LoRA, RTX 5090, ~23 GB VRAM)                │   │
│  │  serve.sh → policy server → vla_interface_node (ROS 2)        │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
              │
              ▼
     /joint_references → admittance_controller → UR5e hardware
```

---

## Subsystems

### Custom Admittance Controller

**Package:** `10_src/src/ur5e_admittance_control/`

The primary contribution of this repository. A `ros2_control` `ControllerInterface` plugin implemented in C++ that replaces the standard `scaled_joint_trajectory_controller` and makes each arm physically compliant under external force.

<details>
<summary><b>How it works</b></summary>

The controller runs per-joint compliance in the update loop using a second-order admittance law:

```
M·ẍ + D·ẋ + K·x = F_ext
```

Per update tick:
1. **Read** 6-DOF wrench from the FT sensor state interfaces (`force.x/y/z`, `torque.x/y/z`)
2. **Filter** the raw wrench with a first-order IIR: `w_f = (1 − α)·w_f + α·w_raw`  (α = 0.005)
3. **Apply deadband** — wrench magnitudes below `wrench_deadband` (5.5 N / N·m) are ignored to suppress sensor noise
4. **Compute compliance offset** for each active axis:
   ```
   offset = clamp((F_ext − D·ẋ) / K, ±max_position_offset)
   ```
5. **Rate-limit** the step: `Δq = clamp(target − q_last, ±max_position_step)`
6. **Write** the commanded position to the joint position command interfaces

The controller subscribes to `~/joint_references` (`trajectory_msgs/JointTrajectoryPoint`) to update the equilibrium reference positions — this is how the GELLO bridge moves the arm while preserving compliance.

**Interface requirements:**
- Command: `position` (6 joints)
- State: `position` + `velocity` (6 joints each) + `force.x/y/z` + `torque.x/y/z` from the FT sensor

</details>

**Admittance parameters (per-axis, Cartesian 6-DOF):**

| Axis | Mass (kg) | Damping (N·s/m) | Stiffness (N/m) |
|---|---|---|---|
| x, y, z | 20.0 | 40.0 | 450.0 |
| rx, ry, rz | 0.2 | 4.0 | 25.0 |

---

### GELLO Teleoperation Bridge

**Script:** `10_src/src/bimanual_ur5e_bringup/scripts/gello_bridge.py`

The GELLO leader arm streams its joint positions via the `gello_software` submodule. The bridge node republishes them directly to `/admittance_controller/joint_references` as `trajectory_msgs/JointTrajectoryPoint` at **50 Hz** with a **200 ms** rolling horizon.

<details>
<summary><b>Key design decisions</b></summary>

- Rate-limiting is applied to the **target signal** (not the position error) to prevent velocity faults without suppressing admittance compliance
- `mock_start_delay_s` startup delay holds the frozen center position before oscillation begins
- `mock:=true` mode validates the full bridge path without GELLO hardware attached

</details>

**Mock validation:**
```bash
ros2 run bimanual_ur5e_bringup gello_bridge.py --ros-args -p mock:=true -p mock_amp_rad:=0.02
```

**Live mode:**
```bash
ros2 run bimanual_ur5e_bringup gello_bridge.py
```

---

### WSG32 Gripper Driver

**Package:** `10_src/src/wsg32_ros2/` (also published as a [standalone repository](https://github.com/PhillippGery/wsg32_ros2))

Custom ROS 2 Jazzy driver for the Weiss WSG 32 Ethernet gripper using the GCL text command interface over TCP.

<details>
<summary><b>Architecture</b></summary>

- `wsg_tcp.py` — pure Python GCL TCP driver (no ROS dependency). Key method: `move_nonblocking(position_mm)` sends `MOVE` and returns immediately
- `wsg32_node.py` — ROS 2 wrapper: subscribes to `~/cmd_pos` (`std_msgs/Float32`, mm), publishes `~/joint_state` for rosbag2, exposes `~/home` and `~/ack_fault` services; applies `cmd_deadband_mm` to suppress GELLO encoder jitter
- Dual-socket architecture: separate TCP connections for commands and feedback polling

</details>

**Hardware prerequisite — enable the GCL text interface once:**
```
http://192.168.1.201 → Settings → Command Interface
```

**Validated status:** GCL interface confirmed, `move_nonblocking()` validated, dual-socket architecture working.

**Network note:** gripper lives on the isolated `192.168.1.X` subnet, separated from enterprise DHCP by IP aliasing on `enp128s31f6`.

**Launch:**
```bash
ros2 launch wsg32_driver wsg32_dual.launch.py
```

---

### Camera System

Three Intel RealSense D415 cameras configured for teleoperation and data collection:

| Role | Serial | ROS Topic |
|---|---|---|
| Wrist left | `151322062583` | `/wrist_left/color/image_raw` |
| Wrist right | `151422060684` | `/wrist_right/color/image_raw` |
| Overhead | TBD | `/overhead/color/image_raw` |

Depth is disabled by default to preserve USB3 bandwidth across three simultaneous streams.

**Launch:**
```bash
ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py
# Override overhead serial when assigned:
ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py overhead_serial:=<serial>
```

---

### Data Collection

**Script:** `60_scripts/twinnexus_record.py`

LeRobot `0.5.2` installed in `~/lerobot_env/` with `--system-site-packages` for `rclpy` access. Recordings are saved as HuggingFace LeRobot v3.0 datasets into `30_data/`.

```bash
source ~/lerobot_env/bin/activate
bash 60_scripts/record_episodes.sh
```

**Current dataset:**

| Dataset | Episodes | Task |
|---|---|---|
| `pick_place_001` | 30 | Pick screwdriver → paper box |

---

### VLA Fine-Tuning (π0.5)

**Directory:** `70_vla/`

π0.5 (Physical Intelligence openpi) fine-tuning pipeline for training visuomotor policies from the collected demonstrations.

<details>
<summary><b>Data pipeline — why a conversion step is needed</b></summary>

LeRobot 0.5.2 produces **v3.0 format** datasets. openpi's bundled LeRobot (0.1.0) expects **v2.1 format**. The conversion script `convert_twinnexus.py` bridges this gap — do not downgrade LeRobot as it would break recording.

</details>

**One-time setup:**
```bash
git clone --recurse-submodules git@github.com:Physical-Intelligence/openpi.git ~/openpi
cd ~/openpi && GIT_LFS_SKIP_SMUDGE=1 uv sync && GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
git apply ~/TwinNexus-Admittance-Platform/70_vla/openpi_changes.patch
ln -s ~/TwinNexus-Admittance-Platform/70_vla/twinnexus_policy.py \
      ~/openpi/src/openpi/policies/twinnexus_policy.py
cd ~/openpi && uv run huggingface-cli login
```

**Convert dataset:**
```bash
cd ~/openpi
uv run ~/TwinNexus-Admittance-Platform/70_vla/convert_twinnexus.py \
  --src ~/TwinNexus-Admittance-Platform/30_data/pick_place_001 \
  --repo pick_place_001_openpi \
  --task "pick up the screwdriver and place it in the paper box" \
  --overwrite
```

**Train (config: `pi05_twinnexus_finetune`):**
```bash
cd ~/openpi
uv run scripts/compute_norm_stats.py --config-name pi05_twinnexus_finetune
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/train.py pi05_twinnexus_finetune \
  --exp-name=ScrewdriverPickPlace_v1 --overwrite
```

Training uses LoRA fine-tuning (`gemma_2b_lora` + `gemma_300m_lora`), 10k steps, fits in ~23 GB VRAM on the RTX 5090.

---

## Key Algorithms

### Admittance Control Law (`src/admittance_controller.cpp`)

The per-joint compliance update implements a simplified admittance law in joint space (full Cartesian Jacobian solver in progress):

```
offset = clamp( (F_ext − D·q̇) / K,  ±max_position_offset )

q_cmd = clamp( q_ref + offset − q_last,  ±max_position_step ) + q_last
```

where `F_ext` is the IIR-filtered wrench component mapped to the joint axis, `q̇` is current joint velocity, and `K`, `D` are the per-axis stiffness and damping parameters.

**Safety limits enforced every tick:**
- `max_position_offset`: 0.01 m — caps the compliance displacement from the equilibrium reference
- `max_position_step`: 0.002 m — rate-limits the command to prevent velocity faults
- `wrench_deadband`: 5.5 N — suppresses FT sensor noise when no external force is applied
- Non-finite FT or joint state values are caught and the update returns `ERROR` immediately

### IIR Wrench Filter

```
w_filtered[k] = (1 − α) · w_filtered[k−1]  +  α · w_raw[k]     α = 0.005
```

Low-pass filter applied independently to all 6 wrench axes before the compliance law. Equivalent cutoff: ~0.08 Hz at 500 Hz control rate — removes sensor chatter without adding significant lag.

### GELLO Bridge Rate Limiter

The bridge applies rate-limiting to the **target joint position** signal, not to the error term. This prevents the admittance controller from seeing a velocity spike when GELLO moves quickly, while keeping the robot's compliance response fully active.

---

## Project Structure

```
TwinNexus-Admittance-Platform/
├── README.md
├── figures/
│   └── bimanual_pick_place_demo.gif      ← bimanual pick-and-place demo
├── 10_src/                               ← ROS 2 colcon workspace root
│   └── src/
│       ├── ur5e_admittance_control/      ← custom C++ admittance controller plugin
│       │   ├── include/admittance_controller/admittance_controller.hpp
│       │   ├── src/admittance_controller.cpp
│       │   ├── config/admittance_params.yaml
│       │   ├── admittance_controller.xml ← pluginlib export
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       ├── bimanual_ur5e_bringup/        ← launch files, config, bringup scripts
│       │   ├── launch/
│       │   │   ├── ur5e_TwinNexus_hwl.launch.py
│       │   │   ├── ur5e_upstream_admittance.launch.py
│       │   │   ├── ur5e_admittance_servo_teleop.launch.py
│       │   │   ├── ur5e_control_mode.launch.py
│       │   │   └── realsense_cameras.launch.py
│       │   ├── scripts/
│       │   │   ├── gello_bridge.py       ← GELLO → joint_references bridge
│       │   │   ├── admittance_keyboard_teleop.py
│       │   │   ├── moveit_servo_to_admittance.py
│       │   │   ├── return_home.py
│       │   │   └── set_servo_command_type.py
│       │   └── config/ur5e_admittance_controller.yaml
│       ├── wsg32_ros2/                   ← WSG32 gripper driver (git submodule)
│       ├── gello_software/               ← GELLO SDK (git submodule, pip-installed)
│       └── vla_interface_node/           ← ROS 2 inference client (in progress)
├── 20_docs/                              ← architecture, network, TF frame docs
├── 30_data/                              ← LeRobot datasets and rosbags
│   ├── pick_place_001/                   ← 30-episode screwdriver pick-place dataset
│   └── pick_place_001_openpi/            ← same dataset in openpi v2.1 format
├── 60_scripts/                           ← host utilities
│   ├── twinnexus_record.py               ← LeRobot recording entry point
│   └── record_episodes.sh
├── 70_vla/                               ← π0.5 fine-tuning pipeline
│   ├── twinnexus_policy.py               ← π0.5 input/output transforms
│   ├── convert_twinnexus.py              ← LeRobot v3.0 → openpi v2.1 converter
│   ├── openpi_changes.patch              ← patch for openpi training config
│   ├── train.sh
│   └── serve.sh
├── 70_hardware/                          ← CAD, BOMs, manuals
│   └── cad/                             ← custom 3D-printed parts (finger mounts, camera adapters)
└── 80_Docu/                              ← papers, reports, analysis
```

---

## Installation & Build

### Prerequisites

- **Ubuntu 24.04** + **ROS 2 Jazzy**
- **UR Robot Driver** (`ros-jazzy-ur-robot-driver`)
- **ros2_control**: `ros-jazzy-ros2-control ros-jazzy-ros2-controllers`
- **MoveIt 2**: `ros-jazzy-moveit`
- **Intel RealSense**: `ros-jazzy-realsense2-camera`
- **Python**: `~/lerobot_env/` with LeRobot 0.5.2 + PyTorch 2.11 (CUDA 12.8)

```bash
sudo apt install \
  ros-jazzy-ur-robot-driver \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-admittance-controller \
  ros-jazzy-kinematics-interface ros-jazzy-kinematics-interface-kdl \
  ros-jazzy-moveit ros-jazzy-moveit-servo \
  ros-jazzy-realsense2-camera \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-ur-moveit-config
```

### Build

```bash
cd ~/TwinNexus-Admittance-Platform/10_src

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build (symlink-install preserves Python script edits without rebuild)
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## Running the System

> **Note:** Always `source 10_src/install/setup.bash` before any `ros2` command. Run `boot_hw` to bring the UR hardware online before launching controllers.

### 1 — Bring up UR5e hardware

```bash
boot_hw   # alias for ur_robot_driver ur_control.launch.py
```

Press **Play** on the robot teach pendant before proceeding.

### 2 — Load the custom admittance controller

```bash
ros2 launch bimanual_ur5e_bringup ur5e_upstream_admittance.launch.py
```

This unloads the active trajectory controller and spawns `ur5e_admittance_control/AdmittanceController` with `admittance_params.yaml`.

### 3 — Start cameras

```bash
ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py
```

### 4a — GELLO Teleoperation

```bash
ros2 run bimanual_ur5e_bringup gello_bridge.py
```

### 4b — Keyboard TCP Teleop (alternative)

```bash
ros2 launch bimanual_ur5e_bringup ur5e_admittance_servo_teleop.launch.py
# then in a separate terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link \
  -p repeat_rate:=20.0 -p key_timeout:=0.25 \
  -p speed:=0.08 -p turn:=0.25 \
  -r /cmd_vel:=/servo_node/delta_twist_cmds
```

### 5 — WSG32 Grippers

```bash
ros2 launch wsg32_driver wsg32_dual.launch.py
```

### 6 — Record a teleoperation episode

```bash
source ~/lerobot_env/bin/activate
bash 60_scripts/record_episodes.sh
```

---

## Configuration Reference

### Admittance Controller (`admittance_params.yaml`)

| Parameter | Value | Description |
|---|---|---|
| `ft_sensor.name` | `tcp_fts_sensor` | FT sensor hardware interface name |
| `ft_sensor.filter_coefficient` | 0.005 | IIR low-pass coefficient (α) for wrench filtering |
| `wrench_deadband` | 5.5 N | Minimum wrench magnitude to activate compliance |
| `max_position_offset` | 0.01 m | Maximum compliance displacement from equilibrium |
| `max_position_step` | 0.002 m | Maximum position command step per control tick |
| `mass[x,y,z]` | 20.0 kg | Virtual mass for translational axes |
| `damping[x,y,z]` | 40.0 N·s/m | Damping for translational axes |
| `stiffness[x,y,z]` | 450.0 N/m | Stiffness for translational axes |
| `mass[rx,ry,rz]` | 0.2 kg | Virtual mass for rotational axes |
| `damping[rx,ry,rz]` | 4.0 N·m·s/rad | Damping for rotational axes |
| `stiffness[rx,ry,rz]` | 25.0 N·m/rad | Stiffness for rotational axes |

### GELLO Bridge

| Parameter | Default | Description |
|---|---|---|
| `mock` | `false` | Run without GELLO hardware (sine wave motion) |
| `mock_amp_rad` | 0.02 | Mock oscillation amplitude (rad) |
| `mock_start_delay_s` | configurable | Hold center before mock oscillation begins |
| Publish rate | 50 Hz | `joint_references` publish frequency |
| Rolling horizon | 200 ms | `time_from_start` on each JointTrajectoryPoint |

---

## ROS2 Topic Reference

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/admittance_controller/joint_references` | `JointTrajectoryPoint` | gello_bridge | admittance_controller |
| `/joint_states` | `JointState` | ur_robot_driver | admittance_controller |
| `/tcp_fts_sensor/wrench` | `WrenchStamped` | ur_robot_driver | admittance_controller |
| `/wrist_left/color/image_raw` | `Image` | realsense2_camera | LeRobot / vla_interface_node |
| `/wrist_right/color/image_raw` | `Image` | realsense2_camera | LeRobot / vla_interface_node |
| `/overhead/color/image_raw` | `Image` | realsense2_camera | LeRobot / vla_interface_node |
| `~/cmd_pos` | `Float32` | operator / gello_bridge | wsg32_node |
| `~/joint_state` | `JointState` | wsg32_node | rosbag2 |
| `/servo_node/delta_twist_cmds` | `TwistStamped` | teleop_twist_keyboard | moveit_servo |

---

## Dependencies

### ROS 2 Packages
`rclcpp` · `controller_interface` · `hardware_interface` · `pluginlib` · `ur_robot_driver` · `ros2_control` · `ros2_controllers` · `admittance_controller` · `kinematics_interface_kdl` · `moveit_servo` · `realsense2_camera` · `trajectory_msgs` · `sensor_msgs` · `geometry_msgs`

### Python
`rclpy` · `numpy` · `lerobot==0.5.2` · `torch==2.11` (CUDA 12.8) · `openpi` (uv venv)

### Hardware
**2× Universal Robots UR5e e-Series** · **2× Weiss WSG32 Ethernet gripper** · **3× Intel RealSense D415** · **GELLO teleoperation device** · **NVIDIA RTX 5090 (32 GB)**

### Policy
**π0.5** (Physical Intelligence openpi) · LoRA fine-tuning (`gemma_2b_lora` + `gemma_300m_lora`)

---

## Author

**Phillipp Gery** — Purdue University, MS Interdisciplinary Engineering (Autonomy & Robotics)  
Fulbright Scholar
