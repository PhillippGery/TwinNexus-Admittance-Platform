# 10_src — ROS 2 Workspace

ROS 2 colcon workspace root for all runtime packages on the TwinNexus Admittance Platform.

---

## System Architecture

```
Policy / go_home / GELLO
        │
        ▼  JointState [j0…j5, gripper_m]
  twinnexus_bridge          (separate process · own GIL · 500 Hz)
        │  rate-limited JointTrajectoryPoint (2 ms lookahead)
        ▼
  admittance_controller     (always active — robot stays compliant)
        │
        ▼
      UR5e + WSG32
```

The bridge is the **single interpolation layer** for all command sources.
No rate limiting lives anywhere else — not in `TwinNexusRobot`, not in inference scripts.

---

## Quick Start — Three Terminals

### Terminal 1 — Hardware driver (always required)

```bash
boot_hw          # launches ur_robot_driver + RViz
                 # then press Play on the UR teach pendant
```

### Terminal 2A — Teleoperation / data collection

```bash
spawntele        # admittance controller + GELLO bridge (500 Hz)
go_home          # move robot to home position
```

### Terminal 2B — Inference / policy control

```bash
spawnctrl        # admittance controller + TwinNexus bridge (500 Hz)
go_home          # move robot to home position
```

> `go_home` publishes to both bridge topics automatically — it works in both modes.

---

## Aliases (defined in `~/.bashrc`)

| Alias | Command | Description |
|-------|---------|-------------|
| `boot_hw` | `ros2 launch ur_robot_driver ur_control.launch.py …` | Start UR5e hardware driver |
| `spawntele` | `ros2 launch bimanual_ur5e_bringup ur5e_TwinNexus_hwl.launch.py` | Teleoperation mode (GELLO bridge) |
| `spawnctrl` | `ros2 launch bimanual_ur5e_bringup ur5e_upstream_admittance.launch.py` | Inference / control mode (TwinNexus bridge) |
| `go_home` | `python3 …/return_home.py` | Send robot to home position via active bridge |
| `vla_env` | `source ROS + workspace + lerobot_env` | Activate full Python/ROS environment for VLA |
| `cb` | `cd $ROS_WS && colcon build --symlink-install && source install/setup.bash` | Build workspace |

---

## Data Collection (Teleoperation)

```bash
# Terminal 1
boot_hw

# Terminal 2
spawntele
go_home

# Terminal 3
vla_env
python ~/TwinNexus-Admittance-Platform/60_scripts/twinnexus_record.py \
  --repo-id PhillippGery/pick_place_001 \
  --num-episodes 20 \
  --episode-time-s 30 \
  --reset-time-s 15 \
  --task "pick and place red cube"
```

Controls during recording:

| Key | Action |
|-----|--------|
| `Enter` | Confirm ready / start next episode |
| `Ctrl+C` | Stop recording (saves completed episodes) |

---

## Inference (Policy Control)

```bash
# Terminal 1
boot_hw

# Terminal 2
spawnctrl
go_home

# Terminal 3
vla_env
python ~/TwinNexus-Admittance-Platform/60_scripts/twinnexus_record.py \
  --repo-id PhillippGery/pick_place_001 \
  --num-episodes 5 \
  --episode-time-s 30 \
  --reset-time-s 10 \
  --task "pick and place red cube"
```

---

## 4. Relaunch and Zero FT Sensors

After `boot_hw_bimanual` is running and both controller managers are active with RTDE pipelines streaming, zero the force-torque sensors **before** launching the admittance controllers. Skipping this step causes the admittance controller to see a false force offset and will make the robot drift or feel stiff.

```bash
ros2 service call /left_arm/io_and_status_controller/zero_ftsensor std_srvs/srv/Trigger {}
ros2 service call /right_arm/io_and_status_controller/zero_ftsensor std_srvs/srv/Trigger {}
```

Verify both streams are at the ambient noise floor (±1.0 N) before launching `spawnctrl_bimanual`:

```bash
ros2 topic echo /left_arm/force_torque_sensor_broadcaster/wrench --once
ros2 topic echo /right_arm/force_torque_sensor_broadcaster/wrench --once
```

Then launch the admittance controllers:

```bash
spawnctrl_bimanual          # or spawnctrl_bimanual teleop:=true for GELLO teleoperation
```

---

## Key ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `JointState` | UR5e actual joint positions (from driver) |
| `/twinnexus_bridge_right/target_joints` | `JointState` | Inference target [j0…j5, gripper_m] → bridge |
| `/twinnexus_bridge_right/go_home` | `JointState` (TRANSIENT_LOCAL) | Home command → bridge |
| `/twinnexus_bridge_right/commanded_position` | `JointState` | Bridge's current interpolated reference |
| `/admittance_controller/joint_references` | `JointTrajectoryPoint` | Bridge → admittance controller (500 Hz) |
| `/right_arm/wsg32_node/cmd_pos` | `Float32` (mm) | Gripper position command |
| `/right_arm/wsg32_node/joint_state` | `JointState` | Gripper actual position |

---

## Diagnostic Commands

```bash
# Confirm bridge is running
ros2 node list | grep twinnexus

# Check 500 Hz publish rate to admittance controller
ros2 topic hz /admittance_controller/joint_references

# Watch bridge interpolated reference live
ros2 topic echo /twinnexus_bridge_right/commanded_position

# Watch raw inference targets coming from policy
ros2 topic echo /twinnexus_bridge_right/target_joints
```

---

## Bridge Parameters (tunable at launch)

Defined in `bimanual_ur5e_bringup/launch/ur5e_upstream_admittance.launch.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_hz` | 500.0 | Timer rate for admittance controller commands |
| `tracking_delta_rad` | 0.002 | Max step per tick for inference targets (1.0 rad/s) |
| `go_home_delta_rad` | 0.001 | Max step per tick for go_home (0.5 rad/s) |
| `joint_states_topic` | `/joint_states` | Source of actual robot position |
| `admittance_topic` | `/admittance_controller/joint_references` | Output to controller |
| `gripper_topic` | `/right_arm/wsg32_node/cmd_pos` | Gripper command output |

---

## Home Position

Defined **once** in `bimanual_ur5e_bringup/scripts/return_home.py`:

```python
HOME_JOINTS     = [1.611, -1.392, -1.494, -1.627, -4.61, -1.732]   # rad
HOME_GRIPPER_MM = 50.0                                                # mm
```

`TwinNexusRobot.go_home()` and the `go_home` alias both import from this file — no duplication.

---

## Build

```bash
cd ~/TwinNexus-Admittance-Platform/10_src
colcon build --symlink-install
source install/setup.bash
```

Or use the `cb` alias from anywhere in the workspace.

Generated folders (`build/`, `install/`, `log/`) are local artifacts and are not committed.
