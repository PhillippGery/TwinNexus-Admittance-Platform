"""
twinnexus_robot.py
------------------
LeRobot-compatible robot class for the TwinNexus Admittance Platform.

Architecture:
    Cameras:      pyrealsense2 direct (no ROS) — 30fps guaranteed, zero wrapper overhead
    Robot state:  ROS 2 /joint_states subscriber
    Gripper state: ROS 2 WSG32 joint_state subscriber
    Commands:     ROS 2 admittance_controller + WSG32 cmd_pos publishers

Observation keys:
    "observation.state"              float32[7]   6 joints (rad) + 1 gripper (m)
    "observation.images.wrist_left"  uint8[720,1280,3]
    "observation.images.wrist_right" uint8[720,1280,3]  (empty string serial = disabled)
    "observation.images.overhead"    uint8[720,1280,3]  (empty string serial = disabled)

Action keys:
    "action"  float32[7]   6 joints (rad) + 1 gripper (m)

Usage:
    config = TwinNexusRobotConfig()
    with TwinNexusRobot(config) as robot:
        obs = robot.get_observation()
        robot.send_action({"action": obs["observation.state"]})
"""

import importlib.util
import logging
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pyrealsense2 as rs
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectoryPoint

from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot




logger = logging.getLogger(__name__)


# ── UR5e joint order (must match admittance controller config) ────────────────
UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# ── Camera image shape ────────────────────────────────────────────────────────
IMG_H = 480
IMG_W = 640
IMG_C = 3

# ── Control parameters ────────────────────────────────────────────────────────
LOOKAHEAD_NS  = 20_000_000  # 20ms rolling horizon — covers actual thread jitter

# 500Hz interpolation thread — mirrors GELLO bridge architecture.
# Policy (30Hz) sets a target; the thread smoothly moves toward it.
# Velocities are in rad/s so the speed is correct regardless of actual thread
# timing (Python sleep is imprecise; per-step deltas would cause speed errors).
INTERP_HZ           = 500
TRACKING_VEL_RAD_S  = 1.0    # rad/s — matches GELLO tracking_delta*500Hz
GO_HOME_VEL_RAD_S   = 0.5    # rad/s — matches GELLO bridge_delta*500Hz
HOME_CONVERGENCE_RAD = 0.01  # rad   — matches GELLO bridge threshold

# Single source of truth for home position lives in return_home.py.
# go_home() loads it via importlib so the values are never duplicated.
_RETURN_HOME_PY = os.path.expanduser(
    "~/TwinNexus-Admittance-Platform/10_src/src/"
    "bimanual_ur5e_bringup/scripts/return_home.py"
)


@RobotConfig.register_subclass("twinnexus")
@dataclass
class TwinNexusRobotConfig(RobotConfig):
    """
    Configuration for TwinNexusRobot.

    Camera serials: set to empty string "" to disable that camera.
    The serials below match the current TwinNexus hardware assignment.
    """
    # Robot identity
    id: str = "twinnexus_right"

    # ── ROS 2 topics ──────────────────────────────────────────────────────────
    joint_states_topic:   str = "/joint_states"
    gripper_state_topic:  str = "/right_arm/wsg32_node/joint_state"
    gripper_cmd_topic:    str = "/right_arm/wsg32_node/cmd_pos"
    admittance_cmd_topic: str = "/admittance_controller/joint_references"

    # ── Camera serial numbers (pyrealsense2 direct — no ROS) ──────────────────
    # Set to "" to disable a camera.
    wrist_left_serial:  str = "" # deatviated for first aprach"151322062583"   # D415 left wrist
    wrist_right_serial: str = "151422060684"   # D415 right wrist

    overhead_serial:    str = "146222254752"   # D455 overhead

    # LeRobot base class field
    calibration_dir: Path | None = None


class TwinNexusRobot(Robot):
    """
    LeRobot Robot implementation for the TwinNexus UR5e + WSG32 + RealSense platform.

    Cameras are read directly via pyrealsense2 — no ROS 2 camera nodes needed.
    Joint states and gripper state come from ROS 2 topics.
    Commands go to the admittance controller and WSG32 via ROS 2.
    """

    config_class = TwinNexusRobotConfig
    name = "twinnexus"

    def __init__(self, config: TwinNexusRobotConfig):
        super().__init__(config)
        self.config = config

        # ── State cache ───────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._joint_pos:   np.ndarray | None = None
        self._gripper_pos: float | None = None

        # ── 500Hz interpolation (ROS2 timer) ─────────────────────────────────────
        # Policy sets _target_*; the ROS2 timer callback smoothly moves toward it.
        # While _target_joints is None the callback is a no-op (safe during
        # recording where the GELLO bridge owns the admittance controller).
        self._target_joints:      list[float] | None = None
        self._target_gripper:     float | None = None
        self._last_interp_joints: list[float] | None = None  # last commanded pos
        self._interp_vel:         float = TRACKING_VEL_RAD_S
        self._interp_timer        = None   # rclpy.Timer — created in connect()
        self._t_last_interp:      float = 0.0
        self._interp_last_joints: list[float] | None = None

        # Camera frames — keyed by camera name
        self._frames: dict[str, np.ndarray | None] = {
            "wrist_left":  None,
            "wrist_right": None,
            "overhead":    None,
        }

        # ── pyrealsense2 pipelines ────────────────────────────────────────────
        # Maps camera name → (serial, rs.pipeline | None)
        self._rs_pipelines: dict[str, rs.pipeline | None] = {
            "wrist_left":  None,
            "wrist_right": None,
            "overhead":    None,
        }
        self._camera_thread:  threading.Thread | None = None
        self._camera_running: bool = False

        # ── ROS 2 internals ───────────────────────────────────────────────────
        self._ros_node:    Node | None = None
        self._executor:    MultiThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._spin_stop:   threading.Event = threading.Event()
        self._connected = False

    # ── LeRobot interface: features ───────────────────────────────────────────

    @property
    def observation_features(self) -> dict[str, Any]:
        feats: dict[str, Any] = {"observation.state": (7,)}
        if self.config.wrist_left_serial:
            feats["observation.images.wrist_left"]  = (IMG_H, IMG_W, IMG_C)
        if self.config.wrist_right_serial:
            feats["observation.images.wrist_right"] = (IMG_H, IMG_W, IMG_C)
        if self.config.overhead_serial:
            feats["observation.images.overhead"]    = (IMG_H, IMG_W, IMG_C)
        return feats

    @property
    def action_features(self) -> dict[str, Any]:
        return {"action": (7,)}

    # ── LeRobot interface: connection ─────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            logger.warning(f"{self} already connected.")
            return

        # ── Start pyrealsense2 cameras ────────────────────────────────────────
        serial_map = {
            "wrist_left":  self.config.wrist_left_serial,
            "wrist_right": self.config.wrist_right_serial,
            "overhead":    self.config.overhead_serial,
        }
        for cam_name, serial in serial_map.items():
            if not serial:
                continue
            try:
                pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_device(serial)
                cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, 30)
                pipeline.start(cfg)
                self._rs_pipelines[cam_name] = pipeline
                logger.info(f"{self} camera {cam_name} ({serial}) started.")
            except Exception as e:
                logger.warning(f"{self} camera {cam_name} ({serial}) failed to start: {e}")

        # Start camera polling thread
        self._camera_running = True
        self._camera_thread = threading.Thread(
            target=self._camera_loop,
            daemon=True,
            name="twinnexus_cameras",
        )
        self._camera_thread.start()

        # ── Start ROS 2 ───────────────────────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = Node("twinnexus_robot")

        self._ros_node.create_subscription(
            JointState,
            self.config.joint_states_topic,
            self._cb_joint_states,
            1,
        )
        self._ros_node.create_subscription(
            JointState,
            self.config.gripper_state_topic,
            self._cb_gripper_state,
            1,
        )

        self._joint_pub = self._ros_node.create_publisher(
            JointTrajectoryPoint,
            self.config.admittance_cmd_topic,
            1,
        )
        self._gripper_pub = self._ros_node.create_publisher(
            Float32,
            self.config.gripper_cmd_topic,
            1,
        )

        # 500Hz ROS2 timer — driven by executor, far more precise than time.sleep()
        self._t_last_interp = time.perf_counter()
        self._interp_timer = self._ros_node.create_timer(
            1.0 / INTERP_HZ,
            self._interp_callback,
        )

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._ros_node)
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_loop,
            daemon=True,
            name="twinnexus_ros_spin",
        )
        self._spin_thread.start()

        # Wait for first joint state
        logger.info(f"{self} waiting for joint states on {self.config.joint_states_topic} ...")
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self._lock:
                if self._joint_pos is not None:
                    break
            time.sleep(0.05)
        else:
            raise TimeoutError(
                f"{self} timed out waiting for joint states. "
                f"Is boot_hw running and Play pressed?"
            )

        # Wait for first camera frame from each active camera
        active_cams = [k for k, v in self._rs_pipelines.items() if v is not None]
        if active_cams:
            logger.info(f"{self} waiting for first camera frames ...")
            deadline = time.time() + 8.0
            while time.time() < deadline:
                with self._lock:
                    if all(self._frames[k] is not None for k in active_cams):
                        break
                time.sleep(0.05)
            else:
                logger.warning(f"{self} some cameras did not produce frames within 8s.")

        self._connected = True
        logger.info(
            f"{self} connected. "
            f"Joints: {self._joint_pos.tolist()} | "
            f"Cameras: {[k for k in active_cams if self._frames[k] is not None]}"
        )

    def pause_for_save(self) -> None:
        """Pause camera and ROS 2 spin (+ interp timer) before dataset.save_episode().

        Both compete heavily for the GIL and starve the image-writer threads and
        compute_episode_stats.  Cancelling the timer also stops 500Hz publishes
        so the admittance controller sits quietly during the save.
        """
        self._camera_running = False
        if self._camera_thread and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=2.0)

        if self._interp_timer is not None:
            self._interp_timer.cancel()

        self._spin_stop.set()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

    def resume_from_save(self) -> None:
        """Restart camera and ROS 2 spin (+ interp timer) after pause_for_save()."""
        self._t_last_interp = time.perf_counter()

        self._spin_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="twinnexus_ros_spin"
        )
        self._spin_thread.start()

        if self._interp_timer is not None:
            self._interp_timer.reset()

        self._camera_running = True
        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name="camera-loop"
        )
        self._camera_thread.start()

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._connected = False

        self._camera_running = False
        if self._camera_thread:
            self._camera_thread.join(timeout=2.0)

        if self._interp_timer is not None:
            self._interp_timer.cancel()

        # Stop pyrealsense2 pipelines
        for cam_name, pipeline in self._rs_pipelines.items():
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
                self._rs_pipelines[cam_name] = None

        # Stop ROS 2 spin thread, then shut down executor
        self._spin_stop.set()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if self._executor:
            self._executor.shutdown(timeout_sec=1.0)
        if self._ros_node:
            self._ros_node.destroy_node()
        self._ros_node = None
        self._executor = None
        logger.info(f"{self} disconnected.")

    # ── LeRobot interface: calibration / configure ────────────────────────────

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    # ── LeRobot interface: observation ────────────────────────────────────────
    def get_observation(self) -> dict[str, Any]:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        # ── Snapshot state under lock — fast, minimal time held ──────────────
        with self._lock:
            if self._joint_pos is None:
                raise RuntimeError(f"{self} has not received joint states yet.")
            joint_pos  = self._joint_pos.copy()
            gripper    = self._gripper_pos if self._gripper_pos is not None else 0.0
            frames_snap = {k: v.copy() if v is not None else None
                        for k, v in self._frames.items()}

        # ── Build observation outside lock ────────────────────────────────────
        state = np.append(joint_pos, gripper).astype(np.float32)
        obs: dict[str, Any] = {"observation.state": state}

        for cam_name in ("wrist_left", "wrist_right", "overhead"):
            serial_attr = f"{cam_name}_serial"
            if not getattr(self.config, serial_attr, ""):
                continue
            frame = frames_snap.get(cam_name)
            obs[f"observation.images.{cam_name}"] = (
                frame if frame is not None
                else np.zeros((IMG_H, IMG_W, IMG_C), dtype=np.uint8)
            )

        return obs

    # ── LeRobot interface: action ─────────────────────────────────────────────

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Update the interpolation target.  The 500Hz thread publishes smoothly."""
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        raw = np.asarray(action["action"], dtype=np.float32)
        if raw.shape != (7,):
            raise ValueError(f"Expected action shape (7,), got {raw.shape}")

        with self._lock:
            self._target_joints  = raw[:6].tolist()
            self._target_gripper = float(raw[6])

        return {"action": raw}

    def go_home(self) -> None:
        """Move to home position through the 500Hz interpolation thread.

        Uses GO_HOME_VEL_RAD_S (0.5 rad/s) — same as GELLO bridge bridge_delta.
        Blocks until the interpolated command position converges to home.
        Home values are read from return_home.py (single source of truth).
        """
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        if "return_home" not in sys.modules:
            spec = importlib.util.spec_from_file_location("return_home", _RETURN_HOME_PY)
            mod  = importlib.util.module_from_spec(spec)
            sys.modules["return_home"] = mod
            spec.loader.exec_module(mod)
        mod = sys.modules["return_home"]

        home_joints  = list(mod.HOME_JOINTS)
        home_gripper = mod.HOME_GRIPPER_MM / 1000.0  # mm → m

        # Stop cameras so their wait_for_frames() calls no longer compete for the
        # GIL — that competition starves spin_once() and causes the 500Hz timer
        # to miss beats, which makes the robot stop-and-go during homing.
        self._camera_running = False
        if self._camera_thread and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=2.0)

        self._interp_vel = GO_HOME_VEL_RAD_S
        with self._lock:
            self._target_joints  = home_joints
            self._target_gripper = home_gripper

        logger.info(
            "go_home: moving to %s at %.1f rad/s",
            [f"{v:.3f}" for v in home_joints],
            GO_HOME_VEL_RAD_S,
        )

        timeout = 30.0
        t_start = time.perf_counter()
        while time.perf_counter() - t_start < timeout:
            with self._lock:
                last = self._last_interp_joints
            if last is not None:
                if max(abs(l - h) for l, h in zip(last, home_joints)) < HOME_CONVERGENCE_RAD:
                    logger.info("go_home: home position reached.")
                    break
            time.sleep(0.05)
        else:
            logger.warning("go_home: timeout — robot may not have reached home.")

        self._interp_vel = TRACKING_VEL_RAD_S

        # Restart cameras now that homing is done
        self._camera_running = True
        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name="camera-loop"
        )
        self._camera_thread.start()

    # ── 500Hz ROS2 timer callback ─────────────────────────────────────────────

    def _interp_callback(self) -> None:
        """Called by the ROS2 executor at 500Hz — one interpolation tick.

        Replaces the Python thread approach.  ROS2 timers use the system's
        monotonic clock and are driven by the executor, giving far more accurate
        500Hz cadence than time.sleep().  This is the same architecture as the
        GELLO bridge (create_timer → publish loop).

        While _target_joints is None the callback is a no-op so the GELLO bridge
        retains exclusive control of the admittance controller during recording.
        """
        t_now     = time.perf_counter()
        # Cap actual_dt to 2× nominal so an occasional late tick never causes a
        # step large enough to hit the UR5e joint speed limiter.
        actual_dt = min(t_now - self._t_last_interp, 2.0 / INTERP_HZ)
        self._t_last_interp = t_now

        with self._lock:
            target_j  = list(self._target_joints) if self._target_joints is not None else None
            target_g  = self._target_gripper
            current_j = list(self._joint_pos)     if self._joint_pos     is not None else None

        if target_j is None:
            return  # no-op: GELLO bridge owns the controller

        # Seed from actual robot position on first active call
        if self._interp_last_joints is None:
            self._interp_last_joints = current_j[:] if current_j is not None else target_j[:]

        vel   = self._interp_vel
        delta = vel * actual_dt

        new_joints = [
            prev + max(-delta, min(delta, goal - prev))
            for prev, goal in zip(self._interp_last_joints, target_j)
        ]

        pt = JointTrajectoryPoint()
        pt.positions       = new_joints
        pt.velocities      = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=LOOKAHEAD_NS)
        self._joint_pub.publish(pt)

        # Gripper: pass target directly — WSG32 node handles its own motion control
        if target_g is not None:
            g_msg = Float32()
            g_msg.data = float(target_g * 1000.0)   # m → mm for WSG32
            self._gripper_pub.publish(g_msg)

        self._interp_last_joints = new_joints

        with self._lock:
            self._last_interp_joints = new_joints[:]

    # ── ROS 2 spin loop ────────────────────────────────────────────────────────

    def _spin_loop(self) -> None:
        """Drive the ROS 2 executor in a loop that can be paused via _spin_stop.

        timeout_sec=0.001 (1ms) lets the executor service the 500Hz timer
        (2ms period) without missing beats.  With 10ms it would only fire ~100Hz.
        """
        while not self._spin_stop.is_set():
            self._executor.spin_once(timeout_sec=0.001)

    # ── Camera loop (pyrealsense2 direct) ─────────────────────────────────────

    def _camera_loop(self) -> None:
        """
        Background thread: polls all active pyrealsense2 pipelines and
        writes frames into the cache at up to 30fps per camera.
        Non-blocking — uses wait_for_frames with a short timeout so
        the thread stays responsive to shutdown signals.
        """
        while self._camera_running:
            time.sleep(0)  # yield GIL so image-writer / async threads can run
            for cam_name, pipeline in self._rs_pipelines.items():
                if pipeline is None:
                    continue
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=50)
                    color = frames.get_color_frame()
                    if color:
                        frame = np.asarray(color.get_data(), dtype=np.uint8)
                        with self._lock:
                            self._frames[cam_name] = frame
                except RuntimeError:
                    pass   # timeout — no frame this cycle, keep going
                except Exception as e:
                    logger.warning(f"{self} camera {cam_name} error: {e}")

    # ── ROS 2 callbacks ───────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            pos = np.array([name_to_pos[n] for n in UR5E_JOINTS], dtype=np.float32)
            with self._lock:
                self._joint_pos = pos
        except KeyError:
            pass

    def _cb_gripper_state(self, msg: JointState) -> None:
        if msg.position:
            with self._lock:
                self._gripper_pos = float(msg.position[0])