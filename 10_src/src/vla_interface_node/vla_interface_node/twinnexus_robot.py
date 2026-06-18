"""
twinnexus_robot.py
------------------
LeRobot-compatible robot class for the TwinNexus Admittance Platform.

Architecture:
    Cameras:      pyrealsense2 direct (no ROS) — 30fps guaranteed, zero wrapper overhead
    Robot state:  ROS 2 /joint_states subscriber
    Gripper state: ROS 2 WSG32 joint_state subscriber
    Commands:     Published to twinnexus_bridge (separate process) via JointState topics.
                  The bridge interpolates at 500Hz and forwards to the admittance controller.
                  No interpolation logic lives here — the bridge owns that.

Observation keys:
    "observation.state"              float32[7]   6 joints (rad) + 1 gripper (m)
    "observation.images.wrist_left"  uint8[480,640,3]
    "observation.images.wrist_right" uint8[480,640,3]  (empty string serial = disabled)
    "observation.images.overhead"    uint8[480,640,3]  (empty string serial = disabled)

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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

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

# ── Single source of truth for home position ──────────────────────────────────
_RETURN_HOME_PY = os.path.expanduser(
    "~/TwinNexus-Admittance-Platform/10_src/src/"
    "bimanual_ur5e_bringup/scripts/return_home.py"
)


def _load_home() -> tuple[list[float], float]:
    """Load HOME_JOINTS and HOME_GRIPPER_MM from return_home.py (cached)."""
    if "return_home" not in sys.modules:
        spec = importlib.util.spec_from_file_location("return_home", _RETURN_HOME_PY)
        mod  = importlib.util.module_from_spec(spec)
        sys.modules["return_home"] = mod
        spec.loader.exec_module(mod)
    mod = sys.modules["return_home"]
    return list(mod.HOME_JOINTS), float(mod.HOME_GRIPPER_MM) / 1000.0   # joints, gripper_m


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

    # ── ROS 2 topics — state (read-only) ─────────────────────────────────────
    joint_states_topic:  str = "/joint_states"
    gripper_state_topic: str = "/right_arm/wsg32_node/joint_state"

    # ── ROS 2 topics — bridge (write) ─────────────────────────────────────────
    # The twinnexus_bridge node interpolates at 500Hz and publishes to the
    # admittance controller.  These three topics are its interface.
    bridge_target_topic:  str = "/twinnexus_bridge_right/target_joints"
    bridge_go_home_topic: str = "/twinnexus_bridge_right/go_home"
    bridge_status_topic:  str = "/twinnexus_bridge_right/commanded_position"

    # ── Camera serial numbers (pyrealsense2 direct — no ROS) ──────────────────
    # Set to "" to disable a camera.
    wrist_left_serial:  str = ""               # D415 left wrist (disabled)
    wrist_right_serial: str = "151422060684"   # D415 right wrist
    overhead_serial:    str = "146222254752"   # D455 overhead

    # LeRobot base class field
    calibration_dir: Path | None = None


class TwinNexusRobot(Robot):
    """
    LeRobot Robot implementation for the TwinNexus UR5e + WSG32 + RealSense platform.

    Cameras are read directly via pyrealsense2 — no ROS 2 camera nodes needed.
    Joint states and gripper state come from ROS 2 topics.
    Commands are published to the twinnexus_bridge node (separate process) which
    handles 500Hz interpolation and forwarding to the admittance controller.
    """

    config_class = TwinNexusRobotConfig
    name = "twinnexus"

    def __init__(self, config: TwinNexusRobotConfig):
        super().__init__(config)
        self.config = config

        # ── State cache ───────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._joint_pos:             np.ndarray | None  = None
        self._gripper_pos:           float | None        = None
        self._bridge_commanded_pos:  list[float] | None = None  # from bridge status

        # Camera frames — keyed by camera name
        self._frames: dict[str, np.ndarray | None] = {
            "wrist_left":  None,
            "wrist_right": None,
            "overhead":    None,
        }

        # ── pyrealsense2 pipelines ────────────────────────────────────────────
        self._rs_pipelines: dict[str, rs.pipeline | None] = {
            "wrist_left":  None,
            "wrist_right": None,
            "overhead":    None,
        }
        self._camera_thread:  threading.Thread | None = None
        self._camera_running: bool = False

        # ── ROS 2 internals ───────────────────────────────────────────────────
        self._ros_node:    Node | None = None
        self._executor:    SingleThreadedExecutor | None = None
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

        self._camera_running = True
        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name="twinnexus_cameras"
        )
        self._camera_thread.start()

        # ── Start ROS 2 ───────────────────────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = Node("twinnexus_robot")

        # State subscribers
        self._ros_node.create_subscription(
            JointState, self.config.joint_states_topic, self._cb_joint_states, 1
        )
        self._ros_node.create_subscription(
            JointState, self.config.gripper_state_topic, self._cb_gripper_state, 1
        )
        self._ros_node.create_subscription(
            JointState, self.config.bridge_status_topic, self._cb_bridge_status, 1
        )

        # Bridge publishers
        self._target_pub = self._ros_node.create_publisher(
            JointState, self.config.bridge_target_topic, 1
        )
        _go_home_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._go_home_pub = self._ros_node.create_publisher(
            JointState, self.config.bridge_go_home_topic, _go_home_qos
        )

        # Spin with SingleThreadedExecutor — only subscribers, no timer needed here
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros_node)
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="twinnexus_ros_spin"
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
                "Is boot_hw running and Play pressed?"
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
        """Pause cameras and ROS 2 spin before dataset.save_episode().

        Both compete for the GIL and starve image-writer threads.
        The bridge (separate process) keeps running — it holds last commanded
        position and remains compliant during save.
        """
        self._camera_running = False
        if self._camera_thread and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=2.0)

        self._spin_stop.set()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

    def resume_from_save(self) -> None:
        """Restart cameras and ROS 2 spin after pause_for_save()."""
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="twinnexus_ros_spin"
        )
        self._spin_thread.start()

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

        for cam_name, pipeline in self._rs_pipelines.items():
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
                self._rs_pipelines[cam_name] = None

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

        with self._lock:
            if self._joint_pos is None:
                raise RuntimeError(f"{self} has not received joint states yet.")
            joint_pos   = self._joint_pos.copy()
            gripper     = self._gripper_pos if self._gripper_pos is not None else 0.0
            frames_snap = {k: v.copy() if v is not None else None
                           for k, v in self._frames.items()}

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
        """Forward action target to the twinnexus_bridge for 500Hz interpolation.

        The bridge (separate process) rate-limits the step and publishes
        JointTrajectoryPoint to the admittance controller at 500Hz.
        """
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        raw = np.asarray(action["action"], dtype=np.float32)
        if raw.shape != (7,):
            raise ValueError(f"Expected action shape (7,), got {raw.shape}")

        msg = JointState()
        msg.position = raw.tolist()   # [j0…j5, gripper_m] — 7 values
        self._target_pub.publish(msg)

        return {"action": raw}

    def go_home(self) -> None:
        """Move to home position via the twinnexus_bridge.

        Publishes home target (TRANSIENT_LOCAL so the bridge receives it even
        if it starts after this call), then blocks until the bridge's commanded
        position converges to home.  Home values come from return_home.py —
        single source of truth, no duplication.
        """
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        home_joints, home_gripper = _load_home()

        msg = JointState()
        msg.position = home_joints + [home_gripper]   # 7 values
        self._go_home_pub.publish(msg)

        logger.info("go_home: homing via bridge to %s", [f"{v:.3f}" for v in home_joints])

        # Poll bridge's commanded_position until it reaches home
        timeout = 30.0
        t_start = time.perf_counter()
        while time.perf_counter() - t_start < timeout:
            with self._lock:
                cmd = self._bridge_commanded_pos
            if cmd is not None:
                if max(abs(c - h) for c, h in zip(cmd, home_joints)) < 0.01:
                    logger.info("go_home: home position reached.")
                    return
            time.sleep(0.05)

        logger.warning("go_home: timeout — robot may not have fully reached home.")

    # ── ROS 2 spin loop ───────────────────────────────────────────────────────

    def _spin_loop(self) -> None:
        while not self._spin_stop.is_set():
            self._executor.spin_once(timeout_sec=0.01)

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

    def _cb_bridge_status(self, msg: JointState) -> None:
        with self._lock:
            self._bridge_commanded_pos = list(msg.position)

    # ── Camera loop (pyrealsense2 direct) ─────────────────────────────────────

    def _camera_loop(self) -> None:
        while self._camera_running:
            time.sleep(0)   # yield GIL
            for cam_name, pipeline in self._rs_pipelines.items():
                if pipeline is None:
                    continue
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=50)
                    color  = frames.get_color_frame()
                    if color:
                        frame = np.asarray(color.get_data(), dtype=np.uint8)
                        with self._lock:
                            self._frames[cam_name] = frame
                except RuntimeError:
                    pass
                except Exception as e:
                    logger.warning(f"{self} camera {cam_name} error: {e}")
