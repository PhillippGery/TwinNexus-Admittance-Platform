"""
twinnexus_robot.py
------------------
LeRobot-compatible robot class for the TwinNexus Admittance Platform.

Bridges ROS 2 topics into the LeRobot Robot interface so the standard
LeRobot record/replay/train pipeline works with the UR5e + WSG32 + D415 stack.

Architecture:
    ROS 2 callbacks write continuously into a local state cache.
    get_observation() reads from the cache — never blocks, never waits for ROS 2.
    send_action() publishes joint references to the admittance controller.

Observation keys (match these EXACTLY in your dataset config):
    "observation.state"                         float32[7]  6 joints + 1 gripper (meters/rad)
    "observation.images.wrist_left"             uint8[H,W,3]
    "observation.images.wrist_right"            uint8[H,W,3]  (None if not connected)
    "observation.images.overhead"               uint8[H,W,3]  (None if not connected)

Action keys:
    "action"                                    float32[7]  6 joints + 1 gripper

Current state (single arm):
    - Right arm only (left arm hardware not yet connected)
    - wrist_left camera only (wrist_right not yet connected)
    - Overhead camera TBD

Usage:
    from twinnexus_robot import TwinNexusRobot, TwinNexusRobotConfig

    config = TwinNexusRobotConfig()
    with TwinNexusRobot(config) as robot:
        obs = robot.get_observation()
        action = {"action": obs["observation.state"]}  # hold position
        robot.send_action(action)

    # Or with LeRobot record script:
    # python -m lerobot.record --robot-path path/to/config.yaml
"""

import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from lerobot.robots.robot import Robot
from lerobot.robots.config import RobotConfig

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
IMG_H = 720
IMG_W = 1280
IMG_C = 3

# ── Control parameters ────────────────────────────────────────────────────────
LOOKAHEAD_NS   = 200_000_000   # 200ms rolling horizon for admittance controller
MAX_DELTA_RAD  = 0.01          # max joint delta per send_action call (rad)
MAX_DELTA_M    = 0.001         # max gripper delta per send_action call (m → mm inside)


@dataclass
class TwinNexusRobotConfig(RobotConfig):
    """
    Configuration for the TwinNexusRobot.
    All parameters have sensible defaults for the current single-arm setup.
    """
    # Robot identity
    id: str = "twinnexus_right"

    # ROS 2 topics
    joint_states_topic:     str = "/joint_states"
    gripper_state_topic:    str = "/right_arm/wsg32_node/joint_state"
    gripper_cmd_topic:      str = "/right_arm/wsg32_node/cmd_pos"
    admittance_cmd_topic:   str = "/admittance_controller/joint_references"

    # Camera topics
    wrist_left_topic:   str = "/wrist_left/realsense2_camera_node/color/image_raw"
    wrist_right_topic:  str = ""   # not connected yet
    overhead_topic:     str = ""   # not assigned yet

    # Safety
    max_delta_rad:  float = MAX_DELTA_RAD
    max_delta_m:    float = MAX_DELTA_M

    # LeRobot base class fields
    calibration_dir: Path | None = None


class TwinNexusRobot(Robot):
    """
    LeRobot Robot implementation for the TwinNexus UR5e + WSG32 + D415 platform.

    Implements the full lerobot.robots.robot.Robot interface:
        connect / disconnect
        get_observation → RobotObservation dict
        send_action     ← RobotAction dict
        calibrate       (no-op — UR5e handles its own calibration)
        configure       (no-op — admittance controller is configured separately)
    """

    config_class = TwinNexusRobotConfig
    name = "twinnexus"

    def __init__(self, config: TwinNexusRobotConfig):
        super().__init__(config)
        self.config = config

        # ── State cache — written by ROS2 callbacks, read by get_observation ──
        self._lock = threading.Lock()
        self._joint_pos:    np.ndarray | None = None   # float32[6] radians
        self._gripper_pos:  float | None = None         # meters
        self._last_pub_joints:   list[float] | None = None  # for rate limiting
        self._last_pub_gripper:  float | None = None

        # Camera frames — one per camera
        self._frames: dict[str, np.ndarray | None] = {
            "wrist_left":  None,
            "wrist_right": None,
            "overhead":    None,
        }

        # ── ROS2 internals ────────────────────────────────────────────────────
        self._ros_node:    Node | None = None
        self._executor:    MultiThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._connected = False

    # ── LeRobot interface: features ───────────────────────────────────────────

    @property
    def observation_features(self) -> dict[str, Any]:
        """
        Describes the shape/type of every key returned by get_observation().
        LeRobot uses this to set up the dataset schema.
        """
        feats: dict[str, Any] = {
            # 6 UR5e joints (rad) + 1 gripper (m) = 7-dim state vector
            "observation.state": (7,),
        }
        # Only register camera keys that are actually configured
        if self.config.wrist_left_topic:
            feats["observation.images.wrist_left"] = (IMG_H, IMG_W, IMG_C)
        if self.config.wrist_right_topic:
            feats["observation.images.wrist_right"] = (IMG_H, IMG_W, IMG_C)
        if self.config.overhead_topic:
            feats["observation.images.overhead"] = (IMG_H, IMG_W, IMG_C)
        return feats

    @property
    def action_features(self) -> dict[str, Any]:
        """
        Describes the shape/type of the action dict passed to send_action().
        7-dim: 6 UR5e joint angles (rad) + 1 gripper width (m).
        """
        return {"action": (7,)}

    # ── LeRobot interface: connection ─────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            logger.warning(f"{self} already connected.")
            return

        # Init ROS2 if not already running
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = Node("twinnexus_robot")

        # ── Subscribers ───────────────────────────────────────────────────────
        self._ros_node.create_subscription(
            JointState,
            self.config.joint_states_topic,
            self._cb_joint_states,
            1,   # depth 1 — always latest, never queue
        )
        self._ros_node.create_subscription(
            JointState,
            self.config.gripper_state_topic,
            self._cb_gripper_state,
            1,
        )
        if self.config.wrist_left_topic:
            self._ros_node.create_subscription(
                Image,
                self.config.wrist_left_topic,
                lambda msg: self._cb_image(msg, "wrist_left"),
                1,
            )
        if self.config.wrist_right_topic:
            self._ros_node.create_subscription(
                Image,
                self.config.wrist_right_topic,
                lambda msg: self._cb_image(msg, "wrist_right"),
                1,
            )
        if self.config.overhead_topic:
            self._ros_node.create_subscription(
                Image,
                self.config.overhead_topic,
                lambda msg: self._cb_image(msg, "overhead"),
                1,
            )

        # ── Publishers ────────────────────────────────────────────────────────
        self._joint_pub = self._ros_node.create_publisher(
            JointTrajectoryPoint,
            self.config.admittance_cmd_topic,
            1,
        )
        self._gripper_pub = self._ros_node.create_publisher(
            # WSG32 node takes Float32 in mm on cmd_pos
            __import__("std_msgs.msg", fromlist=["Float32"]).Float32,
            self.config.gripper_cmd_topic,
            1,
        )

        # ── Spin in background thread ─────────────────────────────────────────
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._ros_node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
            name="twinnexus_ros_spin",
        )
        self._spin_thread.start()

        # Wait for first joint state to arrive (up to 10s)
        logger.info(f"{self} waiting for first joint state on {self.config.joint_states_topic} ...")
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self._lock:
                if self._joint_pos is not None:
                    break
            time.sleep(0.05)
        else:
            raise TimeoutError(
                f"{self} timed out waiting for joint states. "
                f"Is the UR5e driver running? Check: ros2 topic hz {self.config.joint_states_topic}"
            )

        self._connected = True
        logger.info(f"{self} connected. Joint states: {self._joint_pos.tolist()}")

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._connected = False
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
        return True   # UR5e handles its own calibration

    def calibrate(self) -> None:
        pass   # no-op

    def configure(self) -> None:
        pass   # admittance controller configured via launch file

    # ── LeRobot interface: observation ────────────────────────────────────────

    def get_observation(self) -> dict[str, Any]:
        """
        Returns the current robot state as a flat dict.
        Reads from the cache — never blocks on ROS 2.

        Raises RuntimeError if called before connect() or if joint states
        have not arrived yet.
        """
        if not self._connected:
            raise RuntimeError(f"{self} is not connected. Call connect() first.")

        with self._lock:
            if self._joint_pos is None:
                raise RuntimeError(f"{self} has not received joint states yet.")

            # ── State vector: 6 joints (rad) + 1 gripper (m) ─────────────────
            gripper = self._gripper_pos if self._gripper_pos is not None else 0.0
            state = np.append(self._joint_pos, gripper).astype(np.float32)

            obs: dict[str, Any] = {"observation.state": state}

            # ── Camera frames ─────────────────────────────────────────────────
            for cam_key, frame in self._frames.items():
                topic_attr = f"{cam_key}_topic"
                if getattr(self.config, topic_attr, ""):
                    obs[f"observation.images.{cam_key}"] = (
                        frame.copy() if frame is not None
                        else np.zeros((IMG_H, IMG_W, IMG_C), dtype=np.uint8)
                    )

        return obs

    # ── LeRobot interface: action ─────────────────────────────────────────────

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        Send a 7-dim action to the robot.
        action["action"]: float32[7] — first 6 are joint angles (rad), last is gripper width (m).

        Rate-limits the target signal to prevent velocity faults on the UR5e.
        Returns the action actually sent (after rate limiting).
        """
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        raw = np.asarray(action["action"], dtype=np.float32)
        if raw.shape != (7,):
            raise ValueError(f"Expected action shape (7,), got {raw.shape}")

        raw_joints  = raw[:6].tolist()
        raw_gripper = float(raw[6])

        # ── Rate-limit joints ─────────────────────────────────────────────────
        if self._last_pub_joints is None:
            with self._lock:
                self._last_pub_joints = (
                    self._joint_pos.tolist() if self._joint_pos is not None
                    else [0.0] * 6
                )

        safe_joints = []
        for prev, new in zip(self._last_pub_joints, raw_joints):
            delta = new - prev
            delta = max(-self.config.max_delta_rad, min(self.config.max_delta_rad, delta))
            safe_joints.append(prev + delta)
        self._last_pub_joints = safe_joints

        # ── Publish joint reference to admittance controller ──────────────────
        pt = JointTrajectoryPoint()
        pt.positions  = safe_joints
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=LOOKAHEAD_NS)
        self._joint_pub.publish(pt)

        # ── Rate-limit and publish gripper ────────────────────────────────────
        if self._last_pub_gripper is None:
            self._last_pub_gripper = raw_gripper

        g_delta = raw_gripper - self._last_pub_gripper
        g_delta = max(-self.config.max_delta_m, min(self.config.max_delta_m, g_delta))
        safe_gripper = self._last_pub_gripper + g_delta
        self._last_pub_gripper = safe_gripper

        from std_msgs.msg import Float32 as F32
        g_msg = F32()
        g_msg.data = float(safe_gripper * 1000.0)   # m → mm for WSG32 node
        self._gripper_pub.publish(g_msg)

        # ── Return actual sent action ─────────────────────────────────────────
        sent = np.array(safe_joints + [safe_gripper], dtype=np.float32)
        return {"action": sent}

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState) -> None:
        """Reorder and cache UR5e joint positions."""
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            pos = np.array(
                [name_to_pos[n] for n in UR5E_JOINTS],
                dtype=np.float32
            )
            with self._lock:
                self._joint_pos = pos
        except KeyError:
            pass   # partial message, skip

    def _cb_gripper_state(self, msg: JointState) -> None:
        """Cache WSG32 gripper width (JointState position[0] is in meters)."""
        if msg.position:
            with self._lock:
                self._gripper_pos = float(msg.position[0])

    def _cb_image(self, msg: Image, cam_key: str) -> None:
        """Convert ROS Image to numpy uint8 HWC array and cache it."""
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
            with self._lock:
                self._frames[cam_key] = frame
        except Exception:
            pass   # malformed message, keep previous frame
