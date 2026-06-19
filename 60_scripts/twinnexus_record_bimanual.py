#!/usr/bin/env python3
"""
twinnexus_record_bimanual.py
----------------------------
LeRobot data collection for TwinNexus — single-arm and bimanual modes.

When --bimanual is NOT set: uses TwinNexusRobot (existing single-arm class),
identical to twinnexus_record.py.

When --bimanual IS set: uses BimanualTwinNexusRobot (defined below).
  observation.state : (14,) = [right_joints(6), right_gripper(1),
                                left_joints(6),  left_gripper(1)]
  action            : (14,) same layout
  cameras           : wrist_right, wrist_left, overhead (all three active)

Usage:
    vla_env
    # Single arm (identical to twinnexus_record.py):
    python ~/TwinNexus-Admittance-Platform/60_scripts/twinnexus_record_bimanual.py \\
        --repo-id PhillippGery/task_001 --task "pick cube" --num-episodes 20

    # Bimanual:
    python ~/TwinNexus-Admittance-Platform/60_scripts/twinnexus_record_bimanual.py \\
        --repo-id PhillippGery/task_bimanual_001 --task "hand off cube" \\
        --num-episodes 20 --bimanual

Controls:
    Enter    → confirm ready / stop episode early
    Ctrl+C   → stop (saves completed episodes)
"""

import argparse
import logging
import os
import select
import subprocess
import sys
import threading
import time

import numpy as np
import pyrealsense2 as rs
import rclpy
import torch
from lerobot.configs.video import VideoEncoderConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────

IMG_H, IMG_W, IMG_C = 480, 640, 3

_UR5E_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

_RETURN_HOME_CMD = (
    'source /opt/ros/jazzy/setup.bash && '
    'source ~/TwinNexus-Admittance-Platform/10_src/install/setup.bash && '
    'python3 ~/TwinNexus-Admittance-Platform/10_src/src/'
    'bimanual_ur5e_bringup/scripts/return_home.py'
)

# ── Bimanual robot ─────────────────────────────────────────────────────────────

class BimanualTwinNexusRobot:
    """
    Minimal LeRobot-compatible robot for bimanual recording.

    Two UR5e arms + WSG32 grippers via ROS2, three RealSense cameras via
    pyrealsense2 direct.

    observation.state : (14,) [right_joints(6), right_gripper(1),
                               left_joints(6),  left_gripper(1)]
    action            : (14,) same layout
    """

    # Joint names as published by the bimanual driver (tf_prefix applied)
    _RIGHT_JOINTS = [f"right_arm_{n}" for n in _UR5E_JOINTS]
    _LEFT_JOINTS  = [f"left_arm_{n}"  for n in _UR5E_JOINTS]

    # Camera serials
    _CAM_SERIALS = {
        "wrist_right": "151422060684",
        "wrist_left":  "151322062583",
        "overhead":    "146222254752",
    }

    # ROS topics
    _RIGHT_JOINTS_TOPIC  = "/right_arm/joint_states"
    _RIGHT_GRIPPER_TOPIC = "/right_arm/wsg32_node/joint_state"
    _LEFT_JOINTS_TOPIC   = "/left_arm/joint_states"
    _LEFT_GRIPPER_TOPIC  = "/left_arm/wsg32_node/joint_state"
    _RIGHT_BRIDGE_TOPIC  = "/right_arm/twinnexus_bridge_right/target_joints"
    _LEFT_BRIDGE_TOPIC   = "/left_arm/twinnexus_bridge_left/target_joints"

    def __init__(self):
        self._lock = threading.Lock()

        # Robot state
        self._right_joints:  np.ndarray | None = None
        self._right_gripper: float | None       = None
        self._left_joints:   np.ndarray | None  = None
        self._left_gripper:  float | None        = None

        # Camera frames
        self._frames = {k: None for k in self._CAM_SERIALS}
        self._rs_pipelines = {k: None for k in self._CAM_SERIALS}
        self._camera_running = False
        self._camera_thread: threading.Thread | None = None

        # ROS2
        self._ros_node:    Node | None                = None
        self._executor:    SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None    = None
        self._spin_stop    = threading.Event()
        self._connected    = False

    @property
    def observation_features(self) -> dict:
        feats = {"observation.state": (14,)}
        for cam in self._CAM_SERIALS:
            feats[f"observation.images.{cam}"] = (IMG_H, IMG_W, IMG_C)
        return feats

    @property
    def action_features(self) -> dict:
        return {"action": (14,)}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return

        # ── ROS2 first — let DDS discovery settle before cameras spike USB ──
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = Node("twinnexus_bimanual_robot")

        self._ros_node.create_subscription(
            JointState, self._RIGHT_JOINTS_TOPIC,  self._cb_right_joints,  1
        )
        self._ros_node.create_subscription(
            JointState, self._RIGHT_GRIPPER_TOPIC, self._cb_right_gripper, 1
        )
        self._ros_node.create_subscription(
            JointState, self._LEFT_JOINTS_TOPIC,   self._cb_left_joints,   1
        )
        self._ros_node.create_subscription(
            JointState, self._LEFT_GRIPPER_TOPIC,  self._cb_left_gripper,  1
        )

        _go_home_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._right_target_pub = self._ros_node.create_publisher(
            JointState, self._RIGHT_BRIDGE_TOPIC, 1
        )
        self._left_target_pub = self._ros_node.create_publisher(
            JointState, self._LEFT_BRIDGE_TOPIC, 1
        )

        time.sleep(1.0)  # Let ROS2 settle before starting spin thread and cameras
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros_node)
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="bimanual_ros_spin"
        )
        self._spin_thread.start()

        # ── Wait for both arms ────────────────────────────────────────────────
        logger.info("BimanualRobot: waiting for joint states from both arms ...")
        deadline = time.time() + 15.0
        while time.time() < deadline:
            with self._lock:
                if self._right_joints is not None and self._left_joints is not None:
                    break
            time.sleep(0.05)
        else:
            with self._lock:
                missing = []
                if self._right_joints is None:
                    missing.append("right_arm")
                if self._left_joints is None:
                    missing.append("left_arm")
            raise TimeoutError(
                f"BimanualRobot: timed out waiting for joint states from: {missing}. "
                "Is boot_hw_bimanual running with Play pressed on both pendants?"
            )

        # ── Cameras — started after robot confirmed stable ────────────────────
        # Staggered startup avoids a simultaneous USB bandwidth spike that can
        # starve the ur_robot_driver RTDE thread and drop external control.
        for cam, serial in self._CAM_SERIALS.items():
            try:
                pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_device(serial)
                cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, 30)
                pipeline.start(cfg)
                self._rs_pipelines[cam] = pipeline
                logger.info(f"BimanualRobot: camera {cam} ({serial}) started.")
                time.sleep(0.3)   # stagger USB negotiation between cameras
            except Exception as e:
                logger.warning(f"BimanualRobot: camera {cam} ({serial}) failed: {e}")

        self._camera_running = True
        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name="bimanual_cameras"
        )
        self._camera_thread.start()

        # Wait for first camera frames
        active_cams = [k for k, p in self._rs_pipelines.items() if p is not None]
        if active_cams:
            deadline = time.time() + 8.0
            while time.time() < deadline:
                with self._lock:
                    if all(self._frames[k] is not None for k in active_cams):
                        break
                time.sleep(0.05)
            else:
                logger.warning("BimanualRobot: some cameras did not produce frames within 8s.")

        self._connected = True
        with self._lock:
            r = self._right_joints.tolist()
            l = self._left_joints.tolist()
        logger.info(
            f"BimanualRobot connected.\n"
            f"  Right: {[f'{v:.3f}' for v in r]}\n"
            f"  Left:  {[f'{v:.3f}' for v in l]}\n"
            f"  Cameras: {[k for k in active_cams if self._frames[k] is not None]}"
        )

    def get_observation(self) -> dict:
        if not self._connected:
            raise RuntimeError("BimanualRobot is not connected.")
        with self._lock:
            rj = self._right_joints.copy() if self._right_joints is not None else np.zeros(6, np.float32)
            rg = self._right_gripper if self._right_gripper is not None else 0.0
            lj = self._left_joints.copy()  if self._left_joints  is not None else np.zeros(6, np.float32)
            lg = self._left_gripper  if self._left_gripper  is not None else 0.0
            frames_snap = {k: (v.copy() if v is not None else None) for k, v in self._frames.items()}

        state = np.concatenate([rj, [rg], lj, [lg]]).astype(np.float32)
        obs = {"observation.state": state}
        for cam in self._CAM_SERIALS:
            obs[f"observation.images.{cam}"] = (
                frames_snap[cam] if frames_snap[cam] is not None
                else np.zeros((IMG_H, IMG_W, IMG_C), dtype=np.uint8)
            )
        return obs

    def send_action(self, action: dict) -> dict:
        """Split (14,) action → right bridge (7,) + left bridge (7,)."""
        if not self._connected:
            raise RuntimeError("BimanualRobot is not connected.")
        raw = np.asarray(action["action"], dtype=np.float32)
        if raw.shape != (14,):
            raise ValueError(f"Expected action shape (14,), got {raw.shape}")
        right_msg = JointState()
        right_msg.position = raw[:7].tolist()
        self._right_target_pub.publish(right_msg)
        left_msg = JointState()
        left_msg.position = raw[7:].tolist()
        self._left_target_pub.publish(left_msg)
        return {"action": raw}

    def pause_for_save(self) -> None:
        self._camera_running = False
        if self._camera_thread and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=2.0)
        self._spin_stop.set()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

    def resume_from_save(self) -> None:
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="bimanual_ros_spin"
        )
        self._spin_thread.start()
        self._camera_running = True
        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name="bimanual_cameras"
        )
        self._camera_thread.start()

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._connected = False
        self._camera_running = False
        if self._camera_thread:
            self._camera_thread.join(timeout=2.0)
        for cam, pipeline in self._rs_pipelines.items():
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
                self._rs_pipelines[cam] = None
        self._spin_stop.set()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if self._executor:
            self._executor.shutdown(timeout_sec=1.0)
        if self._ros_node:
            self._ros_node.destroy_node()
        self._ros_node = None
        self._executor = None
        logger.info("BimanualRobot disconnected.")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _spin_loop(self) -> None:
        while not self._spin_stop.is_set():
            self._executor.spin_once(timeout_sec=0.01)

    def _cb_right_joints(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        try:
            pos = np.array([n2p[n] for n in self._RIGHT_JOINTS], dtype=np.float32)
            with self._lock:
                self._right_joints = pos
        except KeyError:
            pass

    def _cb_right_gripper(self, msg: JointState) -> None:
        if msg.position:
            with self._lock:
                self._right_gripper = float(msg.position[0])

    def _cb_left_joints(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        try:
            pos = np.array([n2p[n] for n in self._LEFT_JOINTS], dtype=np.float32)
            with self._lock:
                self._left_joints = pos
        except KeyError:
            pass

    def _cb_left_gripper(self, msg: JointState) -> None:
        if msg.position:
            with self._lock:
                self._left_gripper = float(msg.position[0])

    def _camera_loop(self) -> None:
        while self._camera_running:
            time.sleep(0)
            for cam, pipeline in self._rs_pipelines.items():
                if pipeline is None:
                    continue
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=50)
                    color  = frames.get_color_frame()
                    if color:
                        frame = np.asarray(color.get_data(), dtype=np.uint8)
                        with self._lock:
                            self._frames[cam] = frame
                except RuntimeError:
                    pass
                except Exception as e:
                    logger.warning(f"BimanualRobot camera {cam} error: {e}")


# ── Dataset helpers (identical to twinnexus_record.py) ────────────────────────

def build_features(robot) -> dict:
    features = {}
    obs_feats = robot.observation_features
    act_feats = robot.action_features

    features["observation.state"] = {
        "dtype": "float32",
        "shape": list(obs_feats["observation.state"]),
        "names": None,
    }
    features["action"] = {
        "dtype": "float32",
        "shape": list(act_feats["action"]),
        "names": None,
    }
    for key, shape in obs_feats.items():
        if "images" in key:
            features[key] = {
                "dtype": "video",
                "shape": list(shape),
                "names": ["height", "width", "channel"],
            }
    return features


def obs_to_frame(obs: dict, action: np.ndarray, task: str) -> dict:
    frame = {"task": task}
    state = obs["observation.state"]
    frame["observation.state"] = (
        torch.from_numpy(state).float() if isinstance(state, np.ndarray)
        else torch.tensor(state, dtype=torch.float32)
    )
    frame["action"] = (
        torch.from_numpy(action).float() if isinstance(action, np.ndarray)
        else torch.tensor(action, dtype=torch.float32)
    )
    for key, val in obs.items():
        if "images" in key:
            frame[key] = torch.from_numpy(val) if isinstance(val, np.ndarray) else val
    return frame


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="TwinNexus LeRobot data recorder (bimanual-capable)")
    parser.add_argument("--repo-id",        required=True)
    parser.add_argument("--task",           required=True)
    parser.add_argument("--num-episodes",   type=int,   default=20)
    parser.add_argument("--episode-time-s", type=float, default=30.0)
    parser.add_argument("--reset-time-s",   type=float, default=0.0)
    parser.add_argument("--fps",            type=int,   default=24)
    parser.add_argument("--root",           default=None)
    parser.add_argument("--push-to-hub",    action="store_true")
    parser.add_argument("--warmup-time-s",  type=float, default=2.0)
    parser.add_argument("--bimanual",       action="store_true",
                        help="Record both arms (14-dim state/action, 3 cameras).")
    args = parser.parse_args()

    # ── Build robot ───────────────────────────────────────────────────────────
    if args.bimanual:
        robot = BimanualTwinNexusRobot()
        robot_type = "twinnexus_bimanual"
    else:
        from lerobot.robots.twinnexus import TwinNexusRobot, TwinNexusRobotConfig
        robot = TwinNexusRobot(TwinNexusRobotConfig())
        robot_type = "twinnexus"

    print("\n" + "=" * 60)
    print("  TwinNexus LeRobot Recorder")
    print("=" * 60)
    print(f"  Mode:     {'BIMANUAL (14-dim)' if args.bimanual else 'single-arm (7-dim)'}")
    print(f"  Repo:     {args.repo_id}")
    print(f"  Task:     {args.task}")
    print(f"  Episodes: {args.num_episodes}")
    print(f"  Duration: {args.episode_time_s}s per episode")
    print(f"  Reset:    {args.reset_time_s}s between episodes")
    print("=" * 60)
    print("\nMake sure:")
    if args.bimanual:
        print("  1. boot_hw_bimanual is running, Play pressed on BOTH pendants")
        print("  2. spawnctrl_bimanual teleop:=true is running")
        print("  3. GELLO holds have released on BOTH arms")
    else:
        print("  1. boot_hw is running and Play is pressed")
        print("  2. spawnctrl is running")
        print("  3. GELLO hold has released")
    print()
    input("Press Enter to connect robot and start recording...")

    # ── Connect ───────────────────────────────────────────────────────────────
    print("Connecting ...")
    robot.connect()
    print("Connected.")

    features = build_features(robot)
    print(f"Features: {list(features.keys())}")

    # ── Create dataset ────────────────────────────────────────────────────────
    print(f"\nCreating dataset: {args.repo_id}")
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=os.path.join(args.root, args.repo_id) if args.root else None,
        robot_type=robot_type,
        image_writer_threads=8,
        image_writer_processes=0,
        camera_encoder=VideoEncoderConfig(vcodec="h264", preset="fast", g=16),
    )
    print("Dataset created.")

    # ── Recording loop (identical for single-arm and bimanual) ───────────────
    episode_idx = 0
    try:
        while episode_idx < args.num_episodes:
            print(f"\n{'=' * 60}")
            print(f"  Episode {episode_idx + 1} / {args.num_episodes}")
            print(f"{'=' * 60}")
            input("  Position GELLO(s) at start pose. Press Enter to record...")

            # Warmup
            print(f"  Warming up {args.warmup_time_s}s ...")
            warmup_end = time.perf_counter() + args.warmup_time_s
            while time.perf_counter() < warmup_end:
                robot.get_observation()
                time.sleep(1.0 / args.fps)

            print("  Press Enter to stop episode early...")
            print(f"  Recording {args.episode_time_s}s ...")

            frame_count   = 0
            slow_frames   = 0
            episode_start = time.perf_counter()
            dt            = 1.0 / args.fps
            next_frame_t  = episode_start

            while True:
                now     = time.perf_counter()
                elapsed = now - episode_start
                if elapsed >= args.episode_time_s:
                    break
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    print("\n  Episode stopped early.")
                    break

                sleep_s = next_frame_t - now
                if sleep_s > 0:
                    time.sleep(sleep_s)

                t_work = time.perf_counter()
                obs    = robot.get_observation()
                action = obs["observation.state"]
                frame  = obs_to_frame(obs, action, args.task)
                dataset.add_frame(frame)
                work_ms = (time.perf_counter() - t_work) * 1000
                if work_ms > dt * 1000 * 0.8:
                    slow_frames += 1

                frame_count  += 1
                next_frame_t += dt

                if frame_count % args.fps == 0:
                    actual_fps = frame_count / (time.perf_counter() - episode_start)
                    print(f"  Time: {time.perf_counter() - episode_start:.1f}s  "
                          f"Frames: {frame_count}  fps: {actual_fps:.1f}  "
                          f"slow: {slow_frames}")

            elapsed_s = time.perf_counter() - episode_start
            print(f"  Episode done: {frame_count} frames in {elapsed_s:.2f}s "
                  f"({frame_count / elapsed_s:.1f} fps)")

            print("  → Returning to home position ...")
            subprocess.Popen(_RETURN_HOME_CMD, shell=True, executable='/bin/bash')
            time.sleep(2.5)

            robot.pause_for_save()

            t_save = time.perf_counter()
            print("  Saving episode ...")
            dataset.save_episode()
            print(f"  Episode {episode_idx + 1} saved  "
                  f"(total: {dataset.num_episodes})  "
                  f"save time: {time.perf_counter() - t_save:.2f}s")

            robot.resume_from_save()
            episode_idx += 1

            if episode_idx < args.num_episodes:
                print(f"\n  Reset phase ({args.reset_time_s}s) — reset object to start position")
                reset_end = time.perf_counter() + args.reset_time_s
                while time.perf_counter() < reset_end:
                    remaining = reset_end - time.perf_counter()
                    print(f"\r  Time remaining: {remaining:.0f}s  ", end="", flush=True)
                    time.sleep(0.5)
                print()

    except KeyboardInterrupt:
        print(f"\n\nRecording interrupted. {episode_idx} episodes saved.")
        if dataset.num_episodes > 0:
            try:
                dataset.clear_episode_buffer()
            except Exception:
                pass

    finally:
        robot.disconnect()
        print("Robot disconnected.")

    # ── Finalize ──────────────────────────────────────────────────────────────
    if dataset.num_episodes > 0:
        print(f"\nFinalizing dataset with {dataset.num_episodes} episodes ...")
        dataset.finalize()
        print("Dataset saved locally.")
        if args.push_to_hub:
            print("Pushing to HuggingFace Hub ...")
            dataset.push_to_hub()
            print(f"Dataset available at: https://huggingface.co/datasets/{args.repo_id}")
    else:
        print("No episodes recorded.")

    print("\nDone.")


if __name__ == "__main__":
    main()
