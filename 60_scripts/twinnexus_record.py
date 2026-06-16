#!/usr/bin/env python3
"""
twinnexus_record.py
-------------------
Custom LeRobot data collection script for TwinNexus.
Bypasses lerobot_record.py's teleoperator requirement.
Uses TwinNexusRobot directly — GELLO teleoperation runs via ROS 2 (spawnctrl).

Usage:
    vla_env
    python ~/TwinNexus-Admittance-Platform/twinnexus_record.py \
      --repo-id PhillippGery/pick_place_001 \
      --num-episodes 20 \
      --episode-time-s 30 \
      --reset-time-s 15 \
      --task "pick and place red cube"

Controls during recording:
    Enter       → confirm ready / start next episode
    Ctrl+C      → stop recording (saves completed episodes)
"""

import argparse
import time
import sys
import logging
import os

import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.configs.video import VideoEncoderConfig
import subprocess
import threading



logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def build_features(robot) -> dict:
    """Build LeRobot feature spec from TwinNexusRobot observation/action features."""
    features = {}

    # State + action
    obs_feats = robot.observation_features
    act_feats = robot.action_features

    state_shape = obs_feats["observation.state"]
    features["observation.state"] = {
        "dtype": "float32",
        "shape": list(state_shape),
        "names": None,
    }

    action_shape = act_feats["action"]
    features["action"] = {
        "dtype": "float32",
        "shape": list(action_shape),
        "names": None,
    }

    # Cameras
    for key, shape in obs_feats.items():
        if "images" in key:
            features[key] = {
                "dtype": "video",
                "shape": list(shape),
                "names": ["height", "width", "channel"],
            }

    return features


def obs_to_frame(obs: dict, action: np.ndarray, task: str) -> dict:
    """Convert TwinNexusRobot observation + action to LeRobot frame dict."""
    frame = {"task": task}

    # State
    state = obs["observation.state"]
    if isinstance(state, np.ndarray):
        frame["observation.state"] = torch.from_numpy(state).float()
    else:
        frame["observation.state"] = torch.tensor(state, dtype=torch.float32)

    # Action
    if isinstance(action, np.ndarray):
        frame["action"] = torch.from_numpy(action).float()
    else:
        frame["action"] = torch.tensor(action, dtype=torch.float32)

    # Images — LeRobot expects uint8 tensors [H, W, C]
    for key, val in obs.items():
        if "images" in key:
            if isinstance(val, np.ndarray):
                frame[key] = torch.from_numpy(val)
            else:
                frame[key] = val

    return frame



def main():
    parser = argparse.ArgumentParser(description="TwinNexus LeRobot data recorder")
    parser.add_argument("--repo-id",       required=True,  help="HuggingFace repo ID e.g. PhillippGery/pick_place_001")
    parser.add_argument("--task",          required=True,  help="Task description string")
    parser.add_argument("--num-episodes",  type=int, default=20)
    parser.add_argument("--episode-time-s",type=float, default=30.0)
    parser.add_argument("--reset-time-s",  type=float, default=0.0)
    parser.add_argument("--fps",           type=int, default=24)
    parser.add_argument("--root",          default=None,   help="Local dataset root directory")
    parser.add_argument("--push-to-hub",   action="store_true")
    parser.add_argument("--warmup-time-s", type=float, default=2.0, help="Warmup before each episode")
    args = parser.parse_args()


    # ── Import LeRobot dataset ────────────────────────────────────────────────
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # ── Import and connect TwinNexusRobot ─────────────────────────────────────
    from lerobot.robots.twinnexus import TwinNexusRobot, TwinNexusRobotConfig

    config = TwinNexusRobotConfig()
    robot  = TwinNexusRobot(config)

    print("\n" + "="*60)
    print("  TwinNexus LeRobot Recorder")
    print("="*60)
    print(f"  Repo:     {args.repo_id}")
    print(f"  Task:     {args.task}")
    print(f"  Episodes: {args.num_episodes}")
    print(f"  Duration: {args.episode_time_s}s per episode")
    print(f"  Reset:    {args.reset_time_s}s between episodes")
    print("="*60)
    print("\nMake sure:")
    print("  1. boot_hw is running and Play is pressed")
    print("  2. spawnctrl is running")
    print("  3. GELLO hold has released (deltas are small)")
    print()
    input("Press Enter to connect robot and start recording...")

    # ── Connect ───────────────────────────────────────────────────────────────
    print("Connecting to TwinNexusRobot ...")
    robot.connect()
    print("Connected.")

    # ── Build features from robot ─────────────────────────────────────────────
    features = build_features(robot)
    print(f"Features: {list(features.keys())}")

    # ── Create dataset ────────────────────────────────────────────────────────
    print(f"\nCreating dataset: {args.repo_id}")

    # dataset = LeRobotDataset.create(
    #     repo_id=args.repo_id,
    #     fps=args.fps,
    #     features=features,
    #     root=os.path.join(args.root, args.repo_id) if args.root else None,
    #     robot_type="twinnexus",
    #     image_writer_threads=8,      # ← async image writing
    #     image_writer_processes=0,    # ← use threads not processes
    # )

    

    # dataset = LeRobotDataset.create(
    #     repo_id=args.repo_id,
    #     fps=args.fps,
    #     features=features,
    #     root=os.path.join(args.root, args.repo_id) if args.root else None,
    #     robot_type="twinnexus",
    #     image_writer_threads=4,
    # )

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=os.path.join(args.root, args.repo_id) if args.root else None,
        robot_type="twinnexus",
        streaming_encoding=True,
        encoder_threads=4,
        encoder_queue_maxsize=200,
    )


    print(f"Dataset created.")

    # ── Recording loop ────────────────────────────────────────────────────────
    episode_idx = 0
    try:
        while episode_idx < args.num_episodes:
            print(f"\n{'='*60}")
            print(f"  Episode {episode_idx + 1} / {args.num_episodes}")
            print(f"{'='*60}")
            input(f"  Position GELLO at start pose. Press Enter to record...")

            # Warmup — discard first few frames
            print(f"  Warming up {args.warmup_time_s}s ...")
            warmup_end = time.perf_counter() + args.warmup_time_s
            while time.perf_counter() < warmup_end:
                robot.get_observation()
                time.sleep(1.0 / args.fps)

            stop_episode = threading.Event()
            threading.Thread(target=lambda: (input(), stop_episode.set()), daemon=True).start()
            print("  Press Enter to stop episode early...")

            # Record episode
            print(f"  Recording {args.episode_time_s}s ... (move GELLO now)")
            frame_count  = 0
            episode_start = time.perf_counter()
            dt            = 1.0 / args.fps

            # In the record loop — replace the entire loop with:
            last_frame_hash = None
            while time.perf_counter() - episode_start < args.episode_time_s:
                obs = robot.get_observation()
                
                # Only record when we have a new camera frame
                current_hash = hash(obs["observation.images.wrist_right"].tobytes()[:100])
                if current_hash == last_frame_hash:
                    continue  # same frame, skip
                last_frame_hash = current_hash

                if stop_episode.is_set():
                    print("\n  Episode stopped early.")
                    break
                
                action = obs["observation.state"]
                frame  = obs_to_frame(obs, action, args.task)
                dataset.add_frame(frame)
                frame_count += 1

                #print time every second
                if frame_count % args.fps == 0:
                    print(f"Time: {time.perf_counter() - episode_start:.1f}s, Frames: {frame_count}")
                    
                    
                    
            elapsed_s = time.perf_counter() - episode_start
            print(f"  Episode done: {frame_count} frames in {elapsed_s:.2f}s "
                f"({frame_count/elapsed_s:.1f} fps)")
                
            print("  → Returning to home position ...")
                
            # Auto go_home
            subprocess.Popen(
                'source /opt/ros/jazzy/setup.bash && '
                'source ~/TwinNexus-Admittance-Platform/10_src/install/setup.bash && '
                'python3 ~/TwinNexus-Admittance-Platform/10_src/src/bimanual_ur5e_bringup/scripts/return_home.py',
                shell=True,
                executable='/bin/bash'
            )
            #wait 0.5s for the go_home command to execute before starting the reset timer
            time.sleep(0.5)  # wait a bit for the go_home command to execute

            # Save episode
            print("  Saving episode ...")
            dataset.save_episode()
            print(f"  Episode {episode_idx + 1} saved. "
                  f"Total episodes: {dataset.num_episodes}")

            episode_idx += 1

            # Reset phase (skip after last episode)
            if episode_idx < args.num_episodes:
                print(f"\n  Reset phase ({args.reset_time_s}s)")
                
                
                print("  → Reset object to start position")
                reset_end = time.perf_counter() + args.reset_time_s
                while time.perf_counter() < reset_end:
                    remaining = reset_end - time.perf_counter()
                    print(f"\r  Time remaining: {remaining:.0f}s  ", end="", flush=True)
                    time.sleep(0.5)
                print()

    except KeyboardInterrupt:
        print(f"\n\nRecording interrupted. {episode_idx} episodes saved.")
        if dataset.num_episodes > 0:
            # Clear any partial episode buffer
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
        print(f"Dataset saved locally.")

        if args.push_to_hub:
            print("Pushing to HuggingFace Hub ...")
            dataset.push_to_hub()
            print(f"Dataset available at: https://huggingface.co/datasets/{args.repo_id}")
    else:
        print("No episodes recorded.")

    print("\nDone.")


if __name__ == "__main__":
    main()