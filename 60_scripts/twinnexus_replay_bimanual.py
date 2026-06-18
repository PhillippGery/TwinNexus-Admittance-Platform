#!/usr/bin/env python3
"""
twinnexus_replay_bimanual.py
----------------------------
Replay a recorded bimanual episode from a LeRobot dataset.

Loads the action column (14,) = [right_joints(6), right_gripper(1),
left_joints(6), left_gripper(1)] from a parquet file and sends each frame
to both arms via the TwinNexus bridges at the recorded fps.

Safety: each action is compared to the current robot state before sending.
If any joint delta exceeds --max-delta-rad (default 0.15 rad), the action
is skipped and the robot holds its current position for that frame.

Usage:
    vla_env
    python3 ~/TwinNexus-Admittance-Platform/60_scripts/twinnexus_replay_bimanual.py \\
        --dataset ~/TwinNexus-Admittance-Platform/30_data/bimanual_test \\
        --episode 0 \\
        --fps 24

Controls:
    Enter     → start replay / pause-resume
    Ctrl+C    → stop and go home
"""

import argparse
import glob
import importlib.util
import logging
import os
import select
import subprocess
import sys
import time

import numpy as np
import pandas as pd

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── Shared constants ──────────────────────────────────────────────────────────

_RETURN_HOME_CMD = (
    'source /opt/ros/jazzy/setup.bash && '
    'source ~/TwinNexus-Admittance-Platform/10_src/install/setup.bash && '
    'python3 ~/TwinNexus-Admittance-Platform/10_src/src/'
    'bimanual_ur5e_bringup/scripts/return_home.py'
)

# Joint indices within the (14,) state/action vector
# [0:6] right joints, [6] right gripper, [7:13] left joints, [13] left gripper
_RIGHT_JOINT_IDX = slice(0, 6)
_LEFT_JOINT_IDX  = slice(7, 13)


def _load_robot_class():
    """Import BimanualTwinNexusRobot from the recording script (single source of truth)."""
    script = os.path.expanduser(
        "~/TwinNexus-Admittance-Platform/60_scripts/twinnexus_record_bimanual.py"
    )
    spec = importlib.util.spec_from_file_location("twinnexus_record_bimanual", script)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.BimanualTwinNexusRobot


def _load_episode(dataset_path: str, episode_idx: int) -> np.ndarray:
    """
    Load the action sequence for one episode from a LeRobot parquet file.

    Searches {dataset_path}/data/**/episode_{episode_idx:06d}.parquet.
    Returns actions as float32 ndarray of shape (T, 14).
    """
    pattern = os.path.join(
        os.path.expanduser(dataset_path),
        "data", "**", f"episode_{episode_idx:06d}.parquet",
    )
    matches = glob.glob(pattern, recursive=True)
    if not matches:
        raise FileNotFoundError(
            f"No parquet file found for episode {episode_idx} in {dataset_path}.\n"
            f"Searched: {pattern}"
        )

    df = pd.read_parquet(matches[0])

    if "action" not in df.columns:
        raise ValueError(
            f"Parquet file has no 'action' column. Columns: {list(df.columns)}"
        )

    # LeRobot stores array columns as lists; convert to ndarray
    actions = np.array(df["action"].tolist(), dtype=np.float32)

    if actions.ndim != 2 or actions.shape[1] != 14:
        raise ValueError(
            f"Expected action shape (T, 14), got {actions.shape}. "
            "Is this a bimanual dataset?"
        )

    return actions


def _wait_for_enter(prompt: str) -> None:
    """Block until the user presses Enter."""
    input(prompt)


def _enter_pressed() -> bool:
    """Non-blocking check: returns True if Enter is waiting in stdin."""
    return bool(select.select([sys.stdin], [], [], 0)[0])


def _go_home() -> None:
    print("  → Sending go_home ...")
    subprocess.Popen(_RETURN_HOME_CMD, shell=True, executable="/bin/bash")
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="TwinNexus bimanual trajectory replay")
    parser.add_argument(
        "--dataset", required=True,
        help="Path to the LeRobot dataset directory (contains data/, meta/).",
    )
    parser.add_argument(
        "--episode", type=int, default=0,
        help="Episode index to replay (default: 0).",
    )
    parser.add_argument(
        "--fps", type=float, default=24.0,
        help="Replay speed in frames per second (default: 24).",
    )
    parser.add_argument(
        "--max-delta-rad", type=float, default=0.15,
        help="Safety threshold: skip action if any joint delta exceeds this (rad).",
    )
    args = parser.parse_args()

    # ── Load episode ──────────────────────────────────────────────────────────
    print(f"\nLoading episode {args.episode} from {args.dataset} ...")
    actions = _load_episode(args.dataset, args.episode)
    n_frames = len(actions)
    duration = n_frames / args.fps
    print(f"  Loaded {n_frames} frames  ({duration:.1f}s at {args.fps:.0f}fps)")

    # ── Connect robot ─────────────────────────────────────────────────────────
    print("\nImporting BimanualTwinNexusRobot ...")
    BimanualTwinNexusRobot = _load_robot_class()

    robot = BimanualTwinNexusRobot()

    print("\nMake sure:")
    print("  1. boot_hw_bimanual running, Play pressed on BOTH pendants")
    print("  2. spawnctrl_bimanual running (teleop NOT required)")
    print()
    _wait_for_enter("Press Enter to connect ...")

    print("Connecting ...")
    robot.connect()
    print("Connected.")

    # ── Go home before replay ─────────────────────────────────────────────────
    print("\nReturning to home position before replay ...")
    _go_home()
    time.sleep(3.0)   # let robot reach home before we start

    # ── Replay ───────────────────────────────────────────────────────────────
    print(f"\nEpisode {args.episode}:  {n_frames} frames  {duration:.1f}s")
    print(f"Safety threshold:  {args.max_delta_rad:.3f} rad")
    print()
    _wait_for_enter("Press Enter to start replay ...")

    dt          = 1.0 / args.fps
    frame_idx   = 0
    skipped     = 0
    paused      = False
    replay_start = time.perf_counter()
    next_frame_t = replay_start

    print("  Replaying ... (Press Enter to pause/resume, Ctrl+C to stop)\n")

    try:
        while frame_idx < n_frames:

            # ── Pause / resume ────────────────────────────────────────────────
            if _enter_pressed():
                sys.stdin.readline()
                if not paused:
                    paused = True
                    print("\n  [PAUSED] Press Enter to resume ...")
                else:
                    paused = False
                    # Re-sync timer to avoid burst of frames after resume
                    next_frame_t = time.perf_counter()
                    print("  [RESUMED]")

            if paused:
                time.sleep(0.05)
                continue

            # ── Frame timing ──────────────────────────────────────────────────
            now = time.perf_counter()
            sleep_s = next_frame_t - now
            if sleep_s > 0:
                time.sleep(sleep_s)

            # ── Safety check ──────────────────────────────────────────────────
            obs   = robot.get_observation()
            state = obs["observation.state"]   # (14,)
            action = actions[frame_idx]         # (14,)

            right_delta = np.abs(action[_RIGHT_JOINT_IDX] - state[_RIGHT_JOINT_IDX])
            left_delta  = np.abs(action[_LEFT_JOINT_IDX]  - state[_LEFT_JOINT_IDX])
            max_delta   = max(right_delta.max(), left_delta.max())

            if max_delta > args.max_delta_rad:
                skipped += 1
                logger.warning(
                    f"Frame {frame_idx}: max joint delta {max_delta:.3f} rad "
                    f"> {args.max_delta_rad:.3f} rad — skipping."
                )
            else:
                robot.send_action({"action": action})

            # ── Progress ──────────────────────────────────────────────────────
            if frame_idx % int(args.fps) == 0:
                elapsed = time.perf_counter() - replay_start
                print(
                    f"\r  Frame {frame_idx:4d}/{n_frames}  "
                    f"{elapsed:.1f}s/{duration:.1f}s  "
                    f"skipped={skipped}  ",
                    end="", flush=True,
                )

            frame_idx    += 1
            next_frame_t += dt

    except KeyboardInterrupt:
        print(f"\n\nReplay interrupted at frame {frame_idx}/{n_frames}.")

    else:
        print(f"\n\nReplay complete.  {frame_idx} frames sent, {skipped} skipped.")

    # ── Go home after replay ──────────────────────────────────────────────────
    _go_home()

    robot.disconnect()
    print("Robot disconnected.")
    print("\nDone.")


if __name__ == "__main__":
    main()
