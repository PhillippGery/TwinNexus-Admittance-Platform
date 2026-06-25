#!/usr/bin/env python3
"""
convert_twinnexus.py
--------------------
Convert TwinNexus LeRobot v3.0 dataset to openpi-compatible format.

Reads existing parquet + video files recorded with LeRobot 0.5.2
and re-creates the dataset using openpi's bundled LeRobot (0.1.0).

Usage:
    cd ~/openpi

    # Single arm:
    uv run ~/TwinNexus-Admittance-Platform/70_vla/convert_twinnexus.py \
        --src ~/TwinNexus-Admittance-Platform/30_data/pick_place_001 \
        --repo pick_place_001_openpi \
        --task "pick up the screwdriver and place it in the paper box"

    # Bimanual:
    uv run ~/TwinNexus-Admittance-Platform/70_vla/convert_twinnexus.py \
        --src ~/TwinNexus-Admittance-Platform/30_data/bimanual_box_001 \
        --repo bimanual_box_001_openpi \
        --task "pick yeallow Box and place in the red square on the Table" \
        --bimanual

    # Overwrite existing output:
    uv run ... --overwrite
"""

import argparse
import shutil
import pathlib

import cv2
import numpy as np
import pandas as pd

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, HF_LEROBOT_HOME


def parse_args():
    parser = argparse.ArgumentParser(description="Convert TwinNexus dataset to openpi format")
    parser.add_argument("--src",      required=True,  help="Source dataset path (LeRobot v3.0)")
    parser.add_argument("--repo",     required=True,  help="Output repo name (e.g. pick_place_001_openpi)")
    parser.add_argument("--task",     default=None,   help="Task description (overrides dataset task)")
    parser.add_argument("--fps",      type=int, default=24, help="Dataset FPS (default: 24)")
    parser.add_argument("--bimanual", action="store_true",
                        help="Bimanual mode: state/action (14,), 3 cameras (overhead, wrist_right, wrist_left)")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing output dataset")
    return parser.parse_args()


def main():
    args = parse_args()

    src = pathlib.Path(args.src).expanduser().resolve()
    if not src.exists():
        raise FileNotFoundError(f"Source dataset not found: {src}")

    # Resolve task description
    task = args.task
    if task is None:
        tasks_file = src / "meta" / "tasks.jsonl"
        if tasks_file.exists():
            import json
            with open(tasks_file) as f:
                task = json.loads(f.readline())["task"]
            print(f"Using task from dataset: {task}")
        else:
            raise ValueError("No --task provided and no tasks.jsonl found in dataset")

    # Output path
    output_path = HF_LEROBOT_HOME / args.repo
    if output_path.exists():
        if args.overwrite:
            print(f"Removing existing dataset at {output_path}")
            shutil.rmtree(output_path)
        else:
            raise FileExistsError(
                f"Output already exists: {output_path}\n"
                f"Use --overwrite to replace it."
            )

    print(f"Source:   {src}")
    print(f"Output:   {output_path}")
    print(f"Task:     {task}")
    print(f"FPS:      {args.fps}")
    print(f"Bimanual: {args.bimanual}")

    # ── Feature schema ────────────────────────────────────────────────────────
    state_dim  = 14 if args.bimanual else 7
    action_dim = 14 if args.bimanual else 7

    features = {
        "image": {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        },
        "wrist_image": {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        },
        "state": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": ["state"],
        },
        "actions": {
            "dtype": "float32",
            "shape": (action_dim,),
            "names": ["actions"],
        },
    }

    if args.bimanual:
        features["wrist_image_left"] = {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        }

    robot_type = "twinnexus_bimanual" if args.bimanual else "ur5e"

    dataset = LeRobotDataset.create(
        repo_id=args.repo,
        robot_type=robot_type,
        fps=args.fps,
        features=features,
        image_writer_threads=4,
    )

    # ── Load parquet ──────────────────────────────────────────────────────────
    parquet_file = src / "data" / "chunk-000" / "file-000.parquet"
    if not parquet_file.exists():
        raise FileNotFoundError(f"Parquet file not found: {parquet_file}")

    df = pd.read_parquet(parquet_file)
    episodes = sorted(df["episode_index"].unique())
    print(f"\nEpisodes: {len(episodes)} | Total frames: {len(df)}")

    # ── Video paths ───────────────────────────────────────────────────────────
    overhead_video    = src / "videos" / "observation.images.overhead"    / "chunk-000" / "file-000.mp4"
    wrist_right_video = src / "videos" / "observation.images.wrist_right" / "chunk-000" / "file-000.mp4"

    if not overhead_video.exists():
        raise FileNotFoundError(f"Overhead video not found: {overhead_video}")
    if not wrist_right_video.exists():
        raise FileNotFoundError(f"Wrist-right video not found: {wrist_right_video}")

    cap_overhead    = cv2.VideoCapture(str(overhead_video))
    cap_wrist_right = cv2.VideoCapture(str(wrist_right_video))

    cap_wrist_left = None
    if args.bimanual:
        wrist_left_video = src / "videos" / "observation.images.wrist_left" / "chunk-000" / "file-000.mp4"
        if not wrist_left_video.exists():
            raise FileNotFoundError(f"Wrist-left video not found: {wrist_left_video}")
        cap_wrist_left = cv2.VideoCapture(str(wrist_left_video))

    # ── Convert episodes ──────────────────────────────────────────────────────
    for ep_idx in episodes:
        ep_df = df[df["episode_index"] == ep_idx]
        print(f"  Episode {ep_idx:3d}: {len(ep_df):4d} frames", end="", flush=True)

        frames_added = 0
        for _, row in ep_df.iterrows():
            ret1, overhead_frame    = cap_overhead.read()
            ret2, wrist_right_frame = cap_wrist_right.read()

            if not ret1 or not ret2:
                print(f" [WARNING: video ended at frame {frames_added}]", end="")
                break

            frame_dict = {
                "image":       cv2.cvtColor(overhead_frame,    cv2.COLOR_BGR2RGB),
                "wrist_image": cv2.cvtColor(wrist_right_frame, cv2.COLOR_BGR2RGB),
                "state":       np.array(row["observation.state"], dtype=np.float32),
                "actions":     np.array(row["action"],            dtype=np.float32),
                "task":        task,
            }

            if args.bimanual:
                ret3, wrist_left_frame = cap_wrist_left.read()
                if not ret3:
                    print(f" [WARNING: left-wrist video ended at frame {frames_added}]", end="")
                    break
                frame_dict["wrist_image_left"] = cv2.cvtColor(wrist_left_frame, cv2.COLOR_BGR2RGB)

            dataset.add_frame(frame_dict)
            frames_added += 1

        dataset.save_episode()
        print(f" ✓")

    cap_overhead.release()
    cap_wrist_right.release()
    if cap_wrist_left is not None:
        cap_wrist_left.release()

    config_name = "pi05_twinnexus_bimanual_finetune" if args.bimanual else "pi05_twinnexus_finetune"
    print(f"\nDone. {len(episodes)} episodes converted to {output_path}")
    print(f"Next step: uv run scripts/compute_norm_stats.py --config-name {config_name}")


if __name__ == "__main__":
    main()
