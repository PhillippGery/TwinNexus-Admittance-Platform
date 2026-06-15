#!/usr/bin/env python3
"""
test_twinnexus_robot.py
-----------------------
Standalone smoke test for the TwinNexusRobot LeRobot bridge.
Run this BEFORE attempting any episode recording.

Does NOT record a dataset. Validates:
    1. ROS2 topics are reachable from inside the lerobot venv
    2. Joint states arrive and parse correctly
    3. Gripper state arrives
    4. Camera frame arrives and has correct shape
    5. get_observation() returns correct keys and shapes
    6. send_action() with hold-position action produces no errors

Usage:
    source ~/lerobot_env/bin/activate
    # With full stack running (boot_hw + Play + spawnctrl):
    python3 test_twinnexus_robot.py
"""

import sys
import time
import numpy as np

# Add the directory containing twinnexus_robot.py to path
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lerobot.robots.twinnexus import TwinNexusRobot, TwinNexusRobotConfig


def check(label: str, condition: bool, detail: str = "") -> bool:
    status = "✓" if condition else "✗"
    print(f"  {status}  {label}" + (f"  [{detail}]" if detail else ""))
    return condition


def main():
    print("\n" + "="*60)
    print("  TwinNexusRobot Smoke Test")
    print("="*60 + "\n")

    config = TwinNexusRobotConfig()
    robot = TwinNexusRobot(config)

    # ── Step 1: Connect ───────────────────────────────────────────────────────
    print("[1/5] Connecting to ROS2 stack ...")
    try:
        robot.connect(calibrate=False)
        print("      ✓ Connected\n")
    except (TimeoutError, Exception) as e:
        print(f"      ✗ FAILED: {e}")
        print("\n  Checklist:")
        print("  - Is boot_hw running?")
        print("  - Did you press Play on the teach pendant?")
        print("  - Is spawnctrl running?")
        print("  - Are you inside the lerobot venv? (source ~/lerobot_env/bin/activate)\n")
        sys.exit(1)

    all_passed = True

    try:
        # ── Step 2: Observation structure ─────────────────────────────────────
        print("[2/5] Checking observation features ...")
        feats = robot.observation_features
        all_passed &= check("observation.state in features",
                            "observation.state" in feats)
        all_passed &= check("observation.state shape is (7,)",
                            feats.get("observation.state") == (7,))
        all_passed &= check("wrist_left image in features",
                            "observation.images.wrist_left" in feats)
        print()

        # ── Step 3: get_observation ───────────────────────────────────────────
        print("[3/5] Calling get_observation() ...")
        obs = robot.get_observation()

        state = obs["observation.state"]
        all_passed &= check("observation.state present",
                            "observation.state" in obs)
        all_passed &= check("observation.state shape",
                            state.shape == (7,),
                            str(state.shape))
        all_passed &= check("observation.state dtype float32",
                            state.dtype == np.float32,
                            str(state.dtype))
        all_passed &= check("joint values in valid range (±2π)",
                            np.all(np.abs(state[:6]) < 2 * np.pi),
                            str(state[:6].round(3).tolist()))
        all_passed &= check("gripper value ≥ 0",
                            float(state[6]) >= 0.0,
                            f"{state[6]:.4f} m")

        if "observation.images.wrist_left" in obs:
            img = obs["observation.images.wrist_left"]
            all_passed &= check("wrist_left image shape",
                                img.shape == (720, 1280, 3),
                                str(img.shape))
            all_passed &= check("wrist_left image dtype uint8",
                                img.dtype == np.uint8,
                                str(img.dtype))
            all_passed &= check("wrist_left image not all zeros",
                                img.sum() > 0)
        print()

        # ── Step 4: Action features ───────────────────────────────────────────
        print("[4/5] Checking action features ...")
        act_feats = robot.action_features
        all_passed &= check("action in action_features",
                            "action" in act_feats)
        all_passed &= check("action shape is (7,)",
                            act_feats.get("action") == (7,))
        print()

        # ── Step 5: send_action (hold current position) ───────────────────────
        print("[5/5] Sending hold-position action for 3 seconds ...")
        print("      Robot should NOT move.\n")

        for i in range(3):
            obs = robot.get_observation()
            # Hold position: action = current state
            sent = robot.send_action({"action": obs["observation.state"]})
            all_passed &= check(
                f"  send_action cycle {i+1}/3",
                sent["action"].shape == (7,),
                str(sent["action"].round(3).tolist())
            )
            time.sleep(1.0)

        print()

    finally:
        time.sleep(0.5)  # Brief pause before disconnecting
        robot.disconnect()

    # ── Summary ───────────────────────────────────────────────────────────────
    print("="*60)
    if all_passed:
        print("  ALL TESTS PASSED")
        print("  TwinNexusRobot is ready for episode recording.")
    else:
        print("  SOME TESTS FAILED — fix before recording episodes.")
    print("="*60 + "\n")

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
