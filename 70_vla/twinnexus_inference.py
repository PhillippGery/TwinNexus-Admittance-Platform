#!/usr/bin/env python3
"""
twinnexus_inference.py
----------------------
π0.5 inference for the TwinNexus UR5e robot.

Prerequisites:
    1. boot_hw running, Play pressed on UR5e teach pendant
    2. spawnctrl running (admittance controller + GELLO)
    3. openpi policy server running:
           cd ~/openpi && python scripts/serve_policy.py \
               --env pi05_twinnexus_finetune \
               --checkpoint-path <path>

Usage:
    vla_env
    python3 ~/TwinNexus-Admittance-Platform/70_vla/twinnexus_inference.py

Controls:
    Enter   → stop inference cleanly
    Ctrl+C  → emergency stop
"""

import argparse
import logging
import select
import sys
import time

import numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────
POLICY_HOST        = "localhost"
POLICY_PORT        = 8000
INFERENCE_FPS      = 24           # matches recording fps
ACTION_HORIZON     = 10           # action chunk size from training config
REPLAN_STEPS       = 10           # re-query policy every N steps
SAFETY_DELTA_RAD   = 0.1          # rad — skip chunk if any joint jump > this
TASK_PROMPT        = "pick up the screwdriver and place it in the paper box"


def build_policy_obs(robot_obs: dict) -> dict:
    """Remap TwinNexusRobot observation keys to the π0.5 server's input keys."""
    overhead    = robot_obs.get("observation.images.overhead")
    wrist_right = robot_obs.get("observation.images.wrist_right")

    if overhead is None:
        raise RuntimeError("overhead camera frame is None — is the camera connected?")
    if wrist_right is None:
        raise RuntimeError("wrist_right camera frame is None — is the camera connected?")

    return {
        "observation/image":       overhead,      # (480, 640, 3) uint8
        "observation/wrist_image": wrist_right,   # (480, 640, 3) uint8
        "observation/state":       robot_obs["observation.state"].astype(np.float32),  # (7,)
        "prompt":                  TASK_PROMPT,
    }


def is_enter_pressed() -> bool:
    """Non-blocking check: returns True if the user pressed Enter."""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


def safety_check(action_chunk: np.ndarray, current_state: np.ndarray) -> bool:
    """Return True if the chunk is safe to execute.

    Compares the FIRST action's joint positions against the current joints.
    If any joint delta exceeds SAFETY_DELTA_RAD, the chunk is rejected.
    """
    first_joints  = action_chunk[0, :6]
    current_joints = current_state[:6]
    deltas = np.abs(first_joints - current_joints)
    bad = np.where(deltas > SAFETY_DELTA_RAD)[0]
    if bad.size:
        logger.warning(
            "SAFETY: chunk rejected — joint(s) %s would jump by %s rad "
            "(threshold %.3f rad)",
            bad.tolist(),
            np.round(deltas[bad], 4).tolist(),
            SAFETY_DELTA_RAD,
        )
        return False
    return True


def run_inference(robot, policy) -> None:
    """Main inference loop."""
    dt = 1.0 / INFERENCE_FPS
    step = 0
    action_chunk: np.ndarray | None = None
    chunk_step   = REPLAN_STEPS     # force query on first iteration

    print("\nPress Enter to stop inference.\n")
    logger.info("Inference running at %d fps, action horizon %d", INFERENCE_FPS, ACTION_HORIZON)

    while True:
        t_start = time.perf_counter()

        if is_enter_pressed():
            print("\nStopping inference.")
            break

        # ── Re-query policy every REPLAN_STEPS ───────────────────────────────
        if chunk_step >= REPLAN_STEPS:
            obs        = robot.get_observation()
            policy_obs = build_policy_obs(obs)

            logger.debug("Querying policy (step %d)...", step)
            t_infer = time.perf_counter()
            result   = policy.infer(policy_obs)
            logger.info(
                "Policy inference: %.1f ms",
                (time.perf_counter() - t_infer) * 1000,
            )

            action_chunk = np.asarray(result["actions"], dtype=np.float32)  # (10, 7)

            if not safety_check(action_chunk, obs["observation.state"]):
                # Skip chunk; try again next cycle with a fresh query
                time.sleep(dt)
                continue

            chunk_step = 0

        # ── Execute one step from the current chunk ───────────────────────────
        step_action = action_chunk[chunk_step]   # (7,)
        obs         = robot.get_observation()
        current_state = obs["observation.state"]

        # Log state and action every step
        joints_str = " ".join(f"{j:+.4f}" for j in current_state[:6])
        action_str = " ".join(f"{a:+.4f}" for a in step_action[:6])
        gripper_str = f"grip={step_action[6]:+.4f}m"
        print(
            f"[step {step:04d} chunk {chunk_step}] "
            f"state: [{joints_str}] | "
            f"action: [{action_str}] {gripper_str}"
        )

        robot.send_action({"action": step_action})

        chunk_step += 1
        step       += 1

        # ── Sleep to maintain inference fps ───────────────────────────────────
        elapsed  = time.perf_counter() - t_start
        sleep_s  = dt - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            logger.debug("Over budget by %.1f ms", -sleep_s * 1000)


def main() -> None:
    global TASK_PROMPT, INFERENCE_FPS, REPLAN_STEPS, SAFETY_DELTA_RAD

    parser = argparse.ArgumentParser(description="TwinNexus π0.5 inference")
    parser.add_argument("--host",   default=POLICY_HOST, help="Policy server host")
    parser.add_argument("--port",   default=POLICY_PORT, type=int, help="Policy server port")
    parser.add_argument("--task",   default=TASK_PROMPT, help="Task prompt string")
    parser.add_argument("--fps",    default=INFERENCE_FPS, type=int, help="Inference fps")
    parser.add_argument("--replan", default=REPLAN_STEPS, type=int,
                        help="Re-query policy every N steps")
    parser.add_argument("--safety-delta", default=SAFETY_DELTA_RAD, type=float,
                        help="Max joint delta (rad) before a chunk is rejected")
    args = parser.parse_args()

    TASK_PROMPT      = args.task
    INFERENCE_FPS    = args.fps
    REPLAN_STEPS     = args.replan
    SAFETY_DELTA_RAD = args.safety_delta

    # ── Import policy client ──────────────────────────────────────────────────
    import sys as _sys
    _sys.path.insert(0, "/home/phillippgery/openpi/packages/openpi-client/src")
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    # ── Import robot ──────────────────────────────────────────────────────────
    from lerobot.robots.twinnexus import TwinNexusRobot, TwinNexusRobotConfig

    print("\n" + "=" * 60)
    print("  TwinNexus π0.5 Inference")
    print("=" * 60)
    print(f"  Policy server: ws://{args.host}:{args.port}")
    print(f"  Task:          {TASK_PROMPT}")
    print(f"  Fps:           {INFERENCE_FPS}  |  Replan every: {REPLAN_STEPS} steps")
    print(f"  Safety delta:  {SAFETY_DELTA_RAD} rad")
    print("=" * 60)
    print("\nMake sure:")
    print("  1. boot_hw is running and Play is pressed")
    print("  2. spawnctrl is running")
    print("  3. openpi serve_policy.py is running at the address above")
    input("\nPress Enter to connect and start inference...")

    # ── Connect policy server ─────────────────────────────────────────────────
    print(f"\nConnecting to policy server at ws://{args.host}:{args.port} ...")
    policy = WebsocketClientPolicy(host=args.host, port=args.port)
    metadata = policy.get_server_metadata()
    logger.info("Server metadata: %s", metadata)
    print("Policy server connected.")

    # ── Connect robot ─────────────────────────────────────────────────────────
    print("Connecting to TwinNexusRobot ...")
    config = TwinNexusRobotConfig()
    robot  = TwinNexusRobot(config)
    robot.connect()
    print("Robot connected.")

    # ── Move to home before starting ──────────────────────────────────────────
    print("  Moving to home position...")
    robot.go_home()
    print("  Home position reached.")
    input("  Press Enter to start inference...")

    try:
        run_inference(robot, policy)
    except KeyboardInterrupt:
        print("\nCtrl+C — stopping.")
    finally:
        # ── Always return to home after a run ─────────────────────────────────
        print("Returning to home position...")
        robot.go_home()
        print("Disconnecting robot...")
        robot.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
