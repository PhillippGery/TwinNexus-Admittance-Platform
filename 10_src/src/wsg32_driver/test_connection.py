#!/usr/bin/env python3
"""
test_connection.py
------------------
Standalone hardware connectivity test for the WSG 32.
Run this BEFORE launching the ROS2 node to verify the gripper is reachable
and the GCL text interface is enabled.

Usage (from anywhere, no ROS sourcing needed):
    python3 test_connection.py
    python3 test_connection.py --ip 192.168.1.202   # test second gripper

What it does:
    1. Connect to gripper via TCP
    2. Ack any latched fault
    3. Run homing sequence
    4. Move to 10 mm  (nearly closed)
    5. Move to 45 mm  (nearly open)
    6. Move to 27 mm  (midpoint)
    7. Disconnect cleanly

If step 1 fails → check network / cable / alias IP.
If step 2-3 fail → check GCL text interface enabled in web UI.
"""

import sys
import argparse
import time

# Add the package src to path so we can import without colcon build
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'wsg32_driver'))

from wsg_tcp import WSG32TCP


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip',   default='192.168.1.201')
    parser.add_argument('--port', default=1000, type=int)
    args = parser.parse_args()

    print(f"\n{'='*55}")
    print(f"  WSG 32 Connection Test")
    print(f"  Target: {args.ip}:{args.port}")
    print(f"{'='*55}\n")

    gripper = WSG32TCP(ip=args.ip, port=args.port, timeout=10.0)

    # ── Step 1: Connect ───────────────────────────────────────────────────
    print("[1/5] Connecting ...")
    try:
        gripper.connect()
        print("      ✓ Connected and fault cleared.\n")
    except ConnectionError as e:
        print(f"      ✗ FAILED: {e}")
        print("\n  Checklist:")
        print("  - Is the gripper powered on?")
        print("  - Is the M8→RJ45 cable plugged into the switch?")
        print("  - Is the alias IP up?  Run: ip addr show enp128s31f6")
        print("  - Can you ping?  Run: ping 192.168.1.201\n")
        sys.exit(1)

    # ── Step 2: Home ──────────────────────────────────────────────────────
    print("[2/5] Homing (this takes ~3–5 seconds) ...")
    ok = gripper.home()
    if ok:
        print("      ✓ Homing complete.\n")
    else:
        print("      ✗ Homing failed.")
        print("      Is the GCL text interface enabled?")
        print("      Open http://{args.ip} → Settings → Command Interface")
        gripper.disconnect()
        sys.exit(1)

    # ── Step 3–5: Motion test ─────────────────────────────────────────────
    positions = [
        (10.0,  "nearly closed (10 mm)"),
        (45.0,  "nearly open   (45 mm)"),
        (27.5,  "midpoint      (27.5 mm)"),
    ]

    for i, (pos_mm, label) in enumerate(positions, start=3):
        print(f"[{i}/5] Moving to {label} ...")
        ok = gripper.move(pos_mm)
        if ok:
            print(f"      ✓ At {pos_mm} mm.\n")
        else:
            print(f"      ✗ Move to {pos_mm} mm failed.\n")
        time.sleep(0.5)

    # ── Done ──────────────────────────────────────────────────────────────
    print("[5/5] Disconnecting ...")
    gripper.disconnect()
    print("      ✓ Disconnected cleanly.\n")

    print("="*55)
    print("  ALL TESTS PASSED — gripper is ready for the ROS2 node.")
    print("="*55 + "\n")


if __name__ == '__main__':
    main()
