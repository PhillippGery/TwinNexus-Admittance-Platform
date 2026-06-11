#!/usr/bin/env python3
"""
gello_bridge.py
---------------
Bridges GELLO leader joint positions → /admittance_controller/joint_references.

Startup behaviour:
    1. On first joint state: freeze robot's INITIAL position
    2. Publish frozen initial position indefinitely (startup_hold_s = inf by default)
    3. Every 0.5s print: GELLO angles vs Robot angles for visual matching
    4. Once you confirm they match: restart with --ros-args -p startup_hold_s:=3.0
    5. After hold expires: stream GELLO positions live

GELLO hardware config (TwinNexus right arm):
    U2D2 serial:   FTAO4WDM
    Servo IDs:     1-6 (joints) + 7 (gripper)
    Joint offsets: [0, π, -π/2, 3π/2, 5π/2, 3π/2]
    Joint signs:   [1, 1, -1, 1, 1, 1]

Usage:
    # Step 1 — match check (hold forever, no motion):
    ros2 run bimanual_ur5e_bringup gello_bridge.py

    # Step 2 — once matched, enable motion with 3s hold:
    ros2 run bimanual_ur5e_bringup gello_bridge.py --ros-args -p startup_hold_s:=3.0

    # Mock mode:
    ros2 run bimanual_ur5e_bringup gello_bridge.py --ros-args -p mock:=true
"""

import sys
import os
import traceback
import math

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32

# Add gello_software to path
GELLO_PATH = os.path.expanduser(
    "~/TwinNexus-Admittance-Platform/10_src/src/gello_software"
)
if GELLO_PATH not in sys.path:
    sys.path.insert(0, GELLO_PATH)

# ── UR5e joint names (kinematic order, must match admittance controller) ──────
UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# ── GELLO hardware config ─────────────────────────────────────────────────────
GELLO_PORT = (
    "/dev/serial/by-id/"
    "usb-FTDI_USB__-__Serial_Converter_FTAO4WDM-if00-port0"
)

JOINT_OFFSETS = (
    0 * np.pi / 2,    # joint 1  → 0.000
    2 * np.pi / 2,    # joint 2  → 3.142
    -1 * np.pi / 2,   # joint 3  → -1.571
    3 * np.pi / 2,    # joint 4  → 4.712
    5 * np.pi / 2,    # joint 5  → 7.854
    3 * np.pi / 2,    # joint 6  → 4.712
)

JOINT_SIGNS   = (1, 1, -1, 1, 1, 1)
GRIPPER_CONFIG = (7, 115.4, 73.6)

# ── Control ───────────────────────────────────────────────────────────────────
LOOKAHEAD_NS  = 2_000_000   # 200ms



class GELLOBridge(Node):

    def __init__(self):
        super().__init__('gello_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('publish_hz',      500.0)
        self.declare_parameter('startup_hold_s',  3.0)  # time to hold initial position before bridging to GELLO
        self.declare_parameter('mock_amp_rad',    0.02)
        self.declare_parameter('mock_freq_hz',    0.2)
        self.declare_parameter('bridge_delta_rad',   0.001)   # slow during initial bridge
        self.declare_parameter('tracking_delta_rad', 0.002)   # fast once locked in
        self.declare_parameter('tracking_threshold', 0.05)   # rad — when to switch
        self.declare_parameter('max_initial_delta_rad', 0.3)
        self.declare_parameter('gripper_max_mm', 67.0)

        pub_hz            = self.get_parameter('publish_hz').value
        self._hold_s      = self.get_parameter('startup_hold_s').value
        self._bridge_delta       = self.get_parameter('bridge_delta_rad').value
        self._tracking_delta     = self.get_parameter('tracking_delta_rad').value
        self._tracking_threshold = self.get_parameter('tracking_threshold').value
        self._max_initial_delta  = self.get_parameter('max_initial_delta_rad').value
        self._gripper_max_mm = self.get_parameter('gripper_max_mm').value
        self._dt          = 1.0 / pub_hz

        # ── State ─────────────────────────────────────────────────────────
        self._current_pos:   list[float] | None = None  # live robot position
        self._initial_pos:   list[float] | None = None  # frozen at startup
        self._last_pub:      list[float] | None = None  # last published target
        self._gello:         object | None       = None
        self._gello_ready:   bool                = False
        self._elapsed:       float               = 0.0
        self._hold_active:   bool                = True
        self._last_gripper = 0.0
        self.tracking_active: bool               = False

        # ── ROS ───────────────────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states',
                                 self._cb_joint_states, 1)
        self._pub = self.create_publisher(
                    JointTrajectoryPoint,
                    '/admittance_controller/joint_references', 1)
        
        self._gripper_pub = self.create_publisher(
            Float32,
            '/right_arm/wsg32_node/cmd_pos',
            1
        )

        self.create_timer(self._dt, self._publish)

        hold_str = f"{self._hold_s:.1f}s" if not math.isinf(self._hold_s) \
                   else "∞ (match GELLO to robot, then restart with startup_hold_s:=3.0)"
        mode = "LIVE (GELLO)"
        self.get_logger().info(
            f'GELLO bridge [{mode}] @ {pub_hz:.0f}Hz | '
            f'startup_hold={hold_str}'
        )

    # ── GELLO init ────────────────────────────────────────────────────────────

    def _init_gello(self) -> bool:
        try:
            from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig
            config = DynamixelRobotConfig(
                joint_ids=(1, 2, 3, 4, 5, 6),
                joint_offsets=JOINT_OFFSETS,
                joint_signs=JOINT_SIGNS,
                gripper_config=GRIPPER_CONFIG,
            )
            # Pass initial robot position so DynamixelRobot corrects 2π ambiguity
            start = np.append(self._initial_pos[:6], 0.0)
            self._gello = GelloAgent(
                port=GELLO_PORT,
                dynamixel_config=config,
                start_joints=start,
            )
            self.get_logger().info('GELLO initialized.')
            return True
        except Exception as e:
            self.get_logger().fatal(
                f'GELLO init failed: {e}\n{traceback.format_exc()}'
            )
            return False

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            pos = [name_to_pos[n] for n in UR5E_JOINT_NAMES]
        except KeyError:
            self.get_logger().warn(
                f'Joint names mismatch. Got: {list(msg.name)}',
                throttle_duration_sec=2.0)
            return

        self._current_pos = pos

        # Freeze initial position on very first joint state
        if self._initial_pos is None:
            self._initial_pos = pos.copy()
            self.get_logger().info(
                f'Initial robot position frozen: '
                f'{[f"{v:.3f}" for v in self._initial_pos]}'
            )

            # Init GELLO immediately after we have initial position
            ok = self._init_gello()
            self._gello_ready = ok
            if not ok:
                raise SystemExit(1)



    # ── Publish loop ──────────────────────────────────────────────────────────

    def _publish(self):
        if self._initial_pos is None:
            return  # waiting for first joint state

        # ── Get GELLO target ──────────────────────────────────────────────

        if not self._gello_ready:
            return
        result = self._gello_target()

        if result is None:
            return
        gello_target, gripper = result
        self._last_gripper = gripper

        # ── Debug line: GELLO vs Robot ────────────────────────────────────
        # Printed every 0.5s so you can visually compare before motion starts
        self.get_logger().info(
        f'\n'
        f'  GELLO : {[f"{v:+.3f}" for v in gello_target]} grip={gripper:.3f}\n'
        f'  Robot : {[f"{v:+.3f}" for v in self._current_pos]} grip={self._last_gripper:.3f}\n'
        f'  Delta : {[f"{g-r:+.3f}" for g, r in zip(gello_target, self._current_pos)]}',
        throttle_duration_sec=0.5
        )

        # ── Startup hold ──────────────────────────────────────────────────
        # During hold: always publish FROZEN INITIAL POSITION
        # This is safe for the admittance controller — the reference
        # never changes so no motion is commanded
        if self._hold_active:
            self._elapsed += self._dt
            target = self._initial_pos.copy()
            if not math.isinf(self._hold_s) and self._elapsed >= self._hold_s:
                # Safety check — GELLO must be close enough to robot
                max_delta = max(abs(g - r) for g, r in
                                zip(gello_target, self._current_pos))
                if max_delta > self._max_initial_delta:
                    self.get_logger().warn(
                        f'HOLD NOT RELEASED — max delta {max_delta:.3f} rad exceeds '
                        f'max_initial_delta {self._max_initial_delta:.3f} rad. '
                        f'Move GELLO closer to robot pose.',
                        throttle_duration_sec=1.0
                    )
                    # Reset elapsed so we keep checking every hold_s seconds
                    self._elapsed = 0.0
                else:
                    self._hold_active = False
                    self._last_pub    = self._initial_pos.copy()
                    self.get_logger().info(
                        f'Hold released. Max delta was {max_delta:.3f} rad. '
                        f'Bridging to GELLO.'
                    )
        else:
            # Choose rate limit based on how close we are to GELLO
            max_gap = max(abs(g - p) for g, p in
                        zip(gello_target, self._last_pub))
            
            if max_gap > self._tracking_threshold and not self.tracking_active:
                delta = self._bridge_delta
            else:
                delta = self._tracking_delta
                if max_gap < self._tracking_threshold/2:
                    self.tracking_active = True
            
            target = self._rate_limit_with(self._last_pub, gello_target, delta)
            self._last_pub = target

            g_msg = Float32()
            g_msg.data = (1.0 - gripper) * self._gripper_max_mm
            self._gripper_pub.publish(g_msg)

        # ── Publish ───────────────────────────────────────────────────────
        msg = JointTrajectoryPoint()
        msg.positions  = target
        msg.velocities = [0.0] * 6
        msg.time_from_start = Duration(sec=0, nanosec=LOOKAHEAD_NS)
        self._pub.publish(msg)

    # ── Target generators ─────────────────────────────────────────────────────

    def _gello_target(self) -> tuple[list[float], float] | None:
        try:
            state = self._gello.act({})
            joints  = state[:6].tolist()
            gripper = float(state[6])   # 0.0=open, 1.0=closed
            return joints, gripper
        except Exception as e:
            self.get_logger().warn(
                f'GELLO read error: {e} — holding last position',
                throttle_duration_sec=1.0)
            return self._last_pub, self._last_gripper
    

    # ── Safety ────────────────────────────────────────────────────────────────

    def _rate_limit_with(self, previous, target, max_d):
        limited = []
        for prev, new in zip(previous, target):
            d = max(-max_d, min(max_d, new - prev))
            limited.append(prev + d)
        return limited


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GELLOBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()