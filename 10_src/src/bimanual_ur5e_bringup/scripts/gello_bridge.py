#!/usr/bin/env python3
"""
gello_bridge.py
---------------
Generic GELLO → admittance controller bridge for TwinNexus.
Supports right and left arms via ROS 2 parameters.

Startup behaviour:
    1. On first joint state: freeze robot's INITIAL position
    2. Publish frozen initial position for startup_hold_s seconds
    3. Print GELLO vs Robot delta every 0.5s for visual pose matching
    4. Once deltas < max_initial_delta_rad AND hold expires: stream GELLO live

Usage:
    # Right arm (default):
    ros2 run bimanual_ur5e_bringup gello_bridge.py \
      --ros-args -r __node:=gello_bridge_right

    # Left arm:
    ros2 run bimanual_ur5e_bringup gello_bridge.py \
      --ros-args -r __node:=gello_bridge_left \
        -p gello_port:=/dev/serial/by-id/usb-FTDI_..._FTXXXXXX-if00-port0 \
        -p joint_offsets:=[0.0,3.14,-1.57,4.71,7.85,4.71] \
        -p joint_signs:=[1,1,-1,1,1,1] \
        -p gripper_config:=[7,115.4,73.6] \
        -p joint_states_topic:=/left/joint_states \
        -p admittance_topic:=/left_admittance_controller/joint_references \
        -p gripper_topic:=/left_arm/wsg32_node/cmd_pos

GELLO hardware config (TwinNexus RIGHT arm defaults):
    U2D2 serial:   FTAO4WDM
    Servo IDs:     1-6 (joints) + 7 (gripper)
    Joint offsets: [0, π, -π/2, 3π/2, 5π/2, 3π/2]
    Joint signs:   [1, 1, -1, 1, 1, 1]
    Gripper:       servo 7, open=115.4°, close=73.6°
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

# ── UR5e joint names (kinematic order) ───────────────────────────────────────
UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# ── Control ───────────────────────────────────────────────────────────────────
LOOKAHEAD_NS = 2_000_000   # 2ms — matches UR5e 500Hz RTDE cycle


class GELLOBridge(Node):

    def __init__(self):
        super().__init__('gello_bridge')

        # ── Parameters — all hardware config is overridable ───────────────
        # GELLO hardware
        self.declare_parameter(
            'gello_port',
            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4WDM-if00-port0'
        )
        self.declare_parameter(
            'joint_offsets',
            [0.0, 3.14159, -1.5708, 4.7124, 7.8540, 4.7124]   # [0, π, -π/2, 3π/2, 5π/2, 3π/2]
        )
        self.declare_parameter(
            'joint_signs',
            [1.0, 1.0, -1.0, 1.0, 1.0, 1.0]
        )
        self.declare_parameter(
            'gripper_config',
            [7.0, 115.4, 73.6]   # [servo_id, open_deg, close_deg]
        )

        # ROS topics — override for left arm
        self.declare_parameter('joint_states_topic',  '/joint_states')
        self.declare_parameter(
            'admittance_topic',
            '/admittance_controller/joint_references'
        )
        self.declare_parameter('gripper_topic', '/right_arm/wsg32_node/cmd_pos')

        # Control
        self.declare_parameter('publish_hz',            500.0)
        self.declare_parameter('startup_hold_s',          3.0)
        self.declare_parameter('max_initial_delta_rad',   0.3)
        self.declare_parameter('bridge_delta_rad',       0.001)
        self.declare_parameter('tracking_delta_rad',     0.002)
        self.declare_parameter('tracking_threshold',      0.05)
        self.declare_parameter('gripper_max_mm',         55.0)

        # ── Read parameters ───────────────────────────────────────────────
        self._gello_port     = self.get_parameter('gello_port').value
        raw_offsets          = self.get_parameter('joint_offsets').value
        raw_signs            = self.get_parameter('joint_signs').value
        raw_gripper          = self.get_parameter('gripper_config').value
        joint_states_topic   = self.get_parameter('joint_states_topic').value
        admittance_topic     = self.get_parameter('admittance_topic').value
        gripper_topic        = self.get_parameter('gripper_topic').value
        pub_hz               = self.get_parameter('publish_hz').value
        self._hold_s         = self.get_parameter('startup_hold_s').value
        self._max_initial_delta  = self.get_parameter('max_initial_delta_rad').value
        self._bridge_delta       = self.get_parameter('bridge_delta_rad').value
        self._tracking_delta     = self.get_parameter('tracking_delta_rad').value
        self._tracking_threshold = self.get_parameter('tracking_threshold').value
        self._gripper_max_mm     = self.get_parameter('gripper_max_mm').value
        self._dt = 1.0 / pub_hz

        # Convert parameter lists to tuples for gello_software
        self._joint_offsets = tuple(float(v) for v in raw_offsets)
        self._joint_signs   = tuple(int(v)   for v in raw_signs)
        self._gripper_config = (
            int(raw_gripper[0]),
            float(raw_gripper[1]),
            float(raw_gripper[2]),
        )

        # ── State ─────────────────────────────────────────────────────────
        self._current_pos:   list[float] | None = None
        self._initial_pos:   list[float] | None = None
        self._last_pub:      list[float] | None = None
        self._gello:         object | None       = None
        self._gello_ready:   bool                = False
        self._elapsed:       float               = 0.0
        self._hold_active:   bool                = True
        self._last_gripper:  float               = 0.0
        self.tracking_active: bool               = False

        # ── ROS interfaces ────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            joint_states_topic,
            self._cb_joint_states,
            1,
        )
        self._pub = self.create_publisher(
            JointTrajectoryPoint,
            admittance_topic,
            1,
        )
        self._gripper_pub = self.create_publisher(
            Float32,
            gripper_topic,
            1,
        )
        self.create_timer(self._dt, self._publish)

        hold_str = (
            f"{self._hold_s:.1f}s"
            if not math.isinf(self._hold_s)
            else "∞"
        )
        self.get_logger().info(
            f'GELLO bridge [LIVE] @ {pub_hz:.0f}Hz | '
            f'startup_hold={hold_str} | '
            f'port={self._gello_port.split("/")[-1]} | '
            f'joints→{admittance_topic} | '
            f'gripper→{gripper_topic}'
        )

    # ── GELLO init ────────────────────────────────────────────────────────────

    def _init_gello(self) -> bool:
        try:
            from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig

            config = DynamixelRobotConfig(
                joint_ids=(1, 2, 3, 4, 5, 6),
                joint_offsets=self._joint_offsets,
                joint_signs=self._joint_signs,
                gripper_config=self._gripper_config,
            )
            start = np.append(self._initial_pos[:6], 0.0)
            self._gello = GelloAgent(
                port=self._gello_port,
                dynamixel_config=config,
                start_joints=start,
            )
            self.get_logger().info(
                f'GELLO initialized. '
                f'offsets={[f"{v:.3f}" for v in self._joint_offsets]} '
                f'signs={list(self._joint_signs)}'
            )
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
                throttle_duration_sec=2.0,
            )
            return

        self._current_pos = pos

        if self._initial_pos is None:
            self._initial_pos = pos.copy()
            self.get_logger().info(
                f'Initial robot position frozen: '
                f'{[f"{v:.3f}" for v in self._initial_pos]}'
            )
            ok = self._init_gello()
            self._gello_ready = ok
            if not ok:
                raise SystemExit(1)

    # ── Publish loop ──────────────────────────────────────────────────────────

    def _publish(self):
        if self._initial_pos is None:
            return
        if not self._gello_ready:
            return

        result = self._gello_target()
        if result is None:
            return
        gello_target, gripper = result
        self._last_gripper = gripper

        # ── Debug line — only shown before tracking is active ─────────────
        if not self.tracking_active:
            self.get_logger().info(
                f'\n'
                f'  GELLO : {[f"{v:+.3f}" for v in gello_target]} grip={gripper:.3f}\n'
                f'  Robot : {[f"{v:+.3f}" for v in self._current_pos]} grip={self._last_gripper:.3f}\n'
                f'  Delta : {[f"{g-r:+.3f}" for g, r in zip(gello_target, self._current_pos)]}',
                throttle_duration_sec=0.5,
            )

        # ── Startup hold ──────────────────────────────────────────────────
        if self._hold_active:
            self._elapsed += self._dt
            target = self._initial_pos.copy()
            if not math.isinf(self._hold_s) and self._elapsed >= self._hold_s:
                max_delta = max(
                    abs(g - r)
                    for g, r in zip(gello_target, self._current_pos)
                )
                if max_delta > self._max_initial_delta:
                    self.get_logger().warn(
                        f'HOLD NOT RELEASED — max delta {max_delta:.3f} rad '
                        f'exceeds limit {self._max_initial_delta:.3f} rad. '
                        f'Move GELLO closer to robot pose.',
                        throttle_duration_sec=1.0,
                    )
                    self._elapsed = 0.0
                else:
                    self._hold_active = False
                    self._last_pub = self._initial_pos.copy()
                    self.get_logger().info(
                        f'Hold released. Max delta={max_delta:.3f} rad. '
                        f'Bridging to GELLO.'
                    )
        else:
            # ── Two-phase rate limiter ────────────────────────────────────
            max_gap = max(
                abs(g - p)
                for g, p in zip(gello_target, self._last_pub)
            )
            if max_gap > self._tracking_threshold and not self.tracking_active:
                delta = self._bridge_delta
                if max_gap < self._tracking_threshold / 2:
                    self.tracking_active = True
            else:
                delta = self._tracking_delta
                

            target = self._rate_limit_with(self._last_pub, gello_target, delta)
            self._last_pub = target

            # ── Publish gripper ───────────────────────────────────────────
            g_msg = Float32()
            g_msg.data = float((1.0 - gripper) * self._gripper_max_mm)
            self._gripper_pub.publish(g_msg)

        # ── Publish joint reference ───────────────────────────────────────
        msg = JointTrajectoryPoint()
        msg.positions       = target
        msg.velocities      = [0.0] * 6
        msg.time_from_start = Duration(sec=0, nanosec=LOOKAHEAD_NS)
        self._pub.publish(msg)

    # ── Target generator ─────────────────────────────────────────────────────

    def _gello_target(self) -> tuple[list[float], float] | None:
        try:
            state   = self._gello.act({})
            joints  = state[:6].tolist()
            gripper = float(state[6])   # 0.0=open, 1.0=closed
            return joints, gripper
        except Exception as e:
            self.get_logger().warn(
                f'GELLO read error: {e} — holding last position',
                throttle_duration_sec=1.0,
            )
            return self._last_pub, self._last_gripper

    # ── Safety ────────────────────────────────────────────────────────────────

    def _rate_limit_with(self,
                         previous: list[float],
                         target:   list[float],
                         max_d:    float) -> list[float]:
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