#!/usr/bin/env python3
"""
gello_bridge.py
---------------
Bridges GELLO leader joint positions → /admittance_controller/joint_references.

Publishes trajectory_msgs/JointTrajectoryPoint at 50Hz with a 200ms
rolling horizon. The admittance controller interpolates smoothly between
samples at its 500Hz internal rate.

GELLO hardware config (TwinNexus right arm):
    U2D2 serial:   FTAO4WDM
    Servo IDs:     1-6 (joints) + 7 (gripper)
    Joint offsets: [π/2, π/2, 2π, π, π, π]
    Joint signs:   [1, 1, -1, 1, 1, 1]
    Gripper:       open=68.0°, close=26.2°

Usage:
    # Live GELLO teleoperation:
    ros2 run bimanual_ur5e_bringup gello_bridge.py

    # Mock mode for pipeline validation (no hardware):
    ros2 run bimanual_ur5e_bringup gello_bridge.py --ros-args -p mock:=true
"""

import sys
import os
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

# Add gello_software to path
GELLO_PATH = os.path.expanduser(
    "~/TwinNexus-Admittance-Platform/10_src/src/gello_software"
)
if GELLO_PATH not in sys.path:
    sys.path.insert(0, GELLO_PATH)


# ── UR5e joint names (must match admittance controller config) ────────────────
UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# ── GELLO hardware config for TwinNexus right arm ─────────────────────────────
GELLO_PORT = (
    "/dev/serial/by-id/"
    "usb-FTDI_USB__-__Serial_Converter_FTAO4WDM-if00-port0"
)

# Offsets from gello_get_offset.py calibration run
JOINT_OFFSETS = (
    1 * np.pi / 2,   # joint 1
    1 * np.pi / 2,   # joint 2
    4 * np.pi / 2,   # joint 3
    2 * np.pi / 2,   # joint 4
    2 * np.pi / 2,   # joint 5
    2 * np.pi / 2,   # joint 6
)
JOINT_SIGNS   = (1, 1, -1, 1, 1, 1)
GRIPPER_CONFIG = (7, 68.0, 26.2)   # (servo_id, open_deg, close_deg)

# ── Control parameters ────────────────────────────────────────────────────────
LOOKAHEAD_NS  = 200_000_000   # 200ms rolling horizon
MAX_DELTA_RAD = 0.01          # max joint delta per cycle (rad) — safety clamp


class GELLOBridge(Node):

    def __init__(self):
        super().__init__('gello_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('mock',               False)
        self.declare_parameter('publish_hz',         50.0)
        self.declare_parameter('mock_amp_rad',       0.02)
        self.declare_parameter('mock_freq_hz',       0.2)
        self.declare_parameter('mock_start_delay_s', 3.0)
        self.declare_parameter('live_start_delay_s', 3.0)
        self.declare_parameter('max_delta_rad',      MAX_DELTA_RAD)

        self._mock        = self.get_parameter('mock').value
        pub_hz            = self.get_parameter('publish_hz').value
        self._mock_amp    = self.get_parameter('mock_amp_rad').value
        self._mock_freq   = self.get_parameter('mock_freq_hz').value
        self._start_delay = self.get_parameter(
            'mock_start_delay_s' if self._mock else 'live_start_delay_s'
        ).value
        self._max_delta   = self.get_parameter('max_delta_rad').value

        self._dt = 1.0 / pub_hz

        # ── Internal state ────────────────────────────────────────────────
        self._current_pos:    list[float] | None = None
        self._mock_center:    list[float] | None = None
        self._last_published: list[float] | None = None
        self._elapsed = 0.0
        self._t       = 0.0

        # GELLO initialized lazily on first joint state
        self._gello       = None
        self._gello_ready = False

        # ── ROS interfaces ────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            '/joint_states',
            self._cb_joint_states,
            1
        )
        self._pub = self.create_publisher(
            JointTrajectoryPoint,
            '/admittance_controller/joint_references',
            1
        )
        self.create_timer(self._dt, self._publish)

        mode = "MOCK" if self._mock else "LIVE (GELLO)"
        self.get_logger().info(
            f'GELLO bridge started [{mode}] @ {pub_hz:.0f}Hz | '
            f'lookahead={LOOKAHEAD_NS/1e6:.0f}ms | '
            f'max_delta={self._max_delta:.3f}rad/cycle | '
            f'startup_hold={self._start_delay:.1f}s'
        )

    # ── GELLO initialization (called on first joint state) ────────────────────

    def _init_gello(self) -> bool:
        """
        Initialize GelloAgent using the robot's current joint positions
        as start_joints. This shifts offsets by multiples of 2π so the
        GELLO and robot start from the same reference.
        Returns True on success, False on failure.
        """
        try:
            from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig

            config = DynamixelRobotConfig(
                joint_ids=(1, 2, 3, 4, 5, 6),
                joint_offsets=JOINT_OFFSETS,
                joint_signs=JOINT_SIGNS,
                gripper_config=GRIPPER_CONFIG,
            )

            # start_joints = current robot pose (6 joints only, not gripper)
            start = np.append(self._current_pos[:6], 0.0) 
            self.get_logger().info(
                f'Initializing GELLO with start_joints: '
                f'{[f"{v:.3f}" for v in start.tolist()]}'
            )

            self._gello = GelloAgent(
                port=GELLO_PORT,
                dynamixel_config=config,
                start_joints=start,
            )
            self.get_logger().info('GELLO initialized successfully.')
            return True

        except Exception as e:
            self.get_logger().fatal(
                f'Failed to initialize GELLO: {e}\n{traceback.format_exc()}'
            )
            return False

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState):
        """Cache the latest measured joint positions in UR5e order."""
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            pos = [name_to_pos[n] for n in UR5E_JOINT_NAMES]
        except KeyError:
            self.get_logger().warn(
                f'Joint reorder failed. Got: {list(msg.name)}',
                throttle_duration_sec=2.0
            )
            return

        if self._current_pos is None:
            self.get_logger().info(f'First joint state: {[f"{v:.3f}" for v in pos]}')

        self._current_pos = pos

        # ── Lazy GELLO init on first joint state ──────────────────────────
        if not self._mock and self._gello is None:
            ok = self._init_gello()
            if ok:
                self._gello_ready = True
            else:
                raise SystemExit(1)

        # ── Mock center freeze ────────────────────────────────────────────
        if self._mock and self._mock_center is None:
            self._mock_center = pos.copy()
            self._t       = 0.0
            self._elapsed = 0.0
            self.get_logger().info(
                f'Mock center frozen: {[f"{v:.3f}" for v in self._mock_center]}'
            )

    # ── Main publish loop ─────────────────────────────────────────────────────

    def _publish(self):
        if self._current_pos is None:
            return   # waiting for first joint state

        if self._mock:
            raw_target = self._mock_target()
        else:
            if not self._gello_ready:
                return   # GELLO not yet initialized
            raw_target = self._gello_target()
            if raw_target is not None:
                self.get_logger().info(
                    f'Robot: {[f"{v:.3f}" for v in self._current_pos]} | '
                    f'GELLO: {[f"{v:.3f}" for v in raw_target]}',
                    throttle_duration_sec=0.5
    )

        if raw_target is None:
            return

        # ── Startup hold ──────────────────────────────────────────────────
        # Hold robot at current position for start_delay seconds.
        # Live mode: gives operator time to match GELLO pose to robot pose.
        # Mock mode: prevents phase-offset jump at sine wave start.
        self._elapsed += self._dt
        if self._elapsed < self._start_delay:
            raw_target = list(self._current_pos)

        # ── Rate-limit the TARGET signal ──────────────────────────────────
        if self._last_published is None:
            self._last_published = self._current_pos.copy()

        safe_target = self._rate_limit(self._last_published, raw_target)
        self._last_published = safe_target

        # ── Publish ───────────────────────────────────────────────────────
        msg = JointTrajectoryPoint()
        msg.positions  = safe_target
        msg.velocities = [0.0] * 6
        msg.time_from_start = Duration(sec=0, nanosec=LOOKAHEAD_NS)
        self._pub.publish(msg)

        self.get_logger().info(
            f'Published: {[f"{p:.3f}" for p in safe_target]}',
            throttle_duration_sec=1.0
        )

    # ── Target generators ─────────────────────────────────────────────────────

    def _gello_target(self) -> list[float] | None:
        """Read 6 joint angles from the physical GELLO."""
        try:
            state  = self._gello.act({})   # shape (7,): 6 joints + 1 gripper
            joints = state[:6].tolist()
            return joints
        except Exception as e:
            self.get_logger().warn(
                f'GELLO read failed: {e}',
                throttle_duration_sec=1.0
            )
            return None

    def _mock_target(self) -> list[float] | None:
        """Sine wave centered on frozen robot position."""
        if self._mock_center is None:
            return list(self._current_pos)

        # Startup hold handled in _publish — just return sine here
        self._t += self._dt
        return [
            center + self._mock_amp * np.sin(2 * np.pi * self._mock_freq * self._t)
            for center in self._mock_center
        ]

    # ── Safety ────────────────────────────────────────────────────────────────

    def _rate_limit(self,
                    previous: list[float],
                    target:   list[float]) -> list[float]:
        """Limit commanded target change per cycle — prevents velocity spikes."""
        limited = []
        for prev, new in zip(previous, target):
            delta = new - prev
            delta = max(-self._max_delta, min(self._max_delta, delta))
            limited.append(prev + delta)
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