#!/usr/bin/env python3
"""
gello_bridge.py
---------------
Bridges GELLO leader joint positions → admittance_controller/joint_references.

Publishes trajectory_msgs/JointTrajectoryPoint at 50Hz with a 40ms
rolling horizon — the admittance controller interpolates smoothly between
samples at its 500Hz internal rate.

Mock mode (--mock): drives a sine wave on each joint so you can validate
the full GELLO→admittance→UR5e pipeline before the hardware arrives.

Usage:
    # Real GELLO (when hardware arrives):
    ros2 run bimanual_ur5e_bringup gello_bridge.py

    # Mock mode for pipeline validation now:
    ros2 run bimanual_ur5e_bringup gello_bridge.py --ros-args -p mock:=true
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import numpy as np
import math


# ── UR5e joint names (must match your URDF / controller config) ──────────────
UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# ── Safe home position (radians) ─────────────────────────────────────────────
# Verify this is a safe pose on YOUR robot before enabling mock mode
#HOME_POS = [-1.6, -1.07, 1.01, -1.52, -1.5, -1.58]

# ── Rolling horizon ──────────────────────────────────────────────────────────
# 40ms = 2 GELLO cycles ahead at 50Hz publish rate
# Gives the 500Hz admittance controller enough lookahead to interpolate
# without accumulating lag
LOOKAHEAD_NS = 40_000_000   # 40ms in nanoseconds


class GELLOBridge(Node):

    def __init__(self):
        super().__init__('gello_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('mock',          False)
        self.declare_parameter('publish_hz',    50.0)
        self.declare_parameter('mock_amp_rad',  0.02)   # sine amplitude in mock mode
        self.declare_parameter('mock_freq_hz',  0.2)    # sine frequency in mock mode
        # Joint-space safety clamp — max allowed delta per publish cycle (rad)
        # At 50Hz, 0.01 rad/cycle = 0.5 rad/s max — safe for UR5e
        self.declare_parameter('max_delta_rad', 0.01)
        self.declare_parameter('mock_start_delay_s', 3.0)
        self._start_delay = self.get_parameter('mock_start_delay_s').value
        self._elapsed = 0.0

        self._mock        = self.get_parameter('mock').value
        pub_hz            = self.get_parameter('publish_hz').value
        self._mock_amp    = self.get_parameter('mock_amp_rad').value
        self._mock_freq   = self.get_parameter('mock_freq_hz').value
        self._max_delta   = self.get_parameter('max_delta_rad').value

        # ── Internal state ────────────────────────────────────────────────
        self._current_pos: list[float] | None = None   # latest from /joint_states
        self._gello_pos:   list[float] | None = None   # latest from GELLO topic
        self._mock_center: list[float] | None = None   # frozen center for mock oscillation
        self._last_published: list[float] | None = None   # last commanded target after rate limiting
        self._t = 0.0                                   # mock time accumulator

        # ── Subscriber: robot joint states ───────────────────────────────
        # Used to initialise the starting position and for safety clamping
        self.create_subscription(
            JointState,
            '/joint_states',
            self._cb_joint_states,
            1
        )

        # ── Subscriber: GELLO joint states ───────────────────────────────
        # GELLO publishes sensor_msgs/JointState on this topic.
        # Disabled in mock mode.
        if not self._mock:
            self.create_subscription(
                JointState,
                '/gello/joint_states',
                self._cb_gello,
                1
            )

        # ── Publisher: admittance controller reference ────────────────────
        self._pub = self.create_publisher(
            JointTrajectoryPoint,
            '/admittance_controller/joint_references',
            1   # QoS depth 1 — always send latest, never queue
        )

        # ── Timer: publish at fixed rate ─────────────────────────────────
        self._dt = 1.0 / pub_hz
        self._timer = self.create_timer(self._dt, self._publish)

        mode = "MOCK" if self._mock else "LIVE"
        self.get_logger().info(
            f'GELLO bridge started [{mode}] @ {pub_hz:.0f}Hz | '
            f'lookahead={LOOKAHEAD_NS/1e6:.0f}ms | '
            f'max_delta={self._max_delta:.3f}rad/cycle'
        )
        if self._mock:
            self.get_logger().warn(
                'Mock mode active — sine wave on all joints. '
                'Ensure robot is in a safe pose before enabling.'
            )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState):
        """Cache the latest measured joint positions, reordered to UR5e convention."""
        pos = self._reorder_joints(msg)
        if pos is not None:
            if self._current_pos is None:
                self.get_logger().info(f'First joint state received: {pos}')
            self._current_pos = pos
            if self._mock and self._mock_center is None:
                self._mock_center = pos.copy()
                self._t = 0.0
                self.get_logger().info(f'Mock center frozen at: {self._mock_center}')
        else:
            self.get_logger().warn(
                f'_reorder_joints failed. Got names: {list(msg.name)}, '
                f'expected: {UR5E_JOINT_NAMES}',
                throttle_duration_sec=2.0
            )

    def _cb_gello(self, msg: JointState):
        """Cache latest GELLO joint positions."""
        # GELLO joint names must match UR5E_JOINT_NAMES or be in same order
        if len(msg.position) == 6:
            self._gello_pos = list(msg.position)

    # ── Main publish loop ─────────────────────────────────────────────────────

    def _publish(self):
        # Wait until we have a valid robot position to start from
        if self._current_pos is None:
            return

        if self._mock:
            raw_target = self._mock_target()
        else:
            if self._gello_pos is None:
                return   # GELLO not yet publishing — wait silently
            raw_target = self._gello_pos

        if self._last_published is None:
            self._last_published = self._current_pos.copy()

        safe_target = self._rate_limit_target(self._last_published, raw_target)
        self._last_published = safe_target

        # ── Build JointTrajectoryPoint ────────────────────────────────────
        msg = JointTrajectoryPoint()
        msg.positions  = safe_target
        msg.velocities = [0.0] * 6

        # Rolling 40ms horizon:
        # "Interpolate to this position over the next 40ms"
        # We keep streaming at 50Hz so the controller always has a fresh
        # target just ahead in time — continuous smooth motion
        msg.time_from_start = Duration(
            sec=0,
            nanosec=LOOKAHEAD_NS
        )

        self._pub.publish(msg)
        # self.get_logger().info(f'Published: {[f"{p:.3f}" for p in safe_target]}',
        #     throttle_duration_sec=1.0
        # )

    # ── Mock target generator ─────────────────────────────────────────────────

    def _mock_target(self) -> list[float]:
        """
        Generates a slow sine wave offset from home position on each joint.
        Phase-offset between joints so motion looks natural, not synchronized.
        Amplitude is small (default 0.05 rad ≈ 3°) — safe for testing.
        """
        if self._mock_center is None:
            return list(self._current_pos)

        self._elapsed += self._dt
        if self._elapsed < self._start_delay:
            return list(self._mock_center)
        self._t += self._dt


        targets = []
        for i, center in enumerate(self._mock_center):
            phase = i * (math.pi / 3)   # 60° phase offset between joints
            #phase = 0.0
            t_offset = self._t - phase / (2 * math.pi * self._mock_freq)
            offset = self._mock_amp * math.sin(2 * math.pi * self._mock_freq * t_offset)
            targets.append(center + offset)
        return targets

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rate_limit_target(self,
                           previous_target: list[float],
                           new_target: list[float]) -> list[float]:
        """
        Limit how fast the commanded target can change per cycle.
        This prevents velocity spikes when GELLO jumps suddenly, without
        making the published target follow external disturbances.
        """
        limited = []
        for prev, new in zip(previous_target, new_target):
            delta = new - prev
            delta = max(-self._max_delta, min(self._max_delta, delta))
            limited.append(prev + delta)
        return limited

    def _reorder_joints(self, msg: JointState) -> list[float] | None:
        """
        Extract joint positions in UR5E_JOINT_NAMES order from a JointState.
        /joint_states doesn't guarantee ordering — this fixes that.
        Returns None if any joint is missing.
        """
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            return [name_to_pos[n] for n in UR5E_JOINT_NAMES]
        except KeyError:
            return None


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GELLOBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()