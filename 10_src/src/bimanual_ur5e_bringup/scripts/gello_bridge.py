#!/usr/bin/env python3
"""
gello_bridge.py
---------------
GELLO teleoperation bridge for TwinNexus.

Reads GELLO joint positions and publishes them as raw targets to the
TwinNexus bridge, which owns all rate limiting and admittance controller
publishing.  This bridge only controls WHEN to start streaming (startup
hold + delta check) — not HOW FAST the robot moves.

Startup sequence:
    1. First /joint_states → freeze robot's initial position, init GELLO.
    2. Hold for startup_hold_s seconds (publish nothing to TwinNexus).
       Display GELLO vs Robot delta every 10s so user can align.
    3. After hold expires: check max delta.
       If delta > max_initial_delta_rad → warn, reset hold, wait again.
       If delta ≤ max_initial_delta_rad → release hold, start streaming.
    4. Publish raw 7-value JointState (joints + gripper) to TwinNexus
       ~/target_joints.  TwinNexus bridge handles:
         - Approach at go_home speed (gap > approach_threshold)
         - Switch to tracking speed (gap ≤ approach_threshold)
         - Admittance controller publishing at 500Hz
         - Gripper publishing

Go home:
    return_home.py sends to both /gello_bridge_right/go_home and
    /twinnexus_bridge_right/go_home.  TwinNexus moves the robot home.
    This bridge enters hold mode so user must re-align GELLO before resuming.

Usage:
    # Right arm (default):
    ros2 run bimanual_ur5e_bringup gello_bridge.py \\
      --ros-args -r __node:=gello_bridge_right

    # Left arm:
    ros2 run bimanual_ur5e_bringup gello_bridge.py \\
      --ros-args -r __node:=gello_bridge_left \\
        -p gello_port:=/dev/serial/by-id/usb-FTDI_..._FTXXXXXX-if00-port0 \\
        -p joint_offsets:=[0.0,3.14,-1.57,4.71,7.85,4.71] \\
        -p joint_signs:=[1,1,-1,1,1,1] \\
        -p gripper_config:=[7,115.4,73.6] \\
        -p joint_states_topic:=/left_arm/joint_states \\
        -p target_topic:=/twinnexus_bridge_left/target_joints

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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState


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


class GELLOBridge(Node):

    def __init__(self):
        super().__init__('gello_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter(
            'gello_port',
            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4WDM-if00-port0'
        )
        self.declare_parameter(
            'joint_offsets',
            [0.0, 3.14159, -1.5708, 4.7124, 7.8540, 4.7124]
        )
        self.declare_parameter('joint_signs',   [1.0, 1.0, -1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('gripper_config', [7.0, 115.4, 73.6])

        # Topics
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_topic',       '/twinnexus_bridge_right/target_joints')
        self.declare_parameter('joint_prefix',       '')

        # Control
        self.declare_parameter('publish_hz',           500.0)
        self.declare_parameter('startup_hold_s',         3.0)
        self.declare_parameter('max_initial_delta_rad',  0.3)

        # ── Read parameters ───────────────────────────────────────────────
        self._gello_port  = self.get_parameter('gello_port').value
        raw_offsets       = self.get_parameter('joint_offsets').value
        raw_signs         = self.get_parameter('joint_signs').value
        raw_gripper       = self.get_parameter('gripper_config').value
        joint_states_topic= self.get_parameter('joint_states_topic').value
        target_topic      = self.get_parameter('target_topic').value
        joint_prefix      = self.get_parameter('joint_prefix').value
        pub_hz            = self.get_parameter('publish_hz').value
        self._hold_s      = self.get_parameter('startup_hold_s').value
        self._max_initial_delta = self.get_parameter('max_initial_delta_rad').value
        self._dt          = 1.0 / pub_hz

        self._joint_offsets  = tuple(float(v) for v in raw_offsets)
        self._joint_signs    = tuple(int(v)   for v in raw_signs)
        self._gripper_config = (int(raw_gripper[0]), float(raw_gripper[1]), float(raw_gripper[2]))
        self._joint_names    = [f'{joint_prefix}{n}' for n in UR5E_JOINT_NAMES]

        # ── State ─────────────────────────────────────────────────────────
        self._current_pos:  list[float] | None = None
        self._initial_pos:  list[float] | None = None
        self._gello:        object | None       = None
        self._gello_ready:  bool                = False
        self._elapsed:      float               = 0.0
        self._hold_active:  bool                = True

        # ── ROS interfaces ────────────────────────────────────────────────
        go_home_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(JointState, joint_states_topic, self._cb_joint_states, 1)
        self.create_subscription(JointState, '~/go_home', self._cb_go_home, go_home_qos)

        # Single publisher — raw GELLO target to TwinNexus bridge
        self._target_pub = self.create_publisher(JointState, target_topic, 1)

        self.create_timer(self._dt, self._publish)

        self.get_logger().info(
            f'GELLO bridge @ {pub_hz:.0f}Hz | port={self._gello_port.split("/")[-1]} | '
            f'→ {target_topic}'
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
            self._gello = GelloAgent(
                port=self._gello_port,
                dynamixel_config=config,
                start_joints=np.append(self._initial_pos[:6], 0.0),
            )
            self.get_logger().info(
                f'GELLO initialized | '
                f'offsets={[f"{v:.3f}" for v in self._joint_offsets]} | '
                f'signs={list(self._joint_signs)}'
            )
            return True
        except Exception as e:
            self.get_logger().fatal(f'GELLO init failed: {e}\n{traceback.format_exc()}')
            return False

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            pos = [name_to_pos[n] for n in self._joint_names]
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
                f'Initial robot position frozen: {[f"{v:.3f}" for v in self._initial_pos]}'
            )
            self._gello_ready = self._init_gello()
            if not self._gello_ready:
                raise SystemExit(1)

    def _cb_go_home(self, msg: JointState) -> None:
        # TwinNexus bridge handles the actual homing movement.
        # Reset hold so user must re-align GELLO before resuming streaming.
        self._hold_active = True
        self._elapsed     = 0.0
        self.get_logger().info('Go home received — hold active until GELLO is re-aligned.')

    # ── Publish loop ──────────────────────────────────────────────────────────

    def _publish(self) -> None:
        if self._initial_pos is None or not self._gello_ready:
            return

        result = self._gello_target()
        if result is None:
            return

        gello_joints, gripper = result

        # ── Hold phase ────────────────────────────────────────────────────
        if self._hold_active:
            self._elapsed += self._dt

            self.get_logger().info(
                f'\n'
                f'  GELLO : {[f"{v:+.3f}" for v in gello_joints]}\n'
                f'  Robot : {[f"{v:+.3f}" for v in self._current_pos]}\n'
                f'  Delta : {[f"{g-r:+.3f}" for g, r in zip(gello_joints, self._current_pos)]}',
                throttle_duration_sec=10.0,
            )

            if not math.isinf(self._hold_s) and self._elapsed >= self._hold_s:
                max_delta = max(
                    abs(g - r)
                    for g, r in zip(gello_joints, self._current_pos)
                )
                if max_delta > self._max_initial_delta:
                    self.get_logger().warn(
                        f'HOLD NOT RELEASED — max delta {max_delta:.3f} rad '
                        f'exceeds {self._max_initial_delta:.3f} rad. '
                        f'Move GELLO closer to robot pose.',
                        throttle_duration_sec=2.0,
                    )
                    self._elapsed = 0.0
                else:
                    self._hold_active = False
                    self.get_logger().info(
                        f'Hold released (max delta={max_delta:.3f} rad). '
                        f'Streaming GELLO → TwinNexus bridge.'
                    )
            return  # never publish during hold

        # ── Stream raw target to TwinNexus bridge ─────────────────────────
        # TwinNexus owns all rate limiting:
        #   gap > approach_threshold → go_home speed (0.001 rad/step)
        #   gap ≤ approach_threshold → tracking speed (0.002 rad/step)
        msg = JointState()
        msg.position = gello_joints + [gripper]   # 7 values: joints + gripper_m
        self._target_pub.publish(msg)

    # ── GELLO reader ──────────────────────────────────────────────────────────

    def _gello_target(self) -> tuple[list[float], float] | None:
        try:
            state   = self._gello.act({})
            joints  = state[:6].tolist()
            gripper = float(state[6])   # 0.0=open … 1.0=closed
            return joints, gripper
        except Exception as e:
            self.get_logger().warn(
                f'GELLO read error: {e}',
                throttle_duration_sec=1.0,
            )
            return None


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
