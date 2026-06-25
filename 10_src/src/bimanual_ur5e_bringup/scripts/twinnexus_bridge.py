#!/usr/bin/env python3
"""
twinnexus_bridge.py
-------------------
Generic target → admittance controller bridge for TwinNexus.

Sits between any command source (VLA policy, teleoperation, manual publish)
and the admittance controller.  Interpolates smoothly at 500Hz toward the
latest commanded target.  The admittance controller stays active at all times
so the robot remains compliant with external forces regardless of what is
publishing targets.

Architecture (mirrors gello_bridge.py):
    Separate process → own GIL → no competition with camera/inference threads.
    500Hz ROS2 timer (create_timer) → reliable cadence, same as GELLO bridge.
    Per-step delta rate limiter → identical speeds to GELLO bridge.

Topics
    ~/target_joints   [sub]  JointState  position=[j0…j5, gripper_m]  (7 values)
                             Any publisher sends here: VLA inference, manual, etc.
    ~/go_home         [sub]  JointState  position=[j0…j5, gripper_m]  (7 values)
                             TRANSIENT_LOCAL — received even if bridge starts late.
    ~/commanded_position [pub] JointState  current interpolated joint command.
                             TwinNexusRobot.go_home() polls this for convergence.
    <admittance_topic> [pub]  JointTrajectoryPoint  → admittance controller
    <gripper_topic>   [pub]  Float32 (mm)           → WSG32 gripper

Parameters (all overridable at launch for second arm)
    publish_hz          float   500.0
    joint_states_topic  str     /joint_states
    admittance_topic    str     /admittance_controller/joint_references
    gripper_topic       str     /right_arm/wsg32_node/cmd_pos
    joint_prefix        str     ""  (e.g. "right_arm_" for boot_hw_bimanual)
    tracking_delta_rad  float   0.002   (1.0 rad/s @ 500Hz — matches GELLO tracking)
    go_home_delta_rad   float   0.001   (0.5 rad/s @ 500Hz — matches GELLO bridge)

Usage
    # Right arm (launched automatically by spawnctrl):
    ros2 run bimanual_ur5e_bringup twinnexus_bridge.py \
        --ros-args -r __node:=twinnexus_bridge_right

    # Left arm:
    ros2 run bimanual_ur5e_bringup twinnexus_bridge.py \
        --ros-args -r __node:=twinnexus_bridge_left \
        -p joint_states_topic:=/left_arm/joint_states \
        -p admittance_topic:=/left_arm/admittance_controller/joint_references \
        -p joint_prefix:=left_arm_ \
        -p gripper_topic:=/left_arm/wsg32_node/cmd_pos

    # Publish a target manually (example):
    ros2 topic pub /twinnexus_bridge_right/target_joints sensor_msgs/JointState \
        "{position: [1.611, -1.392, -1.494, -1.627, -4.61, -1.732, 0.05]}"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectoryPoint

# ── UR5e joint names (kinematic order) ───────────────────────────────────────
UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

LOOKAHEAD_NS = 2_000_000   # 2ms — same as GELLO bridge


class TwinNexusBridge(Node):

    def __init__(self):
        super().__init__('twinnexus_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('publish_hz',           500.0)
        self.declare_parameter('joint_states_topic',   '/joint_states')
        self.declare_parameter('admittance_topic',     '/admittance_controller/joint_references')
        self.declare_parameter('gripper_topic',        '/right_arm/wsg32_node/cmd_pos')
        self.declare_parameter('joint_prefix',         '')
        self.declare_parameter('tracking_delta_rad',    0.002)
        self.declare_parameter('go_home_delta_rad',     0.001)
        self.declare_parameter('approach_threshold_rad', 0.05)

        pub_hz                   = self.get_parameter('publish_hz').value
        joint_states_topic       = self.get_parameter('joint_states_topic').value
        admittance_topic         = self.get_parameter('admittance_topic').value
        gripper_topic            = self.get_parameter('gripper_topic').value
        joint_prefix             = self.get_parameter('joint_prefix').value
        self._joint_names        = [f'{joint_prefix}{n}' for n in UR5E_JOINT_NAMES]
        self._tracking_delta     = self.get_parameter('tracking_delta_rad').value
        self._go_home_delta      = self.get_parameter('go_home_delta_rad').value
        self._approach_threshold = self.get_parameter('approach_threshold_rad').value
        self._dt                 = 1.0 / pub_hz

        # ── State ─────────────────────────────────────────────────────────
        self._current_pos:    list[float] | None = None  # latest from /joint_states
        self._last_pub:       list[float] | None = None  # last commanded position
        self._target:         list[float] | None = None  # inference/manual target (6 joints)
        self._target_gripper: float | None       = None  # gripper target (metres)
        self._home_target:    list[float] | None = None  # go_home target (6 joints)
        self._home_gripper:   float | None       = None  # go_home gripper (metres)
        # Latches True the first time gap drops below approach_threshold.
        # Never reverts to approach speed until go_home resets it.
        self._tracking_active: bool = False
        self._home_hold_timer = None

        # ── QoS for go_home — durable so bridge receives it even if late ──
        go_home_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(JointState, joint_states_topic, self._cb_joints, 1)
        self.create_subscription(JointState, '~/target_joints',  self._cb_target, 1)
        self.create_subscription(JointState, '~/go_home',        self._cb_go_home, go_home_qos)

        # ── Publishers ────────────────────────────────────────────────────
        self._pub             = self.create_publisher(JointTrajectoryPoint, admittance_topic, 1)
        self._gripper_pub     = self.create_publisher(Float32,     gripper_topic, 1)
        self._status_pub      = self.create_publisher(JointState, '~/commanded_position', 1)

        # ── 500Hz timer ───────────────────────────────────────────────────
        self.create_timer(self._dt, self._publish)

        self.get_logger().info(
            f'TwinNexus bridge @ {pub_hz:.0f}Hz | '
            f'joints→{admittance_topic} | gripper→{gripper_topic} | '
            f'tracking_delta={self._tracking_delta:.4f} rad/step | '
            f'go_home_delta={self._go_home_delta:.4f} rad/step'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_joints(self, msg: JointState) -> None:
        """Update current robot position from /joint_states."""
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._current_pos = [name_to_pos[n] for n in self._joint_names]
        except KeyError:
            self.get_logger().warn(
                f'Joint names mismatch. Got: {list(msg.name)}',
                throttle_duration_sec=2.0,
            )

    def _cb_target(self, msg: JointState) -> None:
        """Receive a new tracking target from any publisher (policy, manual, etc.).

        JointState.position must have 7 values: [j0…j5, gripper_metres].
        go_home has priority — this target is queued but won't activate until
        homing is complete.
        """
        if len(msg.position) < 7:
            self.get_logger().warn(
                f'target_joints needs 7 values (6 joints + gripper), got {len(msg.position)}',
                throttle_duration_sec=2.0,
            )
            return
        self._target         = list(msg.position[:6])
        self._target_gripper = float(msg.position[6])

    def _cb_go_home(self, msg: JointState) -> None:
        """Trigger homing.  go_home takes priority over tracking target."""
        if len(msg.position) < 7:
            self.get_logger().warn(
                f'go_home needs 7 values (6 joints + gripper_m), got {len(msg.position)}',
                throttle_duration_sec=2.0,
            )
            return
        self._home_target     = list(msg.position[:6])
        self._home_gripper    = float(msg.position[6])
        self._tracking_active = False  
        
        # Cancel any active hold timer if homing is re-triggered
        if self._home_hold_timer is not None:
            self.get_logger().info('Cancelling active hold timer due to new go_home command.')
            self._home_hold_timer.cancel()
            self.destroy_timer(self._home_hold_timer)
            self._home_hold_timer = None
            
            self.get_logger().info(
            f'Go home: {[f"{v:.3f}" for v in self._home_target]}'
        )
    def _cb_home_hold_expired(self) -> None:
        """Triggered exactly 3 seconds after reaching home position."""
        # 1. Grab a local reference and instantly clear the class property
        timer = self._home_hold_timer
        if timer is None:
            return
        self._home_hold_timer = None

        # 2. Immediately cancel and destroy the timer before any other processing
        try:
            timer.cancel()
            self.destroy_timer(timer)
        except Exception as e:
            self.get_logger().error(f'Failed to destroy timer: {e}')

        # 3. Execute the rest of your logic safely
        self.get_logger().info('Hold expired. Ready for new commands.')
        self._target = None
        self._home_target = None
        self._tracking_active = False


    # ── 500Hz publish loop ────────────────────────────────────────────────────

    def _publish(self) -> None:
        if self._current_pos is None:
            return  # no joint states yet — wait

        # Seed last_pub from actual robot position on first active call
        if self._last_pub is None:
            self._last_pub = list(self._current_pos)

        # ── Priority 1: go_home ───────────────────────────────────────────
        if self._home_target is not None:
            max_gap = max(abs(h - p) for h, p in zip(self._home_target, self._last_pub))
            # home gripper

            if self._home_gripper is not None:
                self._publish_gripper(self._home_gripper)

            if max_gap < 0.01:
                self.get_logger().info('Home reached.', throttle_duration_sec=2.0)
                # Keep publishing home position but clear the active target
                # self._home_target = None
                # # Clear tracking target too — caller sets a new one after go_home
                # self._target = None

                self._home_hold_timer = self.create_timer(4.0, self._cb_home_hold_expired)
            else:
                self._last_pub = self._rate_limit(
                    self._last_pub, self._home_target, self._go_home_delta
                )
               


            
            self._publish_joints(self._last_pub)
            self._publish_status(self._last_pub)
            return

        # ── Priority 2: tracking target ───────────────────────────────────
        if self._target is not None:
            # Approach phase: use go_home speed until gap first drops below threshold,
            # then latch to tracking speed permanently (until next go_home resets it).
            if not self._tracking_active:
                max_gap = max(abs(t - p) for t, p in zip(self._target, self._last_pub))
                if max_gap <= self._approach_threshold:
                    self._tracking_active = True
            delta = self._tracking_delta if self._tracking_active else self._go_home_delta
            self._last_pub = self._rate_limit(self._last_pub, self._target, delta)
            if self._target_gripper is not None:
                self._publish_gripper(self._target_gripper)
            self._publish_joints(self._last_pub)
            self._publish_status(self._last_pub)
            return

        # ── Priority 3: no target — hold last commanded position ──────────
        # Always publish so the admittance controller reference tracks _last_pub.
        # Without this, the controller keeps a stale reference from the trajectory
        # controller handoff, causing a large velocity spike on the first go_home publish.
        self._publish_joints(self._last_pub)
        self._publish_status(self._last_pub)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rate_limit(self,
                    previous: list[float],
                    target:   list[float],
                    max_d:    float) -> list[float]:
        return [
            prev + max(-max_d, min(max_d, goal - prev))
            for prev, goal in zip(previous, target)
        ]

    def _publish_joints(self, positions: list[float]) -> None:
        pt = JointTrajectoryPoint()
        pt.positions       = positions
        pt.velocities      = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=LOOKAHEAD_NS)
        self._pub.publish(pt)

    def _publish_gripper(self, gripper_m: float) -> None:
        msg = Float32()
        msg.data = float(gripper_m * 1000.0)   # m → mm for WSG32
        self._gripper_pub.publish(msg)

    def _publish_status(self, positions: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self._joint_names
        msg.position     = positions
        self._status_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TwinNexusBridge()
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
