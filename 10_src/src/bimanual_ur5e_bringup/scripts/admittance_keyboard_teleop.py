#!/usr/bin/env python3

import select
import sys
import termios
import tty
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

KEY_BINDINGS = {
    "q": (0, 1.0),
    "a": (0, -1.0),
    "w": (1, 1.0),
    "s": (1, -1.0),
    "e": (2, 1.0),
    "d": (2, -1.0),
    "r": (3, 1.0),
    "f": (3, -1.0),
    "t": (4, 1.0),
    "g": (4, -1.0),
    "y": (5, 1.0),
    "h": (5, -1.0),
}


HELP_TEXT = """\
Admittance keyboard teleop
--------------------------
This publishes equilibrium joint references to /admittance_controller/joint_references.

q/a: shoulder_pan_joint    +/- step
w/s: shoulder_lift_joint   +/- step
e/d: elbow_joint           +/- step
r/f: wrist_1_joint         +/- step
t/g: wrist_2_joint         +/- step
y/h: wrist_3_joint         +/- step

z/x: decrease/increase step size
space: re-sync target to current joint state
c: print current target

CTRL-C to quit
"""


class AdmittanceKeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("admittance_keyboard_teleop")

        self.declare_parameter(
            "reference_topic", "/admittance_controller/joint_references"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("step_size_rad", 0.03)

        reference_topic = (
            self.get_parameter("reference_topic").get_parameter_value().string_value
        )
        joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self.step_size = (
            self.get_parameter("step_size_rad").get_parameter_value().double_value
        )

        self.current_positions: Optional[List[float]] = None
        self.target_positions: Optional[List[float]] = None

        self.publisher = self.create_publisher(
            JointTrajectoryPoint, reference_topic, 10
        )
        self.create_subscription(JointState, joint_state_topic, self.joint_state_cb, 10)
        self.create_timer(1.0 / publish_rate_hz, self.publish_reference)

        self.get_logger().info(HELP_TEXT)
        self.get_logger().info("Waiting for /joint_states before accepting commands.")

    def joint_state_cb(self, msg: JointState) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        if not all(name in name_to_index for name in JOINT_NAMES):
            return

        positions = [msg.position[name_to_index[name]] for name in JOINT_NAMES]
        self.current_positions = positions
        if self.target_positions is None:
            self.target_positions = positions.copy()
            self.get_logger().info("Captured current robot pose as teleop target.")

    def publish_reference(self) -> None:
        if self.target_positions is None:
            return

        msg = JointTrajectoryPoint()
        msg.positions = self.target_positions.copy()
        msg.velocities = [0.0] * len(self.target_positions)
        self.publisher.publish(msg)

    def apply_key(self, key: str) -> bool:
        if key in KEY_BINDINGS:
            if self.target_positions is None:
                self.get_logger().warn("No joint state yet; ignoring key press.")
                return False
            joint_index, direction = KEY_BINDINGS[key]
            self.target_positions[joint_index] += direction * self.step_size
            self.print_target()
            return False

        if key == "z":
            self.step_size = max(0.001, self.step_size / 2.0)
            self.get_logger().info(f"Step size: {self.step_size:.4f} rad")
            return False

        if key == "x":
            self.step_size = min(0.5, self.step_size * 2.0)
            self.get_logger().info(f"Step size: {self.step_size:.4f} rad")
            return False

        if key == " ":
            if self.current_positions is None:
                self.get_logger().warn("No joint state yet; cannot re-sync target.")
                return False
            self.target_positions = self.current_positions.copy()
            self.get_logger().info("Re-synced target to current robot pose.")
            self.print_target()
            return False

        if key == "c":
            self.print_target()
            return False

        return False

    def print_target(self) -> None:
        if self.target_positions is None:
            return
        formatted = ", ".join(
            f"{name}={value:+.3f}" for name, value in zip(JOINT_NAMES, self.target_positions)
        )
        self.get_logger().info(formatted)


def read_key(timeout: float = 0.1) -> Optional[str]:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def main() -> None:
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = AdmittanceKeyboardTeleop()

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = read_key()
            if key is None:
                continue
            if key == "\x03":
                break
            node.apply_key(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
