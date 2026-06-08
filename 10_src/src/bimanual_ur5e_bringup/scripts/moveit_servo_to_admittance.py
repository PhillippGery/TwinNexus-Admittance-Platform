#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class MoveItServoToAdmittanceBridge(Node):
    def __init__(self) -> None:
        super().__init__("moveit_servo_to_admittance")

        self.declare_parameter("input_topic", "/forward_position_controller/commands")
        self.declare_parameter(
            "output_topic", "/admittance_controller/joint_references"
        )
        self.declare_parameter("joint_count", 6)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.joint_count = (
            self.get_parameter("joint_count").get_parameter_value().integer_value
        )

        self.publisher = self.create_publisher(JointTrajectoryPoint, output_topic, 10)
        self.create_subscription(Float64MultiArray, input_topic, self.callback, 10)

        self.get_logger().info(
            f"Bridging MoveIt Servo output {input_topic} -> {output_topic}"
        )

    def callback(self, msg: Float64MultiArray) -> None:
        positions: List[float] = list(msg.data)
        if len(positions) != self.joint_count:
            self.get_logger().warn(
                f"Ignoring Servo output with {len(positions)} joints; expected {self.joint_count}."
            )
            return

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * self.joint_count
        self.publisher.publish(point)


def main() -> None:
    rclpy.init()
    node = MoveItServoToAdmittanceBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
