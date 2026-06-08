#!/usr/bin/env python3

import sys

import rclpy
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node


class ServoCommandTypeSetter(Node):
    def __init__(self) -> None:
        super().__init__("set_servo_command_type")

        self.declare_parameter("service_name", "/servo_node/switch_command_type")

        service_name = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )
        self.client = self.create_client(ServoCommandType, service_name)

    def run(self) -> int:
        if not self.client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("Timed out waiting for Servo command-type service.")
            return 1

        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("Servo command-type service call failed.")
            return 1

        if not future.result().success:
            self.get_logger().error("Servo rejected the Twist command type request.")
            return 1

        self.get_logger().info("MoveIt Servo command type set to Twist.")
        return 0


def main() -> None:
    rclpy.init()
    node = ServoCommandTypeSetter()
    exit_code = 1
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
