#!/usr/bin/env python3
"""
return_home.py
--------------
Sends go_home target to active TwinNexus / GELLO bridges.
The bridge handles the 500Hz interpolation with its own rate limiter.

Usage:
    go_home
"""

import time
import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ── Configuration ─────────────────────────────────────────────────────────────
HOME_JOINTS     = [1.611, -1.392, -1.494, -1.627, -4.61, -1.732]
HOME_GRIPPER_MM = 50.0

# Left arm home — update after left-arm calibration if needed.
LEFT_HOME_JOINTS     = [-0.272, -1.6385, 1.2859, -1.414, -1.5339, 1.492]
LEFT_HOME_GRIPPER_MM = HOME_GRIPPER_MM

GELLO_HOME_TOPIC_RIGHT     = '/right_arm/gello_bridge_right/go_home'
GELLO_HOME_TOPIC_LEFT      = '/left_arm/gello_bridge_left/go_home'
TWINNEXUS_HOME_TOPIC_RIGHT = '/right_arm/twinnexus_bridge_right/go_home'
TWINNEXUS_HOME_TOPIC_LEFT  = '/left_arm/twinnexus_bridge_left/go_home'
GRIPPER_TOPIC_RIGHT        = '/right_arm/wsg32_node/cmd_pos'
GRIPPER_TOPIC_LEFT         = '/left_arm/wsg32_node/cmd_pos'


def main():
    rclpy.init()
    node = Node('return_home')

    go_home_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    pubs = [
        node.create_publisher(JointState, GELLO_HOME_TOPIC_RIGHT, go_home_qos),
        node.create_publisher(JointState, GELLO_HOME_TOPIC_LEFT, go_home_qos),
        node.create_publisher(JointState, TWINNEXUS_HOME_TOPIC_RIGHT, go_home_qos),
        node.create_publisher(JointState, TWINNEXUS_HOME_TOPIC_LEFT, go_home_qos),
        node.create_publisher(Float32, GRIPPER_TOPIC_RIGHT, 1),
        node.create_publisher(Float32, GRIPPER_TOPIC_LEFT, 1),
    ]

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def _home_msg(joints, gripper_mm):
        msg = JointState()
        msg.position = joints + [gripper_mm / 1000.0]
        return msg

    def _gripper_msg(gripper_mm):
        msg = Float32()
        msg.data = gripper_mm
        return msg

    right_home = _home_msg(HOME_JOINTS, HOME_GRIPPER_MM)
    left_home = _home_msg(LEFT_HOME_JOINTS, LEFT_HOME_GRIPPER_MM)

    pubs[0].publish(right_home)
    pubs[1].publish(left_home)
    pubs[2].publish(right_home)
    pubs[3].publish(left_home)
    pubs[4].publish(_gripper_msg(HOME_GRIPPER_MM))
    pubs[5].publish(_gripper_msg(LEFT_HOME_GRIPPER_MM))
    executor.spin_once(timeout_sec=0.02)

    print(f"Right home target sent: {[f'{v:.3f}' for v in HOME_JOINTS]}")
    print(f"Left home target sent:  {[f'{v:.3f}' for v in LEFT_HOME_JOINTS]}")

    deadline = time.time() + 2.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
