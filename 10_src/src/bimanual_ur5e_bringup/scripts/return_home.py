#!/usr/bin/env python3
"""
return_home.py
--------------
Sends go_home target to GELLO bridge.
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
HOME_JOINTS     = [1.7228, -1.8912, -1.1980, -1.6220, -4.7100, -1.6285]
HOME_GRIPPER_MM = 50.0

GELLO_HOME_TOPIC  = '/gello_bridge_right/go_home'
GRIPPER_TOPIC     = '/right_arm/wsg32_node/cmd_pos'


def main():
    rclpy.init()
    node = Node('return_home')

    go_home_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    home_pub = node.create_publisher(JointState, GELLO_HOME_TOPIC, go_home_qos)
    gripper_pub = node.create_publisher(Float32,    GRIPPER_TOPIC,    1)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Send home target to GELLO bridge — it handles everything from here
    # Send multiple times to guarantee delivery

    msg = JointState()
    msg.position = HOME_JOINTS
    home_pub.publish(msg)
    executor.spin_once(timeout_sec=0.02)

    print(f"Home target sent: {[f'{v:.3f}' for v in HOME_JOINTS]}")
    print("Bridge is moving robot to home at tracking speed.")
    print("Watch terminal for 'Home reached' — then match GELLO pose.")

    # Open gripper
    g_msg = Float32()
    g_msg.data = HOME_GRIPPER_MM
    gripper_pub.publish(g_msg)

    # Drain messages
    deadline = time.time() + 1.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()