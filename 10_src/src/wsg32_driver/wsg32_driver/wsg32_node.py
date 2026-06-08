"""
wsg32_node.py
-------------
ROS 2 Jazzy node for the Weiss WSG 32 gripper.

Subscribes to a position command topic and forwards it to the gripper
via the GCL TCP driver. Also publishes gripper state and exposes
homing / fault-clear services.

Topics
------
Subscribed:
  ~/cmd_pos   [std_msgs/Float32]
      Target gripper width in mm (0.0 = closed, 55.0 = open).
      Published by your GELLO teleoperation node.

Published:
  ~/joint_state   [sensor_msgs/JointState]
      Gripper reported as a single prismatic joint.
      name: ["wsg32_jaw"]
      position: [width_in_meters]   ← what rosbag2 records for π₀ training

Services
--------
  ~/home       [std_srvs/Trigger]   Run homing sequence (blocks ~5 s)
  ~/ack_fault  [std_srvs/Trigger]   Clear latched fault

Parameters
----------
  gripper_ip       (string,  default "192.168.1.201")
  gripper_port     (int,     default 1000)
  gripper_name     (string,  default "wsg32_right")   used in joint state name
  max_pos_mm       (float,   default 66.0)      physical stroke limit
  cmd_deadband_mm  (float,   default 0.2)       ignore moves smaller than this
  publish_hz       (float,   default 50.0)      state publish rate

Usage
-----
  # Single gripper:
  ros2 run wsg32_driver wsg32_node

  # With custom IP (e.g. second gripper):
  ros2 run wsg32_driver wsg32_node --ros-args \\
      -r __ns:=/right_arm \\
      -p gripper_ip:=192.168.1.201 \\
      -p gripper_name:=wsg32_right
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

from wsg32_driver.wsg_tcp import WSG32TCP


class WSG32Node(Node):

    def __init__(self):
        super().__init__('wsg32_node')

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter('gripper_ip',      '192.168.1.201')
        self.declare_parameter('gripper_port',    1000)
        self.declare_parameter('gripper_name',    'wsg32_right')
        self.declare_parameter('max_pos_mm',      66.0)
        self.declare_parameter('cmd_deadband_mm', 0.2)
        self.declare_parameter('publish_hz',      50.0)

        ip       = self.get_parameter('gripper_ip').value
        port     = self.get_parameter('gripper_port').value
        self._name      = self.get_parameter('gripper_name').value
        max_pos  = self.get_parameter('max_pos_mm').value
        self._deadband  = self.get_parameter('cmd_deadband_mm').value
        pub_hz   = self.get_parameter('publish_hz').value

        # ── Internal state ────────────────────────────────────────────────
        self._last_cmd_mm: float = -1.0   # sentinel: no command yet

        # ── Hardware driver ───────────────────────────────────────────────
        self._driver = WSG32TCP(ip=ip, port=port)
        self._driver.MAX_POS_MM = max_pos

        self.get_logger().info(f'[{self._name}] Connecting to {ip}:{port} ...')
        try:
            self._driver.connect()
        except ConnectionError as e:
            self.get_logger().fatal(str(e))
            raise SystemExit(1)

        self.get_logger().info(f'[{self._name}] Connected. Running homing ...')
        if not self._driver.home():
            self.get_logger().error(
                f'[{self._name}] Homing failed. '
                'Gripper may not be in reference position!'
            )
        else:
            self.get_logger().info(f'[{self._name}] Homing complete.')

        # ── ROS interfaces ────────────────────────────────────────────────

        # Subscriber: receive position commands from GELLO node
        self._sub_cmd = self.create_subscription(
            Float32,
            '~/cmd_pos',
            self._cb_cmd_pos,
            10                  # QoS depth
        )

        # Publisher: joint state for rosbag2 recording
        self._pub_state = self.create_publisher(
            JointState,
            '~/joint_state',
            10
        )

        # Services
        self.create_service(Trigger, '~/home',      self._srv_home)
        self.create_service(Trigger, '~/ack_fault', self._srv_ack_fault)

        # Timer: publish state at fixed rate
        # NOTE: we don't read position back from the gripper in this minimal
        # driver (GCL has no async state push without the AUTOUPDATE command).
        # We publish the last commanded position as the state.
        # This is fine for π₀ training — the commanded position IS the action.
        self._timer = self.create_timer(
            1.0 / pub_hz,
            self._publish_state
        )

        self.get_logger().info(
            f'[{self._name}] Ready. '
            f'Listening on ~/cmd_pos, publishing ~/joint_state @ {pub_hz} Hz.'
        )

    # ── Subscriber callback ───────────────────────────────────────────────

    def _cb_cmd_pos(self, msg: Float32) -> None:
        """
        Called every time the GELLO node publishes a new gripper width target.

        Deadband filter: if the new command is within deadband_mm of the
        last sent command, skip it. This prevents spamming the TCP socket
        with tiny useless moves caused by GELLO encoder noise.
        """
        target_mm = float(msg.data)

        # Deadband check
        if abs(target_mm - self._last_cmd_mm) < self._deadband:
            return

        self._last_cmd_mm = target_mm

        try:
            # NON-BLOCKING: fire-and-forget, return immediately.
            # The gripper will track the target while we accept the next cmd.
            self._driver.move_nonblocking(target_mm)
        except Exception as e:
            self.get_logger().warn(f'[{self._name}] move failed: {e}')

    # ── State publisher ───────────────────────────────────────────────────

    def _publish_state(self) -> None:
        """
        Publish the last commanded position as a JointState message.
        rosbag2 records this alongside robot joint states for the π₀ dataset.
        """
        if self._last_cmd_mm < 0.0:
            return   # no command received yet, nothing to publish

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = [f'{self._name}_jaw']
        msg.position = [self._last_cmd_mm / 1000.0]   # mm → meters
        msg.velocity = [0.0]
        msg.effort   = [0.0]
        self._pub_state.publish(msg)

    # ── Service callbacks ─────────────────────────────────────────────────

    def _srv_home(self, _req, response: Trigger.Response) -> Trigger.Response:
        """Re-home the gripper on demand (e.g., after a fault recovery)."""
        self.get_logger().info(f'[{self._name}] Homing requested via service.')
        ok = self._driver.home()
        response.success = ok
        response.message = 'Homing complete.' if ok else 'Homing failed — check gripper state.'
        return response

    def _srv_ack_fault(self, _req, response: Trigger.Response) -> Trigger.Response:
        """Clear a latched fault (fast-stop, overcurrent, etc.)."""
        ok = self._driver.ack_fault()
        response.success = ok
        response.message = 'Fault cleared.' if ok else 'Fault clear failed.'
        return response

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info(f'[{self._name}] Shutting down, disconnecting gripper.')
        self._driver.disconnect()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WSG32Node()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
