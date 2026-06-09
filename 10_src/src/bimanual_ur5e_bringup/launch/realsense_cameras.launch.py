"""
realsense_cameras.launch.py
---------------------------
Launches all three RealSense D415 cameras for the TwinNexus platform.

Camera assignment:
  wrist_left   serial 151322062583  → /wrist_left/color/image_raw
  wrist_right  serial 151422060684  → /wrist_right/color/image_raw
  overhead     serial XXXXXXXXXX    → /overhead/color/image_raw  ← fill in when known

Each camera runs as an independent node in its own namespace.
Only RGB stream is enabled — depth is disabled to reduce USB3 bandwidth.
All three cameras share one USB3 controller; running depth on all three
simultaneously risks dropping frames. Enable per-camera if needed.

Usage:
    ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py

    # Enable depth on wrist cameras if needed:
    ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py enable_depth:=true

    # Override a serial number at launch time:
    ros2 launch bimanual_ur5e_bringup realsense_cameras.launch.py overhead_serial:=150123456789
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


# ── Serial numbers ────────────────────────────────────────────────────────────
SERIAL_WRIST_LEFT  = '151322062583'
SERIAL_WRIST_RIGHT = '151422060684'
SERIAL_OVERHEAD    = 'XXXXXXXXXX'    # TODO: fill in when overhead camera is assigned

# ── Stream config ─────────────────────────────────────────────────────────────
# Resolution and FPS for RGB stream.
# 1280x720 @ 30fps is the safest choice for three simultaneous cameras on USB3.
# Increase  only if you have confirmed bandwidth headroom.
COLOR_WIDTH  = 1280
COLOR_HEIGHT = 720
COLOR_FPS    = 30


def make_camera_node(camera_name: str, serial: str,
                     enable_depth: LaunchConfiguration) -> GroupAction:
    """
    Creates a namespaced RealSense node for one camera.

    Published topics (under /<camera_name>/):
      color/image_raw          sensor_msgs/Image       RGB frame
      color/camera_info        sensor_msgs/CameraInfo  intrinsics
      depth/image_rect_raw     sensor_msgs/Image       depth (if enabled)
      depth/camera_info        sensor_msgs/CameraInfo  depth intrinsics (if enabled)
    """
    return GroupAction([
        PushRosNamespace(camera_name),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                # ── Identity ──────────────────────────────────────────────
                'serial_no':     serial,
                'camera_name':   camera_name,

                # ── RGB stream ────────────────────────────────────────────
                'enable_color':        True,
                'color_width':         COLOR_WIDTH,
                'color_height':        COLOR_HEIGHT,
                'color_fps':           COLOR_FPS,

                # ── Depth stream ──────────────────────────────────────────
                # Disabled by default — enable per-camera if needed
                'enable_depth':        enable_depth,
                'depth_width':         640,
                'depth_height':        480,
                'depth_fps':           30,

                # ── Disable unused streams to save bandwidth ──────────────
                'enable_infra1':       False,
                'enable_infra2':       False,
                'enable_accel':        False,
                'enable_gyro':         False,
                'enable_pointcloud':   False,

                # ── Alignment ─────────────────────────────────────────────
                # Align depth to color frame — needed if depth is used
                # with the TwinNexusRobot observation dict
                'align_depth.enable':  False,
            }],
            output='screen',
            emulate_tty=True,
        )
    ])


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='false',
        description='Enable depth stream on all cameras (increases USB3 bandwidth)'
    )
    overhead_serial_arg = DeclareLaunchArgument(
        'overhead_serial',
        default_value=SERIAL_OVERHEAD,
        description='Serial number of the overhead camera'
    )

    enable_depth   = LaunchConfiguration('enable_depth')
    overhead_serial = LaunchConfiguration('overhead_serial')

    # ── Camera nodes ──────────────────────────────────────────────────────────
    wrist_left = make_camera_node(
        camera_name='wrist_left',
        serial=SERIAL_WRIST_LEFT,
        enable_depth=enable_depth,
    )

    wrist_right = make_camera_node(
        camera_name='wrist_right',
        serial=SERIAL_WRIST_RIGHT,
        enable_depth=enable_depth,
    )

    # Overhead uses the launch argument so serial can be overridden at runtime
    # without editing this file — useful until the overhead camera is assigned
    overhead = GroupAction([
        PushRosNamespace('overhead'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                'serial_no':           overhead_serial,
                'camera_name':         'overhead',
                'enable_color':        True,
                'color_width':         COLOR_WIDTH,
                'color_height':        COLOR_HEIGHT,
                'color_fps':           COLOR_FPS,
                'enable_depth':        enable_depth,
                'depth_width':         640,
                'depth_height':        480,
                'depth_fps':           30,
                'enable_infra1':       False,
                'enable_infra2':       False,
                'enable_accel':        False,
                'enable_gyro':         False,
                'enable_pointcloud':   False,
                'align_depth.enable':  False,
            }],
            output='screen',
            emulate_tty=True,
        )
    ])

    return LaunchDescription([
        enable_depth_arg,
        overhead_serial_arg,
        wrist_left,
        wrist_right,
        overhead,
    ])