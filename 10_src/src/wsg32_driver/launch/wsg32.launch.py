"""
wsg32.launch.py
---------------
Launch file for a single WSG32 gripper.

Usage:
    ros2 launch wsg32_driver wsg32.launch.py
    ros2 launch wsg32_driver wsg32.launch.py gripper_ip:=192.168.1.202 gripper_name:=wsg32_right
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments (overridable from command line) ──────────────────
    gripper_ip_arg = DeclareLaunchArgument(
        'gripper_ip',
        default_value='192.168.1.201',
        description='IP address of the WSG32 gripper on the ghost subnet'
    )
    gripper_name_arg = DeclareLaunchArgument(
        'gripper_name',
        default_value='wsg32_right',
        description='Logical name; used as joint name in JointState messages'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='right_arm',
        description='ROS namespace for this gripper node'
    )

    # ── Config file ───────────────────────────────────────────────────────
    config_file = PathJoinSubstitution([
        FindPackageShare('wsg32_driver'),
        'config',
        'wsg32_params.yaml'
    ])

    # ── Node ──────────────────────────────────────────────────────────────
    wsg32_node = Node(
        package='wsg32_driver',
        executable='wsg32_node',
        name='wsg32_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            config_file,
            {
                # Command-line overrides win over the yaml file
                'gripper_ip':   LaunchConfiguration('gripper_ip'),
                'gripper_name': LaunchConfiguration('gripper_name'),
            }
        ],
        output='screen',
        emulate_tty=True,     # colored logs in terminal
    )

    return LaunchDescription([
        gripper_ip_arg,
        gripper_name_arg,
        namespace_arg,
        wsg32_node,
    ])
