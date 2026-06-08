"""
wsg32_dual.launch.py
--------------------
Launch both WSG32 grippers for the TwinNexus dual-arm rig.

Left gripper:   192.168.1.201  →  namespace /right_arm  →  joint wsg32_rigth_jaw
Right gripper:  192.168.1.202  →  namespace /left_arm →  joint wsg32_left_jaw

Usage:
    ros2 launch wsg32_driver wsg32_dual.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare('wsg32_driver'),
        'config',
        'wsg32_params.yaml'
    ])

    left_gripper = Node(
        package='wsg32_driver',
        executable='wsg32_node',
        name='wsg32_node',
        namespace='left_arm',
        parameters=[
            config_file,
            {
                'gripper_ip':   '192.168.1.201',
                'gripper_name': 'wsg32_right',
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    right_gripper = Node(
        package='wsg32_driver',
        executable='wsg32_node',
        name='wsg32_node',
        namespace='right_arm',
        parameters=[
            config_file,
            {
                'gripper_ip':   '192.168.1.202',
                'gripper_name': 'wsg32_left',
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        left_gripper,
        right_gripper,
    ])
