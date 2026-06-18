"""
ur5e_upstream_admittance_bimanual.launch.py
-------------------------------------------
TwinNexus inference / policy control for both UR5e arms.

Usage (after boot_hw_bimanual + Play on both teach pendants):
    # Controller only (policy / manual targets):
    spawnctrl_bimanual

    # Controller + GELLO teleoperation (RIGHT arm, confirm pipeline):
    spawnctrl_bimanual teleop:=true

GELLO bridge starts in hold mode — robot does not move until:
  1. Hold timer (3s) expires AND
  2. Max GELLO-robot joint delta < max_initial_delta_rad (default 0.5 rad)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ARM_ADMITTANCE = os.path.join(os.path.dirname(__file__), "_ur5e_admittance_arm.launch.py")
_PKG_SHARE = get_package_share_directory("bimanual_ur5e_bringup")


def _arm_include(side: str, gripper_ip: str, gripper_name: str) -> IncludeLaunchDescription:
    ns = f"{side}_arm"
    tf_prefix = f"{side}_arm_"
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(_ARM_ADMITTANCE),
        launch_arguments={
            "arm_namespace": ns,
            "controller_manager": f"/{ns}/controller_manager",
            "admittance_params_file": (
                f"{_PKG_SHARE}/config/ur5e_admittance_controller_{side}.yaml"
            ),
            "gripper_ip": gripper_ip,
            "gripper_name": gripper_name,
            "bridge_name": f"twinnexus_bridge_{side}",
            "joint_states_topic": f"/{ns}/joint_states",
            "admittance_topic": f"/{ns}/admittance_controller/joint_references",
            "gripper_topic": f"/{ns}/wsg32_node/cmd_pos",
            "joint_prefix": tf_prefix,
        }.items(),
    )


def generate_launch_description():
    teleop             = LaunchConfiguration('teleop')
    max_initial_delta  = LaunchConfiguration('max_initial_delta_rad')

    # GELLO bridges — launched when teleop:=true.
    # 10s delay gives admittance spawner + TwinNexus bridges time to start.
    # Each GELLO bridge has a 3s hold before any motion begins.
    gello_bridges = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='bimanual_ur5e_bringup',
                executable='gello_bridge.py',
                name='gello_bridge_right',
                namespace='right_arm',
                condition=IfCondition(teleop),
                parameters=[{
                    'gello_port':            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO51SL-if00-port0',
                    'joint_offsets':         [0.0, 3.14159, 3.14159, 7.854, 9.425, 7.854],
                    'joint_signs':           [1.0, 1.0, -1.0, 1.0, 1.0, 1.0],
                    'gripper_config':        [7.0,  159.357, 201.157],
                    'joint_states_topic':    '/right_arm/joint_states',
                    'target_topic':          '/right_arm/twinnexus_bridge_right/target_joints',
                    'joint_prefix':          'right_arm_',
                    'publish_hz':             30.0,
                    'startup_hold_s':         3.0,
                    'max_initial_delta_rad':  max_initial_delta,
                }],
                output='screen',
                emulate_tty=True,
            ),
            Node(
                package='bimanual_ur5e_bringup',
                executable='gello_bridge.py',
                name='gello_bridge_left',
                namespace='left_arm',
                condition=IfCondition(teleop),
                parameters=[{
                    'gello_port':            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4WDM-if00-port0',
                    'joint_offsets':         [4.71239, 3.14159, 4.71239, 4.71239, 1.5708, 1.5708],
                    'joint_signs':           [1.0, 1.0, -1.0, 1.0, 1.0, 1.0],
                    'gripper_config':        [7.0, 74.455, 116.255],
                    'joint_states_topic':    '/left_arm/joint_states',
                    'target_topic':          '/left_arm/twinnexus_bridge_left/target_joints',
                    'joint_prefix':          'left_arm_',
                    'publish_hz':             30.0,
                    'startup_hold_s':         3.0,
                    'max_initial_delta_rad':  max_initial_delta,
                }],
                output='screen',
                emulate_tty=True,
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Launch GELLO bridges for teleoperation (true/false).',
        ),
        DeclareLaunchArgument(
            'max_initial_delta_rad',
            default_value='0.5',
            description='Max GELLO-robot joint delta (rad) before hold releases.',
        ),

        _arm_include("right", "192.168.1.201", "wsg32_right"),
        _arm_include("left",  "192.168.1.202", "wsg32_left"),
        gello_bridges,
    ])
