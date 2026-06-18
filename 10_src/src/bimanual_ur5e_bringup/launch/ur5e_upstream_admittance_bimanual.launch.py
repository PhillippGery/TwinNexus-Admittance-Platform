"""
ur5e_upstream_admittance_bimanual.launch.py
-------------------------------------------
TwinNexus inference / policy control for both UR5e arms.

Usage (after boot_hw_bimanual + Play on both teach pendants):
    ros2 launch bimanual_ur5e_bringup ur5e_upstream_admittance_bimanual.launch.py
    # or: spawnctrl_bimanual
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

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
    return LaunchDescription([
        _arm_include("right", "192.168.1.201", "wsg32_right"),
        _arm_include("left", "192.168.1.202", "wsg32_left"),
    ])
