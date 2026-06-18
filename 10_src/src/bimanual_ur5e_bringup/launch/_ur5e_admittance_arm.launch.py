"""
_ur5e_admittance_arm.launch.py — per-arm admittance + TwinNexus bridge.

Included by ur5e_upstream_admittance_bimanual.launch.py (not meant to launch directly).
Expects boot_hw_bimanual (or boot_hw_left/right) to already be running.
"""



from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context: LaunchContext, *args, **kwargs):
    # 1. EVALUATE ALL CONFIGURATIONS IMMEDIATELY (Stops the Context Leak)
    traj_ctrl = LaunchConfiguration("trajectory_controller").perform(context)
    ctrl_mgr = LaunchConfiguration("controller_manager").perform(context)
    switch_t = LaunchConfiguration("switch_timeout").perform(context)
    adm_ctrl_name = LaunchConfiguration("admittance_controller_name").perform(context)
    spawn_timeout = LaunchConfiguration("controller_spawner_timeout").perform(context)
    adm_params = LaunchConfiguration("admittance_params_file").perform(context)
    post_unload = float(LaunchConfiguration("post_unload_delay").perform(context))
    grip_delay = float(LaunchConfiguration("gripper_delay").perform(context))
    grip_ip = LaunchConfiguration("gripper_ip").perform(context)
    grip_name = LaunchConfiguration("gripper_name").perform(context)
    arm_ns = LaunchConfiguration("arm_namespace").perform(context)
    bridge_name = LaunchConfiguration("bridge_name").perform(context)
    j_states_top = LaunchConfiguration("joint_states_topic").perform(context)
    adm_top = LaunchConfiguration("admittance_topic").perform(context)
    grip_top = LaunchConfiguration("gripper_topic").perform(context)
    j_prefix = LaunchConfiguration("joint_prefix").perform(context)

    # 2. CREATE NODES WITH STRICT NAMES AND NAMESPACES
    admittance_unspawner = Node(
        package="controller_manager",
        executable="unspawner",
        name=f"{arm_ns}_admittance_unspawner",
        namespace=arm_ns,
        output="screen",
        arguments=[adm_ctrl_name, "--controller-manager", ctrl_mgr, "--switch-timeout", switch_t],
    )

    unspawner = Node(
        package="controller_manager",
        executable="unspawner",
        name=f"{arm_ns}_trajectory_unspawner",
        namespace=arm_ns,
        output="screen",
        arguments=[traj_ctrl, "--controller-manager", ctrl_mgr, "--switch-timeout", switch_t],
    )

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        name=f"{arm_ns}_admittance_spawner",
        namespace=arm_ns,
        output="screen",
        arguments=[
            adm_ctrl_name,
            "--controller-manager", ctrl_mgr,
            "--controller-manager-timeout", spawn_timeout,
            "--param-file", adm_params,
        ],
    )

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("wsg32_driver"), "launch", "wsg32.launch.py"])
        ]),
        launch_arguments={
            "gripper_ip": grip_ip,
            "gripper_name": grip_name,
            "namespace": arm_ns,
        }.items(),
    )

    bridge_node = Node(
        package="bimanual_ur5e_bringup",
        executable="twinnexus_bridge.py",
        name=bridge_name,
        namespace=arm_ns,
        output="screen",
        parameters=[{
            "publish_hz": 500.0,
            "joint_states_topic": j_states_top,
            "admittance_topic": adm_top,
            "gripper_topic": grip_top,
            "joint_prefix": j_prefix,
            "tracking_delta_rad": 0.002,
            "go_home_delta_rad": 0.001,
        }],
    )

    # 3. CHAIN EVENTS USING THE ISOLATED PYTHON OBJECTS
    return [
        admittance_unspawner,
        RegisterEventHandler(
            OnProcessExit(target_action=admittance_unspawner, on_exit=[unspawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=unspawner, on_exit=[
                bridge_node,
                TimerAction(period=post_unload, actions=[spawner]),
            ])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=spawner, on_exit=[
                TimerAction(period=grip_delay, actions=[gripper]),
            ])
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("trajectory_controller", default_value="scaled_joint_trajectory_controller"),
        DeclareLaunchArgument("controller_manager"),
        DeclareLaunchArgument("switch_timeout", default_value="5.0"),
        DeclareLaunchArgument("admittance_controller_name", default_value="admittance_controller"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="30"),
        DeclareLaunchArgument("admittance_params_file"),
        DeclareLaunchArgument("post_unload_delay", default_value="1.0"),
        DeclareLaunchArgument("gripper_delay", default_value="5.0"),
        DeclareLaunchArgument("gripper_ip"),
        DeclareLaunchArgument("gripper_name"),
        DeclareLaunchArgument("arm_namespace"),
        DeclareLaunchArgument("bridge_name"),
        DeclareLaunchArgument("joint_states_topic"),
        DeclareLaunchArgument("admittance_topic"),
        DeclareLaunchArgument("gripper_topic"),
        DeclareLaunchArgument("joint_prefix"),
        OpaqueFunction(function=launch_setup) # Injects the strictly evaluated setup
    ])