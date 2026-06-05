from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory_controller = LaunchConfiguration("trajectory_controller")
    controller_manager = LaunchConfiguration("controller_manager")
    switch_timeout = LaunchConfiguration("switch_timeout")
    admittance_controller_name = LaunchConfiguration("admittance_controller_name")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    admittance_params_file = LaunchConfiguration("admittance_params_file")
    post_unload_delay = LaunchConfiguration("post_unload_delay")

    admittance_unspawner = Node(
        package="controller_manager",
        executable="unspawner",
        output="screen",
        arguments=[
            admittance_controller_name,
            "--controller-manager",
            controller_manager,
            "--switch-timeout",
            switch_timeout,
        ],
    )

    unspawner = Node(
        package="controller_manager",
        executable="unspawner",
        output="screen",
        arguments=[
            trajectory_controller,
            "--controller-manager",
            controller_manager,
            "--switch-timeout",
            switch_timeout,
        ],
    )

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            admittance_controller_name,
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--param-file",
            admittance_params_file,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory_controller",
                default_value="scaled_joint_trajectory_controller",
                description="Controller to unload before activating admittance mode.",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Controller manager node name.",
            ),
            DeclareLaunchArgument(
                "switch_timeout",
                default_value="5.0",
                description="Timeout passed to the controller_manager unspawner.",
            ),
            DeclareLaunchArgument(
                "admittance_controller_name",
                default_value="admittance_controller",
                description="Name of the upstream admittance controller instance.",
            ),
            DeclareLaunchArgument(
                "controller_spawner_timeout",
                default_value="30",
                description="Timeout used by the controller spawner while waiting for controller_manager.",
            ),
            DeclareLaunchArgument(
                "admittance_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("bimanual_ur5e_bringup"),
                        "config",
                        "ur5e_admittance_controller.yaml",
                    ]
                ),
                description="Parameter file used when spawning the upstream admittance controller.",
            ),
            DeclareLaunchArgument(
                "post_unload_delay",
                default_value="1.0",
                description="Delay between unloading the trajectory controller and spawning admittance.",
            ),
            admittance_unspawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=admittance_unspawner,
                    on_exit=[unspawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=unspawner,
                    on_exit=[TimerAction(period=post_unload_delay, actions=[spawner])],
                )
            ),
        ]
    )
