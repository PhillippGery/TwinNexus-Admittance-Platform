from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    gripper_delay = LaunchConfiguration("gripper_delay")

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

    # ── WSG32 right arm gripper ───────────────────────────────────────────────
    gripper_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("wsg32_driver"),
                "launch",
                "wsg32.launch.py",
            ])
        ]),
        launch_arguments={
            "gripper_ip":   "192.168.1.201",
            "gripper_name": "wsg32_right",
            "namespace":    "right_arm",
        }.items(),
    )

    # ── TwinNexus bridge — interpolation layer between any target source ──────
    # and the admittance controller.  Runs as a separate process (own GIL) so
    # camera/inference threads cannot starve its 500Hz timer.
    bridge_node = Node(
        package="bimanual_ur5e_bringup",
        executable="twinnexus_bridge.py",
        name="twinnexus_bridge_right",
        output="screen",
        parameters=[{
            "publish_hz":          500.0,
            "joint_states_topic":  "/joint_states",
            "admittance_topic":    "/admittance_controller/joint_references",
            "gripper_topic":       "/right_arm/wsg32_node/cmd_pos",
            "tracking_delta_rad":  0.002,
            "go_home_delta_rad":   0.001,
        }],
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
            DeclareLaunchArgument(
                "gripper_delay",
                default_value="5.0",
                description="Seconds after admittance spawns before starting the WSG32 gripper.",
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
                    on_exit=[
                        # Bridge starts as soon as the trajectory controller is gone.
                        # It begins holding the current robot position immediately so the
                        # admittance controller inherits a correct reference the moment it
                        # activates — no stale-reference velocity spike on first command.
                        bridge_node,
                        TimerAction(period=post_unload_delay, actions=[spawner]),
                    ],
                )
            ),
            # Start gripper after admittance controller is up
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawner,
                    on_exit=[
                        TimerAction(period=gripper_delay, actions=[gripper_right]),
                    ],
                )
            ),
        ]
    )
