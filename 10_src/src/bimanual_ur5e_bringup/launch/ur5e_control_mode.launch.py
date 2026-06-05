from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    control_mode = LaunchConfiguration("control_mode").perform(context)
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    if control_mode == "admittance" and tf_prefix:
        raise RuntimeError(
            "control_mode:=admittance currently requires tf_prefix:='' so the admittance "
            "controller can match the UR joint and tcp_fts_sensor interface names."
        )

    driver_launch_arguments = {
        "ur_type": LaunchConfiguration("ur_type"),
        "robot_ip": LaunchConfiguration("robot_ip"),
        "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
        "launch_rviz": LaunchConfiguration("launch_rviz"),
        "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        "mock_sensor_commands": LaunchConfiguration("mock_sensor_commands"),
        "headless_mode": LaunchConfiguration("headless_mode"),
        "launch_dashboard_client": LaunchConfiguration("launch_dashboard_client"),
        "controller_spawner_timeout": LaunchConfiguration("controller_spawner_timeout"),
        "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
        "tf_prefix": LaunchConfiguration("tf_prefix"),
        "use_tool_communication": LaunchConfiguration("use_tool_communication"),
        "tool_device_name": LaunchConfiguration("tool_device_name"),
        "tool_tcp_port": LaunchConfiguration("tool_tcp_port"),
    }

    if control_mode == "admittance":
        driver_launch_arguments["activate_joint_controller"] = "false"
    else:
        driver_launch_arguments["activate_joint_controller"] = activate_joint_controller

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
                )
            ),
            launch_arguments=driver_launch_arguments.items(),
        )
    ]

    if control_mode == "admittance":
        actions.append(
            TimerAction(
                period=LaunchConfiguration("admittance_spawn_delay"),
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        output="screen",
                        arguments=[
                            "admittance_controller",
                            "--controller-manager",
                            "/controller_manager",
                            "--controller-manager-timeout",
                            LaunchConfiguration("controller_spawner_timeout"),
                            "--param-file",
                            LaunchConfiguration("admittance_params_file"),
                        ],
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "control_mode",
                default_value="trajectory",
                choices=["trajectory", "admittance"],
                description="Selects which motion controller is active after bringup.",
            ),
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR robot model passed to the upstream driver launch.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                description="IP address by which the robot can be reached.",
            ),
            DeclareLaunchArgument(
                "kinematics_params_file",
                description="Calibration file passed through to the UR driver launch.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz through the upstream driver launch.",
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
                description="Trajectory-mode controller to activate when control_mode:=trajectory.",
            ),
            DeclareLaunchArgument(
                "activate_joint_controller",
                default_value="true",
                description="Whether trajectory mode should activate the selected joint controller.",
            ),
            DeclareLaunchArgument(
                "controller_spawner_timeout",
                default_value="30",
                description="Timeout used by controller spawners.",
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
                description="Parameter file used when spawning the admittance controller.",
            ),
            DeclareLaunchArgument(
                "admittance_spawn_delay",
                default_value="8.0",
                description="Delay before spawning the admittance controller after UR bringup starts.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            DeclareLaunchArgument(
                "mock_sensor_commands",
                default_value="false",
                description="Enable mock command interfaces for sensors when using mock hardware.",
            ),
            DeclareLaunchArgument(
                "headless_mode",
                default_value="false",
                description="Enable headless mode for robot control.",
            ),
            DeclareLaunchArgument(
                "launch_dashboard_client",
                default_value="true",
                description="Launch the UR dashboard client.",
            ),
            DeclareLaunchArgument(
                "tf_prefix",
                default_value="",
                description="Prefix applied to robot interface and frame names.",
            ),
            DeclareLaunchArgument(
                "use_tool_communication",
                default_value="false",
                description="Enable UR tool serial forwarding.",
            ),
            DeclareLaunchArgument(
                "tool_device_name",
                default_value="/tmp/ttyUR",
                description="Device path created for tool communication.",
            ),
            DeclareLaunchArgument(
                "tool_tcp_port",
                default_value="54321",
                description="TCP port used for tool communication.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
