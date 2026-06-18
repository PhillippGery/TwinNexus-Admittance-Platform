"""
_ur5e_arm.launch.py  —  namespace-aware single UR5e arm driver.

Included by boot_hw_right, boot_hw_left, boot_hw_bimanual.
Not meant to be launched directly (underscore prefix).

Key difference from ur_control.launch.py:
  • robot_description generated in Python (subprocess xacro) and passed as
    a direct parameter to ros2_control_node — avoids the topic race condition
    that causes SIGABRT when two arms start simultaneously.
  • Spawner uses --controller-manager /<namespace>/controller_manager so both
    arms can run in the same ROS2 graph without clashing.
  • tf_prefix pre-substituted in controllers YAML (allow_substs fails inside
    GroupAction).
"""
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    ns          = LaunchConfiguration("namespace").perform(context)
    robot_ip    = LaunchConfiguration("robot_ip").perform(context)
    tf_prefix   = LaunchConfiguration("tf_prefix").perform(context)
    kinematics  = LaunchConfiguration("kinematics_params_file").perform(context)
    headless    = LaunchConfiguration("headless_mode").perform(context)
    mock_hw     = LaunchConfiguration("use_mock_hardware").perform(context)
    activate_jc = LaunchConfiguration("activate_joint_controller").perform(context)
    initial_jc  = LaunchConfiguration("initial_joint_controller").perform(context)
    cm_timeout  = LaunchConfiguration("controller_spawner_timeout").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    rev_port    = LaunchConfiguration("reverse_port").perform(context)
    cmd_port    = LaunchConfiguration("script_command_port").perform(context)
    send_port   = LaunchConfiguration("script_sender_port").perform(context)
    traj_port   = LaunchConfiguration("trajectory_port").perform(context)

    cm_path = f"/{ns}/controller_manager" if ns else "/controller_manager"

    # ── Resolve package paths ────────────────────────────────────────────────
    ur_drv  = get_package_share_directory("ur_robot_driver")
    ur_desc = get_package_share_directory("ur_description")
    ur_lib  = get_package_share_directory("ur_client_library")

    urdf_xacro          = f"{ur_drv}/urdf/ur.urdf.xacro"
    joint_limits        = f"{ur_desc}/config/ur5e/joint_limits.yaml"
    physical_params     = f"{ur_desc}/config/ur5e/physical_parameters.yaml"
    visual_params       = f"{ur_desc}/config/ur5e/visual_parameters.yaml"
    script_file         = f"{ur_lib}/resources/external_control.urscript"
    input_recipe        = f"{ur_drv}/resources/rtde_input_recipe.txt"
    output_recipe       = f"{ur_drv}/resources/rtde_output_recipe.txt"
    update_rate_cfg     = f"{ur_drv}/config/ur5e_update_rate.yaml"
    ur_rsp_py           = f"{ur_drv}/launch/ur_rsp.launch.py"
    ur_dash_py          = f"{ur_drv}/launch/ur_dashboard_client.launch.py"

    # ── Generate robot_description via xacro (same as ur_rsp.launch.py) ─────
    # Pass it directly so ros2_control_node doesn't need to wait for the topic.
    xacro_out = subprocess.run(
        [
            "xacro", urdf_xacro,
            f"ur_type:=ur5e",
            f"robot_ip:={robot_ip}",
            f"tf_prefix:={tf_prefix}",
            f"name:=ur5e",
            f"joint_limit_params:={joint_limits}",
            f"kinematics_params:={kinematics}",
            f"physical_params:={physical_params}",
            f"visual_params:={visual_params}",
            f"script_filename:={script_file}",
            f"input_recipe_filename:={input_recipe}",
            f"output_recipe_filename:={output_recipe}",
            f"safety_limits:=true",
            f"safety_pos_margin:=0.15",
            f"safety_k_position:=20",
            f"use_mock_hardware:={mock_hw}",
            f"mock_sensor_commands:=false",
            f"headless_mode:={headless}",
            f"use_tool_communication:=false",
            f"tool_parity:=0",
            f"tool_baud_rate:=115200",
            f"tool_stop_bits:=1",
            f"tool_rx_idle_chars:=1.5",
            f"tool_tx_idle_chars:=3.5",
            f"tool_device_name:=/tmp/ttyUR",
            f"tool_tcp_port:=54321",
            f"tool_voltage:=0",
            f"reverse_ip:=0.0.0.0",
            f"script_command_port:={cmd_port}",
            f"reverse_port:={rev_port}",
            f"script_sender_port:={send_port}",
            f"trajectory_port:={traj_port}",
        ],
        capture_output=True, text=True, check=True,
    )
    robot_description_param = {"robot_description": xacro_out.stdout}

    # ── Pre-substitute controllers YAML ──────────────────────────────────────
    # In Jazzy, relative YAML keys ("controller_manager:", "joint_trajectory_controller:",
    # etc.) do NOT match nodes running with --ros-args -r __ns:=/<ns>. Every top-level
    # key must be fully qualified ("/<ns>/<name>:") so parameter matching works.
    # update_rate also lives here because ur5e_update_rate.yaml's relative key is ignored.
    import re
    with open(f"{ur_drv}/config/ur_controllers.yaml") as f:
        ctrl_content = f.read().replace("$(var tf_prefix)", tf_prefix)
    if ns:
        ctrl_content = re.sub(
            r'^([a-z][a-z0-9_]+):',
            rf'/{ns}/\1:',
            ctrl_content,
            flags=re.MULTILINE,
        )
        ctrl_content = f"/{ns}/controller_manager:\n  ros__parameters:\n    update_rate: 500\n\n" + ctrl_content
    _tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    _tmp.write(ctrl_content)
    _tmp.flush()
    controllers_cfg = _tmp.name

    # ── Controller lists ─────────────────────────────────────────────────────
    active = [
        "joint_state_broadcaster", "io_and_status_controller",
        "speed_scaling_state_broadcaster", "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster", "ur_configuration_controller",
    ]
    inactive = [
        "scaled_joint_trajectory_controller", "joint_trajectory_controller",
        "forward_velocity_controller", "forward_position_controller",
        "forward_effort_controller", "force_mode_controller",
        "passthrough_trajectory_controller", "freedrive_mode_controller",
        "tool_contact_controller",
    ]
    if activate_jc == "true":
        active.append(initial_jc)
        inactive.remove(initial_jc)
    if mock_hw == "true" and "tcp_pose_broadcaster" in active:
        active.remove("tcp_pose_broadcaster")

    real_hw = UnlessCondition("true" if mock_hw == "true" else "false")

    rsp_args = dict(
        ur_type="ur5e", robot_ip=robot_ip, tf_prefix=tf_prefix,
        kinematics_params_file=kinematics,
        use_mock_hardware=mock_hw, headless_mode=headless,
        mock_sensor_commands="false", use_tool_communication="false",
        tool_parity="0", tool_baud_rate="115200", tool_stop_bits="1",
        tool_rx_idle_chars="1.5", tool_tx_idle_chars="3.5",
        tool_device_name="/tmp/ttyUR", tool_tcp_port="54321", tool_voltage="0",
        reverse_ip="0.0.0.0", reverse_port=rev_port,
        script_command_port=cmd_port, script_sender_port=send_port,
        trajectory_port=traj_port,
        safety_limits="true", safety_pos_margin="0.15", safety_k_position="20",
    )

    arm_group = GroupAction([
        PushRosNamespace(ns),

        # robot_description passed directly → no topic race condition
        Node(
            package="controller_manager", executable="ros2_control_node",
            parameters=[update_rate_cfg, robot_description_param, controllers_cfg],
            output="screen",
        ),

        # RSP still needed so other nodes (RViz, MoveIt) can get robot_description
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(ur_rsp_py),
            launch_arguments=rsp_args.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(ur_dash_py),
            launch_arguments={"robot_ip": robot_ip}.items(),
            condition=real_hw,
        ),

        Node(
            package="ur_robot_driver", executable="robot_state_helper",
            name="ur_robot_state_helper", output="screen", condition=real_hw,
            parameters=[{"headless_mode": headless == "true"}, {"robot_ip": robot_ip}],
        ),

        Node(
            package="ur_robot_driver", executable="urscript_interface",
            parameters=[{"robot_ip": robot_ip}], output="screen", condition=real_hw,
        ),

        Node(
            package="ur_robot_driver", executable="controller_stopper_node",
            name="controller_stopper", output="screen", emulate_tty=True, condition=real_hw,
            parameters=[
                {"headless_mode": headless == "true"},
                {"joint_controller_active": activate_jc == "true"},
                {"consistent_controllers": [
                    "io_and_status_controller", "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster", "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster", "ur_configuration_controller",
                ]},
            ],
        ),

        Node(
            package="ur_robot_driver", executable="trajectory_until_node",
            name="trajectory_until_node", output="screen",
            parameters=[{"motion_controller": initial_jc}],
        ),
    ])

    # Spawners outside GroupAction — explicit cm_path bypasses PushRosNamespace limit
    spawner_active = Node(
        package="controller_manager", executable="spawner", namespace=ns,
        arguments=["--controller-manager", cm_path,
                   "--controller-manager-timeout", cm_timeout] + active,
        output="screen",
    )
    spawner_inactive = Node(
        package="controller_manager", executable="spawner", namespace=ns,
        arguments=["--controller-manager", cm_path,
                   "--controller-manager-timeout", cm_timeout,
                   "--inactive"] + inactive,
        output="screen",
    )

    nodes = [arm_group, spawner_active, spawner_inactive]

    if launch_rviz == "true":
        from launch.substitutions import PathJoinSubstitution
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", output="log",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            )],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace",                  default_value=""),
        DeclareLaunchArgument("robot_ip"),
        DeclareLaunchArgument("tf_prefix",                  default_value=""),
        DeclareLaunchArgument("kinematics_params_file",
            default_value="/home/phillippgery/TwinNexus-Admittance-Platform/ur5e_factory_calibration.yaml"),
        DeclareLaunchArgument("launch_rviz",                default_value="true"),
        DeclareLaunchArgument("headless_mode",              default_value="false"),
        DeclareLaunchArgument("use_mock_hardware",          default_value="false"),
        DeclareLaunchArgument("activate_joint_controller",  default_value="true"),
        DeclareLaunchArgument("initial_joint_controller",
            default_value="scaled_joint_trajectory_controller"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="60"),
        DeclareLaunchArgument("reverse_port",               default_value="50001"),
        DeclareLaunchArgument("script_command_port",        default_value="50004"),
        DeclareLaunchArgument("script_sender_port",         default_value="50002"),
        DeclareLaunchArgument("trajectory_port",            default_value="50003"),
        OpaqueFunction(function=launch_setup),
    ])
