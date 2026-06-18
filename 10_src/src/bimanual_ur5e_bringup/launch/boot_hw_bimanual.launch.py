import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_ARM_DRIVER = os.path.join(
    os.path.dirname(__file__), "_ur5e_arm.launch.py"
)
KINEMATICS_RIGHT = "/home/phillippgery/TwinNexus-Admittance-Platform/ur5e_factory_calibration.yaml"
KINEMATICS_LEFT  = "/home/phillippgery/TwinNexus-Admittance-Platform/10_src/robot_calibration.yaml"


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")

    right_arm = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(_ARM_DRIVER),
        launch_arguments={
            "namespace":              "right_arm",
            "robot_ip":               "192.168.1.21",
            "tf_prefix":              "right_arm_",
            "kinematics_params_file": KINEMATICS_RIGHT,
            "launch_rviz":            "false",
            "reverse_port":           "50021",  # Changed
            "script_sender_port":     "50022",  # Changed
            "trajectory_port":        "50023",  # Changed
            "script_command_port":    "50024",  # Changed
        }.items(),
    )

    left_arm = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(_ARM_DRIVER),
        launch_arguments={
            "namespace":              "left_arm",
            "robot_ip":               "192.168.1.22",
            "tf_prefix":              "left_arm_",
            "kinematics_params_file": KINEMATICS_LEFT,
            "launch_rviz":            "false",
            "reverse_port":           "50011",
            "script_command_port":    "50014",
            "script_sender_port":     "50012",
            "trajectory_port":        "50013",
        }.items(),
    )

    # Static TF: right arm at origin, left arm 64 cm along Y
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="right_to_left_arm_tf",
        arguments=["0", "0.64", "0", "0", "0", "0",
                   "right_arm_base", "left_arm_base"],
        output="log",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        right_arm,
        # Delay left arm 5 s — both ros2_control_nodes starting simultaneously
        # causes a DDS participant race condition (alternating SIGABRT).
        TimerAction(period=5.0, actions=[left_arm]),
        #static_tf,
        #TimerAction(period=13.0, actions=[rviz]),
    ])
