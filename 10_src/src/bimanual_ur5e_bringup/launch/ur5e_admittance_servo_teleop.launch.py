from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    twist_frame_id = LaunchConfiguration("twist_frame_id")
    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
            "launch_servo": "true",
        }.items(),
    )

    servo_bridge = Node(
        package="bimanual_ur5e_bringup",
        executable="moveit_servo_to_admittance.py",
        output="screen",
    )

    set_twist_mode = Node(
        package="bimanual_ur5e_bringup",
        executable="set_servo_command_type.py",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR robot model for the MoveIt Servo stack.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch MoveIt RViz alongside Servo teleop.",
            ),
            DeclareLaunchArgument(
                "twist_frame_id",
                default_value="base_link",
                description="Frame used for keyboard Cartesian twist commands.",
            ),
            moveit_servo_launch,
            servo_bridge,
            TimerAction(period=3.0, actions=[set_twist_mode]),
            LogInfo(
                msg=[
                    "Start keyboard teleop in an interactive terminal with: ",
                    "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
                    "--ros-args -p stamped:=true -p frame_id:=",
                    twist_frame_id,
                    " -p repeat_rate:=20.0 -p key_timeout:=0.25 "
                    "-p speed:=0.08 -p turn:=0.25 "
                    "-r /cmd_vel:=/servo_node/delta_twist_cmds",
                ]
            ),
        ]
    )
