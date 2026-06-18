from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"
                ])
            ),
            launch_arguments={
                "ur_type":                "ur5e",
                "robot_ip":               "192.168.1.21",
                "kinematics_params_file": "/home/phillippgery/TwinNexus-Admittance-Platform/ur5e_factory_calibration.yaml",
                "launch_rviz":            "true",
            }.items(),

            

            
        ),
    ])
