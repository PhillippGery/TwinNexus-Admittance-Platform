"""
ur5e_upstream_admittance.launch.py
-----------------------------------
TwinNexus full-stack launch — Step 2 of 2.

ARCHITECTURE (current: single right arm)
-----------------------------------------
                    ┌─────────────────────────────────┐
                    │         TwinNexus Stack          │
                    │                                  │
  GELLO right ──→  │  gello_bridge_right.py           │
  (FTAO4WDM)       │    ↓ /admittance_controller/     │
                   │      joint_references             │
                   │    ↓ /right_arm/wsg32_node/       │
                   │      cmd_pos                      │
                   │                                   │
                   │  admittance_controller ←──────────┼── UR5e right (192.168.1.21)
                   │  wsg32_node (right)   ←──────────┼── WSG32 right (192.168.1.201)
                   │  realsense cameras    ←──────────┼── 3x D415/D455
                   └─────────────────────────────────┘

BIMANUAL EXPANSION (future — add left arm)
-------------------------------------------
  GELLO left ──→  gello_bridge_left.py
                    ↓ /left_admittance_controller/joint_references
                    ↓ /left_arm/wsg32_node/cmd_pos
                  admittance_controller_left ← UR5e left (192.168.1.22)
                  wsg32_node (left)          ← WSG32 left (192.168.1.202)

USAGE
-----
  # Step 1 — boot robot (run once per session):
  boot_hw    # alias: ros2 launch bimanual_ur5e_bringup boot_hw.launch.py
  # → Press PLAY on teach pendant

  # Step 2 — launch full stack:
  spawnctrl  # alias: ros2 launch bimanual_ur5e_bringup ur5e_upstream_admittance.launch.py

  # GELLO bridge starts in hold mode — robot does not move.
  # Watch terminal for:
  #   GELLO : [...] grip=X.XXX
  #   Robot : [...] grip=X.XXX
  #   Delta : [...]
  # Match GELLO to robot pose until deltas are small (<0.3 rad).
  # After startup_hold_s (default 5.0s), bridge releases and robot follows GELLO.

PARAMETERS
----------
  startup_hold_s       : seconds to hold initial position (default 5.0)
  max_initial_delta_rad: max GELLO-robot delta before hold releases (default 0.3)
  peripheral_start_delay: seconds after admittance spawns before peripherals start (default 8.0)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    trajectory_controller       = LaunchConfiguration('trajectory_controller')
    controller_manager          = LaunchConfiguration('controller_manager')
    switch_timeout              = LaunchConfiguration('switch_timeout')
    admittance_controller_name  = LaunchConfiguration('admittance_controller_name')
    controller_spawner_timeout  = LaunchConfiguration('controller_spawner_timeout')
    admittance_params_file      = LaunchConfiguration('admittance_params_file')
    post_unload_delay           = LaunchConfiguration('post_unload_delay')
    peripheral_start_delay      = LaunchConfiguration('peripheral_start_delay')
    enable_depth                = LaunchConfiguration('enable_depth')
    startup_hold_s              = LaunchConfiguration('startup_hold_s')
    max_initial_delta_rad       = LaunchConfiguration('max_initial_delta_rad')

    # ── Controller switching ──────────────────────────────────────────────────

    admittance_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        output='screen',
        arguments=[
            admittance_controller_name,
            '--controller-manager', controller_manager,
            '--switch-timeout',     switch_timeout,
        ],
    )

    unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        output='screen',
        arguments=[
            trajectory_controller,
            '--controller-manager', controller_manager,
            '--switch-timeout',     switch_timeout,
        ],
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            admittance_controller_name,
            '--controller-manager',         controller_manager,
            '--controller-manager-timeout', controller_spawner_timeout,
            '--param-file',                 admittance_params_file,
        ],
    )

    # # ── Cameras ───────────────────────────────────────────────────────────────
    # cameras_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('bimanual_ur5e_bringup'),
    #             'launch',
    #             'realsense_cameras.launch.py',
    #         ])
    #     ]),
    #     launch_arguments={'enable_depth': enable_depth}.items(),
    # )

    # ── WSG32 right arm gripper ───────────────────────────────────────────────
    gripper_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wsg32_driver'),
                'launch',
                'wsg32.launch.py',
            ])
        ]),
        launch_arguments={
            'gripper_ip':   '192.168.1.201',
            'gripper_name': 'wsg32_right',
            'namespace':    'right_arm',
        }.items(),
    )

    # ── GELLO bridge — RIGHT ARM ──────────────────────────────────────────────
    # Bridges GELLO right (U2D2: FTAO4WDM) →
    #   /admittance_controller/joint_references  (UR5e right joints)
    #   /right_arm/wsg32_node/cmd_pos            (WSG32 right gripper)
    #
    gello_bridge_right = Node(
        package='bimanual_ur5e_bringup',
        executable='gello_bridge.py',
        name='gello_bridge_right',
        parameters=[{
            # ── GELLO hardware — RIGHT arm ────────────────────────────────
            'gello_port':     '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO51SL-if00-port0',
            'joint_offsets':  [0.0, 3.14159, 3.14159, 4.71239, 6.28318, 4.71239],
            'joint_signs':    [1.0, 1.0, -1.0, 1.0, 1.0, 1.0],
            'gripper_config': [7.0, 201.157, 159.357],

            # ── ROS topics ────────────────────────────────────────────────
            'joint_states_topic': '/joint_states',
            'target_topic':       '/twinnexus_bridge_right/target_joints',

            # ── Control ───────────────────────────────────────────────────
            'publish_hz':            30.0,
            'startup_hold_s':        3.0,
            'max_initial_delta_rad': max_initial_delta_rad,
        }],
        output='screen',
        emulate_tty=True,
    )

    # ──left arm  ────────────────────────
    # gello_bridge_left = Node(
    #     package='bimanual_ur5e_bringup',
    #     executable='gello_bridge.py',
    #     name='gello_bridge_left',
    #     parameters=[{
    #         # ── GELLO hardware — LEFT arm (fill after calibration) ────────
    #         'gello_port':     '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTXXXXXX-if00-port0',
    #         'joint_offsets':  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # ← from gello_get_offset.py
    #         'joint_signs':    [1.0, 1.0, -1.0, 1.0, 1.0, 1.0],  # ← verify for left arm
    #         'gripper_config': [7.0, 0.0, 0.0],                   # ← from gello_get_offset.py
    #
    #         # ── ROS topics — LEFT arm ─────────────────────────────────────
    #         'joint_states_topic': '/left/joint_states',
    #         'admittance_topic':   '/left_admittance_controller/joint_references',
    #         'gripper_topic':      '/left_arm/wsg32_node/cmd_pos',
    #
    #         # ── Control ───────────────────────────────────────────────────
    #         'publish_hz':             500.0,
    #         'startup_hold_s':         3.0,
    #         'max_initial_delta_rad':  max_initial_delta_rad,
    #         'bridge_delta_rad':       0.001,
    #         'tracking_delta_rad':     0.002,
    #         'tracking_threshold':     0.05,
    #         'gripper_max_mm':         55.0,
    #     }],
    #     output='screen',
    #     emulate_tty=True,
    # )

    # ── Peripheral bundle ─────────────────────────────────────────────────────
    peripherals = TimerAction(
        period=peripheral_start_delay,
        actions=[
            GroupAction([
                # cameras_launch,
                gripper_right,
                gello_bridge_right,
                # ── FUTURE: uncomment when left arm hardware arrives ──────────
                # gripper_left,
                # gello_bridge_left,
            ])
        ],
    )

    # ── Launch description ────────────────────────────────────────────────────
    return LaunchDescription([

        # ── Arguments ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'trajectory_controller',
            default_value='scaled_joint_trajectory_controller',
        ),
        DeclareLaunchArgument(
            'controller_manager',
            default_value='/controller_manager',
        ),
        DeclareLaunchArgument(
            'switch_timeout',
            default_value='5.0',
        ),
        DeclareLaunchArgument(
            'admittance_controller_name',
            default_value='admittance_controller',
        ),
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='30',
        ),
        DeclareLaunchArgument(
            'admittance_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('bimanual_ur5e_bringup'),
                'config',
                'ur5e_admittance_controller.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'post_unload_delay',
            default_value='1.0',
            description='Delay between unloading trajectory and spawning admittance.',
        ),
        DeclareLaunchArgument(
            'peripheral_start_delay',
            default_value='6.0',
            description='Seconds after admittance spawns before cameras/grippers/GELLO start.',
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
            description='Enable RealSense depth streams.',
        ),

        DeclareLaunchArgument(
            'max_initial_delta_rad',
            default_value='0.5',
            description=(
                'Maximum GELLO-robot joint delta (rad) allowed before hold releases. '
                '0.5 rad ≈ 28.6°. Prevents robot from jumping if GELLO is far from robot pose.'
            ),
        ),

        # ── Execution sequence ─────────────────────────────────────────────────
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
                    TimerAction(
                        period=post_unload_delay,
                        actions=[spawner],
                    )
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawner,
                on_exit=[peripherals],
            )
        ),
    ])