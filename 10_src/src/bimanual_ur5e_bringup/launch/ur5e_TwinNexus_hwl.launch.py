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

    # ── Cameras ───────────────────────────────────────────────────────────────
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bimanual_ur5e_bringup'),
                'launch',
                'realsense_cameras.launch.py',
            ])
        ]),
        launch_arguments={'enable_depth': enable_depth}.items(),
    )

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
    # FUTURE LEFT ARM: add gello_bridge_left node here with:
    #   - different GELLO_PORT (left U2D2 serial)
    #   - different joint_references topic (/left_admittance_controller/...)
    #   - different gripper topic (/left_arm/wsg32_node/cmd_pos)
    gello_bridge_right = Node(
        package='bimanual_ur5e_bringup',
        executable='gello_bridge.py',
        name='gello_bridge_right',
        parameters=[{
            'startup_hold_s':         startup_hold_s,
            'max_initial_delta_rad':  max_initial_delta_rad,
            'publish_hz':             500.0,
            'bridge_delta_rad':       0.001,
            'tracking_delta_rad':     0.002,
            'tracking_threshold':     0.05,
        }],
        output='screen',
        emulate_tty=True,
    )

    # ── Peripheral bundle ─────────────────────────────────────────────────────
    peripherals = TimerAction(
        period=peripheral_start_delay,
        actions=[
            GroupAction([
                cameras_launch,
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
            'startup_hold_s',
            default_value='3.0',
            description=(
                'Seconds GELLO bridge holds initial position before releasing. '
                'Gives operator time to match GELLO pose to robot. '
                'Bridge will NOT release until GELLO deltas are within max_initial_delta_rad.'
            ),
        ),
        DeclareLaunchArgument(
            'max_initial_delta_rad',
            default_value='0.3',
            description=(
                'Maximum GELLO-robot joint delta (rad) allowed before hold releases. '
                '0.3 rad ≈ 17°. Prevents robot from jumping if GELLO is far from robot pose.'
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