"""
ur5e_upstream_admittance.launch.py
-----------------------------------
TwinNexus full-stack launch — Step 2 of 2.

Prerequisites (must be done first, in order):
    1. ros2 launch bimanual_ur5e_bringup boot_hw.launch.py   ← connects to UR5e
    2. Press PLAY on the UR5e teach pendant (remote control)
    3. THIS FILE                                              ← you are here

What this file does, in sequence:
    1. Unloads any running admittance_controller (clean slate)
    2. Unloads scaled_joint_trajectory_controller
    3. Waits post_unload_delay seconds
    4. Spawns upstream admittance_controller
    5. After admittance is up: launches cameras, gripper drivers, GELLO bridge

Aliases (add to ~/.bashrc):
    alias boot_hw='ros2 launch bimanual_ur5e_bringup boot_hw.launch.py'
    alias spawnctrl='ros2 launch bimanual_ur5e_bringup ur5e_upstream_admittance.launch.py'
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
    trajectory_controller        = LaunchConfiguration('trajectory_controller')
    controller_manager           = LaunchConfiguration('controller_manager')
    switch_timeout               = LaunchConfiguration('switch_timeout')
    admittance_controller_name   = LaunchConfiguration('admittance_controller_name')
    controller_spawner_timeout   = LaunchConfiguration('controller_spawner_timeout')
    admittance_params_file       = LaunchConfiguration('admittance_params_file')
    post_unload_delay            = LaunchConfiguration('post_unload_delay')
    peripheral_start_delay       = LaunchConfiguration('peripheral_start_delay')
    enable_depth                 = LaunchConfiguration('enable_depth')
    mock_gello                   = LaunchConfiguration('mock_gello')

    # ── Step 1 & 2: Controller switching (unchanged from original) ────────────

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

    # ── Step 3: Peripheral stack (cameras + grippers + GELLO bridge) ──────────
    # Launched after admittance controller is up, with an additional delay
    # to let the controller fully initialise before accepting joint references.

    # ── Cameras ───────────────────────────────────────────────────────────────
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bimanual_ur5e_bringup'),
                'launch',
                'realsense_cameras.launch.py',
            ])
        ]),
        launch_arguments={
            'enable_depth': enable_depth,
        }.items(),
    )

    # ── WSG32 grippers ────────────────────────────────────────────────────────
    # Right arm: 192.168.1.201  Left arm: 192.168.1.202
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

    gripper_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wsg32_driver'),
                'launch',
                'wsg32.launch.py',
            ])
        ]),
        launch_arguments={
            'gripper_ip':   '192.168.1.202',
            'gripper_name': 'wsg32_left',
            'namespace':    'left_arm',
        }.items(),
    )

    # ── GELLO bridge ──────────────────────────────────────────────────────────
    gello_bridge = Node(
        package='bimanual_ur5e_bringup',
        executable='gello_bridge.py',
        name='gello_bridge',
        parameters=[{
            'mock':               mock_gello,
            'publish_hz':         50.0,
            'mock_amp_rad':       0.02,       # safe default — change manually for testing
            'mock_start_delay_s': 3.0,
            'max_delta_rad':      0.01,
        }],
        output='screen',
        emulate_tty=True,
    )

    # ── Bundle all peripherals behind a timer delay ───────────────────────────
    # peripheral_start_delay gives the admittance controller time to fully
    # initialise before cameras and the GELLO bridge start sending data.
    peripherals = TimerAction(
        period=peripheral_start_delay,
        actions=[
            GroupAction([
                cameras_launch,
                gripper_right,
                gripper_left,
                gello_bridge,
            ])
        ],
    )

    # ── Wire everything together ──────────────────────────────────────────────
    return LaunchDescription([

        # ── Declare arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            'trajectory_controller',
            default_value='scaled_joint_trajectory_controller',
            description='Controller to unload before activating admittance mode.',
        ),
        DeclareLaunchArgument(
            'controller_manager',
            default_value='/controller_manager',
            description='Controller manager node name.',
        ),
        DeclareLaunchArgument(
            'switch_timeout',
            default_value='5.0',
            description='Timeout passed to the controller_manager unspawner.',
        ),
        DeclareLaunchArgument(
            'admittance_controller_name',
            default_value='admittance_controller',
            description='Name of the upstream admittance controller instance.',
        ),
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='30',
            description='Timeout used by the controller spawner.',
        ),
        DeclareLaunchArgument(
            'admittance_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('bimanual_ur5e_bringup'),
                'config',
                'ur5e_admittance_controller.yaml',
            ]),
            description='Parameter file for the upstream admittance controller.',
        ),
        DeclareLaunchArgument(
            'post_unload_delay',
            default_value='1.0',
            description='Delay between unloading trajectory controller and spawning admittance.',
        ),
        DeclareLaunchArgument(
            'peripheral_start_delay',
            default_value='8.0',
            description=(
                'Seconds to wait after admittance controller spawns before '
                'starting cameras, grippers, and GELLO bridge. '
                'Gives the controller time to fully initialise.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
            description='Enable RealSense depth streams (increases USB3 bandwidth).',
        ),
        DeclareLaunchArgument(
            'mock_gello',
            default_value='false',
            description=(
                'Run GELLO bridge in mock mode (sine wave). '
                'Set true for pipeline testing without GELLO hardware.'
            ),
        ),

        # ── Execution sequence ────────────────────────────────────────────────
        # 1. Unload any running admittance_controller
        admittance_unspawner,

        # 2. On exit: unload trajectory controller
        RegisterEventHandler(
            OnProcessExit(
                target_action=admittance_unspawner,
                on_exit=[unspawner],
            )
        ),

        # 3. On exit: wait post_unload_delay, then spawn admittance controller
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

        # 4. On exit: wait peripheral_start_delay, then launch everything else
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawner,
                on_exit=[peripherals],
            )
        ),
    ])