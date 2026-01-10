import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    gz_headless = LaunchConfiguration('gz_headless')
    position_proportional_gain = LaunchConfiguration('position_proportional_gain')

    bringup_share = get_package_share_directory('arm_bringup')
    description_share = get_package_share_directory('arm_description')

    # Some environments have Snap-provided core libraries in LD_LIBRARY_PATH (e.g. /snap/core*/...).
    # Gazebo Harmonic from APT / ROS vendor can crash if it picks up those incompatible libs.
    # Filter them out for this launch to keep it native Ubuntu 24.04 / APT-based.
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    ld_library_path_filtered = ':'.join(
        [p for p in ld_library_path.split(':') if p and '/snap/' not in p]
    )
    set_ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=ld_library_path_filtered,
    )

    gz_ros2_control_lib = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')

    set_gz_system_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gz_ros2_control_lib + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    )

    xacro_path = os.path.join(description_share, 'xacro', 'ur5.xacro')
    # Command concatenates tokens without spaces, so we include the space explicitly
    robot_description = Command(
        [
            'xacro ',
            xacro_path,
            ' position_proportional_gain:=',
            position_proportional_gain,
        ]
    )

    # Add both description and bringup shares to the resource path so meshes/worlds are found
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=(
            os.path.dirname(description_share)
            + ':'
            + os.path.dirname(bringup_share)
            + ':'
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
    )

    world_path = os.path.join(bringup_share, 'worlds', 'ur5_world.world')

    # If headless, run server-only (no GUI). This helps determine whether perceived motion lags
    # are just Gazebo rendering stutters.
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # pass world file via gz_args
        launch_arguments={
            'gz_args': [
                '-r ',
                gz_headless,
                ' -v ',
                gz_verbosity,
                ' ',
                world_path,
            ]
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world',
            'default',
            '-name',
            'ur5',
            '-string',
            robot_description,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
        ],
        output='screen',
    )

    # Bridge for /clock from Gazebo to ROS2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # parameter_bridge expects the closing bracket to be omitted
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Ensure controllers are ACTIVE. This is more robust than using the spawner
    # directly because it is idempotent (doesn't fail if controllers are already
    # loaded/active) and retries configuration/activation until success or timeout.
    ensure_controllers = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='arm_bringup',
                executable='ensure_controllers_active',
                output='screen',
                parameters=[
                    {'controller_manager': '/controller_manager'},
                    {'controllers': ['joint_state_broadcaster', 'joint_trajectory_controller']},
                    {'timeout_s': 60.0},
                    {'poll_period_s': 0.5},
                    {'switch_timeout_s': 60.0},
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gz_verbosity',
                default_value='1',
                description='Gazebo (gz sim) verbosity level; higher values can cause stutters',
            ),
            DeclareLaunchArgument(
                'gz_headless',
                default_value='',
                description=(
                    'If set to \'-s\', runs gz sim server-only (no GUI). '
                    'Leave empty for normal GUI.'
                ),
            ),
            DeclareLaunchArgument(
                'position_proportional_gain',
                default_value='0.3',
                description=(
                    'gz_ros2_control internal position loop proportional gain '
                    '(higher = stiffer, too high can oscillate).'
                ),
            ),
            set_ld_library_path,
            set_gz_resource_path,
            set_gz_system_plugin_path,
            gz_launch,
            rsp,
            clock_bridge,
            spawn,
            ensure_controllers,
        ]
    )
