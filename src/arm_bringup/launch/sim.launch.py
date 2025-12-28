import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_share = get_package_share_directory('arm_bringup')
    description_share = get_package_share_directory('arm_description')

    xacro_path = os.path.join(description_share, 'xacro', 'ur5.xacro')
    # Command concatenates tokens without spaces, so we include the space explicitly
    robot_description = Command(['xacro ', xacro_path])

    # Add both description and bringup shares to the resource path so meshes/worlds are found
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=(
            os.path.dirname(description_share)
            + ":" + os.path.dirname(bringup_share)
            + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        ),
    )

    world_path = os.path.join(bringup_share, "worlds", "ur5_world.world")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        # pass world file via gz_args
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "default",
            "-name", "ur5",
            "-string", robot_description,
            "-x", "0.0", "-y", "0.0", "-z", "0.0"
        ],
        output="screen",
    )

    # Bridge for /clock from Gazebo to ROS2
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        # parameter_bridge expects the closing bracket to be omitted
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Controller spawners - delay to avoid race with gz_ros2_control / entity spawn.
    # On slower machines the controller_manager services can appear before the hardware
    # interfaces are fully ready, causing sporadic "Failed to configure controller".
    delayed_jsb_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "60",
                    "--service-call-timeout", "60",
                    "--switch-timeout", "60",
                ],
                output="screen",
            )
        ]
    )

    delayed_traj_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "60",
                    "--service-call-timeout", "60",
                    "--switch-timeout", "60",
                ],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_launch,
        rsp,
        clock_bridge,
        spawn,
        delayed_jsb_spawner,
        delayed_traj_spawner,
    ])
