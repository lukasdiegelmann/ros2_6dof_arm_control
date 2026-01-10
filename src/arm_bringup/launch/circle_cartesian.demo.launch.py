from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Sim args (forwarded into sim.launch.py)
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    gz_headless = LaunchConfiguration("gz_headless")
    position_proportional_gain = LaunchConfiguration("position_proportional_gain")

    # Shared args
    use_sim_time = LaunchConfiguration("use_sim_time")
    trajectory_topic = LaunchConfiguration("trajectory_topic")

    # Delay starting the circle node so controllers + /joint_states are up.
    start_delay_s = LaunchConfiguration("start_delay_s")

    # Circle args (keep lists fixed; scalars are configurable)
    radius = LaunchConfiguration("radius")
    plane = LaunchConfiguration("plane")
    num_points = LaunchConfiguration("num_points")
    loops = LaunchConfiguration("loops")
    point_duration = LaunchConfiguration("point_duration")
    pause_s = LaunchConfiguration("pause_s")

    vel_scale = LaunchConfiguration("vel_scale")
    max_acc = LaunchConfiguration("max_acc")

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("arm_description"),
            "xacro",
            "ur5.xacro",
        ]
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " position_proportional_gain:=",
                position_proportional_gain,
            ]
        )
    }

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_bringup"),
                    "launch",
                    "sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_verbosity": gz_verbosity,
            "gz_headless": gz_headless,
            "position_proportional_gain": position_proportional_gain,
        }.items(),
    )

    go_to_pose_node = Node(
        package="arm_apps",
        executable="go_to_pose_node",
        name="go_to_pose_node",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time,
                "base_link": "base_link",
                "tip_link": "ee_link",
                "trajectory_topic": trajectory_topic,
            },
        ],
    )

    draw_circle_node = Node(
        package="arm_apps",
        executable="draw_circle_cartesian_node",
        name="draw_circle_cartesian_node",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time,
                "frame_id": "base_link",
                "center": [0.35, 0.0, 0.30],
                "radius": radius,
                "plane": plane,
                "num_points": num_points,
                "loops": loops,
                "start_angle": 0.0,
                "direction": 1,
                "constant_rpy": [0.0, 1.57, 0.0],
                "pause_s": pause_s,
                "vel_scale": vel_scale,
                "max_acc": max_acc,
                "num_waypoints": 50,
                "desired_duration": 0.0,
                "point_duration": point_duration,
                "continue_on_fail": False,
                "log_csv": False,
            },
        ],
    )

    delayed_circle = TimerAction(
        period=start_delay_s,
        actions=[draw_circle_node],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "trajectory_topic",
                default_value="/joint_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument("gz_verbosity", default_value="1"),
            DeclareLaunchArgument("gz_headless", default_value=""),
            DeclareLaunchArgument("position_proportional_gain", default_value="0.3"),
            DeclareLaunchArgument(
                "start_delay_s",
                default_value="6.0",
                description="Seconds to wait before starting draw_circle_cartesian_node (lets controllers + /joint_states come up)",
            ),
            # Bigger default circle so motion is clearly visible
            DeclareLaunchArgument("radius", default_value="0.18"),
            DeclareLaunchArgument("plane", default_value="xy"),
            DeclareLaunchArgument("num_points", default_value="80"),
            DeclareLaunchArgument("loops", default_value="2"),
            DeclareLaunchArgument("point_duration", default_value="0.30"),
            DeclareLaunchArgument("pause_s", default_value="0.0"),
            DeclareLaunchArgument("vel_scale", default_value="0.8"),
            DeclareLaunchArgument("max_acc", default_value="3.0"),
            sim_launch,
            go_to_pose_node,
            delayed_circle,
        ]
    )
