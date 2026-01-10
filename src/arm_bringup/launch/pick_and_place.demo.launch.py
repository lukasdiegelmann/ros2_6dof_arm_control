from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    trajectory_topic = LaunchConfiguration("trajectory_topic")
    run_demo = LaunchConfiguration("run_demo")
    vel_scale = LaunchConfiguration("vel_scale")
    max_acc = LaunchConfiguration("max_acc")
    num_waypoints = LaunchConfiguration("num_waypoints")
    desired_duration = LaunchConfiguration("desired_duration")
    pause_s = LaunchConfiguration("pause_s")
    continue_on_failure = LaunchConfiguration("continue_on_failure")
    return_home_on_failure = LaunchConfiguration("return_home_on_failure")
    home_index = LaunchConfiguration("home_index")
    log_targets = LaunchConfiguration("log_targets")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("arm_description"),
        "xacro",
        "ur5.xacro",
    ])

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arm_bringup"),
                "launch",
                "sim.launch.py",
            ])
        )
    )

    go_to_pose_node = Node(
        package="arm_apps",
        executable="go_to_pose_node",
        name="go_to_pose_node",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            {"base_link": "base_link"},
            {"tip_link": "ee_link"},
            {"trajectory_topic": trajectory_topic},
        ],
    )

    demo_sequence_node = Node(
        package="arm_apps",
        executable="demo_sequence_node",
        name="demo_sequence_node",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time,
                "targets_yaml": "config/demo_targets.yaml",
                "trajectory_topic": trajectory_topic,
                "vel_scale": vel_scale,
                "max_acc": max_acc,
                "num_waypoints": num_waypoints,
                "desired_duration": desired_duration,
                "pause_s": pause_s,
                "continue_on_failure": continue_on_failure,
                "return_home_on_failure": return_home_on_failure,
                "home_index": home_index,
                "log_targets": log_targets,
            }]
        ,
        condition=IfCondition(run_demo),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "trajectory_topic",
            default_value="/joint_trajectory_controller/joint_trajectory",
        ),
        DeclareLaunchArgument("run_demo", default_value="true"),
        DeclareLaunchArgument("vel_scale", default_value="0.8"),
        DeclareLaunchArgument("max_acc", default_value="3.0"),
        DeclareLaunchArgument("num_waypoints", default_value="30"),
        DeclareLaunchArgument("desired_duration", default_value="0.0"),
        DeclareLaunchArgument("pause_s", default_value="0.0"),
        DeclareLaunchArgument("continue_on_failure", default_value="false"),
        DeclareLaunchArgument("return_home_on_failure", default_value="true"),
        DeclareLaunchArgument("home_index", default_value="-1"),
        DeclareLaunchArgument("log_targets", default_value="true"),
        sim_launch,
        go_to_pose_node,
        demo_sequence_node,
    ])
