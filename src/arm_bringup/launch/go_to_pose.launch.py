from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("arm_description"),
        "xacro",
        "ur5.xacro"
    ])

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher_go_to_pose",
            output="screen",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="arm_apps",
            executable="go_to_pose_node",
            name="go_to_pose_node",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": True},
                {"base_link": "base_link"},
                {"tip_link": "ee_link"},
            ],
        ),
    ])
