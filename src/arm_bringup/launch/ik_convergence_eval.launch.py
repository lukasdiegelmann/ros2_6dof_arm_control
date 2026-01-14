from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    output_csv = LaunchConfiguration("output_csv")

    xacro_file = PathJoinSubstitution([FindPackageShare("arm_description"), "xacro", "ur5.xacro"])
    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    eval_node = Node(
        package="arm_apps",
        executable="ik_convergence_eval",
        name="ik_convergence_eval",
        output="screen",
        parameters=[
            robot_description,
            {"base_link": "base_link"},
            {"tip_link": "ee_link"},
            {"output_csv": output_csv},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_csv",
                default_value="docs/evaluation",
                description=(
                    "Where to write the IK convergence CSVs. If this is a directory, timestamped "
                    "files are created inside it. If a *.csv path is given (legacy), timestamped "
                    "files are created in its parent directory."
                ),
            ),
            eval_node,
        ]
    )
