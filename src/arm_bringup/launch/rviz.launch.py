#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Args
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rqt_plot = LaunchConfiguration("use_rqt_plot")
    plot_joint_count = LaunchConfiguration("plot_joint_count")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")

    pkg_share = get_package_share_directory("arm_bringup")
    installed_default_rviz = os.path.join(pkg_share, "config", "rviz", "default.rviz")
    source_default_rviz = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "rviz", "default.rviz")
    )
    default_rviz_config = (
        installed_default_rviz if os.path.exists(installed_default_rviz) else source_default_rviz
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock if available (Gazebo/Sim time).",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description=(
            "Full path to an RViz2 config file. Default is arm_bringup/config/rviz/default.rviz. "
            "If missing, RViz will start without -d."
        ),
    )

    declare_use_rqt_plot = DeclareLaunchArgument(
        "use_rqt_plot",
        default_value="false",
        description=(
            "If true, starts rqt_plot for /joint_states/position[i]. "
            "Requires a GUI session and rqt_plot installed."
        ),
    )

    declare_start_robot_state_publisher = DeclareLaunchArgument(
        "start_robot_state_publisher",
        default_value="false",
        description=(
            "If true, starts robot_state_publisher using arm_description/ur5.xacro. "
            "Enable this when launching RViz standalone (otherwise TF frames may be missing)."
        ),
    )

    # UR5 has 6 joints. If you use a different robot, override this.
    declare_plot_joint_count = DeclareLaunchArgument(
        "plot_joint_count",
        default_value="6",
        description="Number of joint position indices to plot from /joint_states/position[i].",
    )

    def _make_nodes(context, *_args, **_kwargs):
        config_path = rviz_config.perform(context).strip()
        rviz_args = []
        if config_path:
            if os.path.exists(config_path):
                rviz_args = ["-d", config_path]
            else:
                return [
                    LogInfo(
                        msg=(
                            f"RViz config not found: '{config_path}'. "
                            "Starting RViz2 without a config (-d)."
                        )
                    ),
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=[],
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),
                ]

        nodes = []

        # Provide robot_description to RViz itself so RobotModel can load from parameter.
        # Keep robot_state_publisher opt-in to avoid double-starting it in sim launches.
        description_share = get_package_share_directory("arm_description")
        xacro_path = os.path.join(description_share, "xacro", "ur5.xacro")
        robot_description = Command(["xacro ", xacro_path])
        nodes.append(
            Node(
                condition=IfCondition(start_robot_state_publisher),
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": robot_description},
                ],
            )
        )

        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=rviz_args,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": robot_description},
                ],
            )
        )

        # Optional: rqt_plot for /joint_states/position[i]. We can't reliably infer joint count
        # here because robot_description may be provided by a different launch.
        try:
            n = int(plot_joint_count.perform(context))
        except Exception:
            n = 6

        rqt_topics = [f"/joint_states/position[{i}]" for i in range(max(0, n))]
        nodes.append(
            ExecuteProcess(
                condition=IfCondition(use_rqt_plot),
                cmd=["rqt_plot", *rqt_topics],
                output="screen",
            )
        )
        return nodes

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz_config,
            declare_start_robot_state_publisher,
            declare_use_rqt_plot,
            declare_plot_joint_count,
            OpaqueFunction(function=_make_nodes),
        ]
    )
