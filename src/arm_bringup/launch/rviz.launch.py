#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Args
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # Default RViz config path (optional)
    # Put your config file here if you have one:
    #   arm_bringup/rviz/arm.rviz
    default_rviz_config = ""
    try:
        pkg_share = get_package_share_directory("arm_bringup")
        candidate = os.path.join(pkg_share, "rviz", "arm.rviz")
        if os.path.exists(candidate):
            default_rviz_config = candidate
    except Exception:
        # arm_bringup share not found (shouldn't happen if installed), keep empty
        default_rviz_config = ""

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock if available (Gazebo/Sim time).",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Full path to an RViz2 config file. Leave empty to use RViz default.",
    )

    # RViz node (rviz2 executable)
    rviz_node_kwargs = {
        "package": "rviz2",
        "executable": "rviz2",
        "name": "rviz2",
        "output": "screen",
        "parameters": [{"use_sim_time": use_sim_time}],
    }

    # If rviz_config is non-empty, pass -d <config>
    # Launch substitutions are strings, so simplest is: always pass args if default exists.
    # If it's empty, RViz2 ignores "-d" being absent, so we conditionally add it here.
    rviz_args = []
    if default_rviz_config:
        rviz_args = ["-d", rviz_config]
        rviz_node_kwargs["arguments"] = rviz_args

    rviz_node = Node(**rviz_node_kwargs)

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz_config,
            rviz_node,
        ]
    )
