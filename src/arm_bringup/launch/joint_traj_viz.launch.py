from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arm_apps",
            executable="joint_trajectory_viz_node",
            name="joint_trajectory_viz_node",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"plot_frame": "base_link"},
                {"publish_rate_hz": 10.0},
                {"max_points": 2000},
                {"time_window_sec": 0.0},
                {"time_scale": 0.2},
                {"value_scale": 1.0},
                {"value_center": 0.0},
                {"joint_z_offset": 0.05},
                {"line_width": 0.004},
                {"sample_stride": 1},
                {"min_sample_dt_sec": 0.0},
                {"show_commanded": False},
                {"commanded_topic": "/joint_trajectory_controller/joint_trajectory"},
                {"show_labels": False},
                {"clear_service": True},
            ],
        ),
    ])
