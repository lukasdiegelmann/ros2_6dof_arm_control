import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gazebo_ros",
            executable="gzserver",
            name="gazebo",
            output="screen",
            arguments=["-s", "libgazebo_ros_factory.so"],
        ),
        Node(
            package="ur_bringup",
            executable="spawn_ur5",
            name="spawn_ur5",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
    ])