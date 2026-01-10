from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the path to the robot description xacro file,
    # it can be found in the arm_description package under
    # the xacro directory.
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('arm_description'), 'xacro', 'ur5.xacro']
    )

    # Create a robot description parameter by processing the xacro
    # file. It works by calling the xacro CLI tool and gets the stdout
    # from the command and writes it to the attribute.
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    return LaunchDescription(
        [
            # The Robot State Publisher node is needed to publish the transforms
            # of the robot to /tf and /tf_static. It needs the robot description
            # parameter to know the structure of the robot.
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher_go_to_pose',
                output='screen',
                parameters=[robot_description, {'use_sim_time': True}],
            ),
            # The go_to_pose node from the arm_apps package is launched
            Node(
                package='arm_apps',
                executable='go_to_pose_node',
                name='go_to_pose_node',
                output='screen',
                parameters=[
                    robot_description,
                    {'use_sim_time': True},
                    {'base_link': 'base_link'},
                    {'tip_link': 'ee_link'},
                ],
            ),
        ]
    )
