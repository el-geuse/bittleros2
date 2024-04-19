import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Locate the package and URDF file
    pkg_path = os.path.join(get_package_share_directory('clyde_description'))
    urdf_file = os.path.join(pkg_path, 'src/urdf', 'clyde_simple.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
