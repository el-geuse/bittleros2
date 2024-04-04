from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='clyde_description')\
    .find('clyde_description')
    default_model_path = os.path.join(pkg_share, 'src/urdf/clyde_simple.urdf')
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_model_path,
            description='Path to the robot urdf file.'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('urdf_file')]),
                    value_type=str
                )
            }]
        ),
    ])
