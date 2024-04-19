from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='clyde_driver',
            executable='audio_publisher',
            name='audio_publisher'
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{'rotate': 270}]
        ),
        Node(
            package='clyde_driver',
            executable='basic_driver',
            name='basic_driver'
        ),
    ])
