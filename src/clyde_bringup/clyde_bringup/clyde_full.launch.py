from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clyde_vision',
            executable='person_locator',
            name='person_locator'
        ),
        Node(
            package='clyde_audio',
            executable='audio_processor',
            name='audio_processor'
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='republish',
            arguments=[
                'compressed', 'in/compressed:=image_raw/compressed',
                'raw', 'out:=image_raw/uncompressed'
            ],
            remappings=[
                ('in', 'image_raw/compressed'),
                ('out', 'image_raw/uncompressed')
            ]
        )
    ])
