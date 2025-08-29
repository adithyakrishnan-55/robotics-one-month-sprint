from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'parameters',
        'example.yaml'
    )

    return LaunchDescription([
        Node(
            package='parameters',       # your package name
            executable='my_node',       # the node executable
            name='my_node',
            parameters=[config]
        )
    ])
