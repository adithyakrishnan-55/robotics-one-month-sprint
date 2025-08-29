# ~/ros2_ws/src/parameters/launch/turtle_param_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parameters',
            executable='turtle_param_node',
            name='turtle_param_node',
            output='screen',
            parameters=['config/example.yaml']
        )
    ])
