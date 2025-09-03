from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directory
    pkg_ai_car_description = FindPackageShare(package='ai_car_description').find('ai_car_description')
    urdf_file = os.path.join(pkg_ai_car_description, 'urdf', 'ai_car.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description_content
            }]
        ),
        
        # Start Gazebo with empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
		'/ai_car_ws/src/ai_car_description/worlds/simple_obstacles.world'],
            output='screen'
        ),
        
        # Spawn AI Car in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_ai_car',
            arguments=[
                '-entity', 'ai_car',
                '-topic', '/robot_description',
                '-x', '0.0',
                '-y', '0.0', 
                '-z', '0.5'
            ],
            output='screen'
        )
    ])
