import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import LaunchConfigurationNotEquals

def generate_launch_description():
    # Package name
    pkg_share = FindPackageShare(package='robot_arm_description').find('robot_arm_description')
    
    # Paths
    default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot.rviz')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    # Robot state publisher - FIXED: Use ParameterValue
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationNotEquals('use_sim_time', 'true')
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
