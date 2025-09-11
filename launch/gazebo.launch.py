import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare(package='robot_arm_description').find('robot_arm_description')
    gazebo_ros_dir = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    
    # Paths
    default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    robot_name_in_model = 'robot'
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Declare launch arguments  
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_model_path_cmd = DeclareLaunchArgument(
        'model', 
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', 
        default_value='0.0',
        description='x position of robot'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='y position of robot'
    )
        
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', 
        default_value='0.0',
        description='z position of robot'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py'))
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py'))
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model]),
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, 
                  '-topic', 'robot_description',
                  '-x', x_pose,
                  '-y', y_pose,
                  '-z', z_pose],
        output='screen'
    )

    # Load controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_slider_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['slider_position_controller', '--controller-manager', '/controller_manager'],
    )

    load_arm_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller', '--controller-manager', '/controller_manager'],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_slider_position_controller)
    ld.add_action(load_arm_position_controller)

    return ld
