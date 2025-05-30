from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('hey_agv_new').find('hey_agv_new')
    
    # Get paths to config files
    world_path = os.path.join(pkg_share, 'urdf', 'test_world_new.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'dose_car.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Use Command to process xacro and ParameterValue to convert one datatype to another (here into string)
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    # Launch Gazebo and ExecuteProccess() is used to run commands in terminal
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],  # -r flag to run immediately
        output='screen'
    )

    # Launch Robot State Publisher 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Delay execution of robot_state_publisher
    delayed_robot_state_publisher = TimerAction(
        period=0.0,  # 2 second delay
        actions=[robot_state_publisher]
    )
    

    # Spawn URDF in Gazebo using ign service
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'agv',
            '-topic', '/robot_description',
            '-x', '-3.9',
            '-y', '-15.9',
            '-z', '0.1'  # Slightly above ground to prevent initial collision
        ],
        output='screen'
        
    )
    

    # ros gazebo bridge with premade launch file
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_bridge'), '/launch/ros_gz_bridge.launch.py'
        ]),
        launch_arguments={
            'bridge_name': 'ros_gz_bridge',
            'config_file': bridge_config
        }.items()
        
    )


    remapper_node = Node(
        package = 'hey_agv_new',
        executable = 'frame_remapper',
        output = 'screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        # condition=IfCondition(LaunchConfiguration('gui'))
    )



    # used to remap the frames of odom messages
    mapper_node = Node(
        package="hey_agv_new",
        executable="frame_remapper",
        name="frame_remapper",
        output="screen"
    )
    

    # Scan topic publisher with merged lidar data
    scan_merger_node = Node(
        package="hey_agv_new",
        executable="scan_merger_v2",
        name="scan_merger",
        output="screen"
    )

    

    # Delay execution of spawn_entity
    delayed_spawn = TimerAction(
        period=3.0,  # 1 second delay after Gazebo starts
        actions=[spawn_entity]
    )



    declare_launch_bridge = DeclareLaunchArgument(
        'launch_bridge',
        default_value='true',
        description='Launch the ROS-GZ bridge (set to false if running separately)'
    )


    delayed_mapper_node = TimerAction(
        period=5.0,  # 1 second delay after Gazebo starts
        actions=[mapper_node]
    )

    delayed_scan_merger_node = TimerAction(
        period=5.0,  # 1 second delay after Gazebo starts
        actions=[scan_merger_node]
    )
    
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_launch_bridge,
        bridge,
        delayed_robot_state_publisher,
        gazebo,
        delayed_spawn,
        delayed_mapper_node,
        delayed_scan_merger_node,
        
        # joint_state_publisher,
      
       # robot_state_publisher,   # it is launch first because gazebo spawn needed it
        
        # bridge,
        # remapper_node
    ])