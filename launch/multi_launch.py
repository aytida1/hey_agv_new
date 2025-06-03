#!/usr/bin/env python3

import os
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Package directory
    pkg_hey_agv_new = FindPackageShare(package='hey_agv_new').find('hey_agv_new')

    # File paths
    xacro_file = os.path.join(pkg_hey_agv_new, 'urdf', 'dose_car.urdf.xacro')
    world_path = os.path.join(pkg_hey_agv_new, 'urdf', 'test_world_new.sdf')

    # Launch configuration variables
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # AGV configurations with much longer delays to avoid conflicts
    agv_configs = [
        {
            'name': 'agv_1',
            'namespace': 'agv1',
            'x': '-3.9',
            'y': '-15.9',
            'z': '0.1',
            'spawn_delay': 10.0,  # Much longer delay for first AGV
            'nodes_delay': 12.0
        },
        {
            'name': 'agv_2',
            'namespace': 'agv2',
            'x': '0',
            'y': '0',
            'z': '0.1',
            'spawn_delay': 20.0,  # Much longer delay for second AGV
            'nodes_delay': 22.0
        },
        # Temporarily comment out third AGV for testing
        # {
        #     'name': 'agv_3',
        #     'namespace': 'agv3',
        #     'x': '-2.8',
        #     'y': '-14.3',
        #     'z': '0.1',
        #     'spawn_delay': 30.0,
        #     'nodes_delay': 32.0
        # }
    ]
    
    def generate_bridge_config(agv_configs):
        """Generate dynamic bridge configuration for all AGVs"""
        bridge_config_data = []
        
        for agv_config in agv_configs:
            namespace = agv_config['namespace']
            model_name = agv_config['name']
            
            # Command velocity (ROS → Gazebo)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/cmd_vel",
                'gz_topic_name': f"/{namespace}/cmd_vel",
                'ros_type_name': "geometry_msgs/msg/Twist",
                'gz_type_name': "gz.msgs.Twist",
                'direction': "ROS_TO_GZ"
            })
            
            # Odometry (Gazebo → ROS)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/odom",
                'gz_topic_name': f"/{namespace}/odom",
                'ros_type_name': "nav_msgs/msg/Odometry",
                'gz_type_name': "gz.msgs.Odometry",
                'direction': "GZ_TO_ROS"
            })
            
            # Joint states (Gazebo → ROS)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/joint_states",
                'gz_topic_name': f"/{namespace}/joint_states",
                'ros_type_name': "sensor_msgs/msg/JointState",
                'gz_type_name': "gz.msgs.Model",
                'direction': "GZ_TO_ROS"
            })
            
            # Lidar 1 scan (Gazebo → ROS)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/scan1",
                'gz_topic_name': f"/{namespace}/lidar1",
                'ros_type_name': "sensor_msgs/msg/LaserScan",
                'gz_type_name': "gz.msgs.LaserScan",
                'direction': "GZ_TO_ROS"
            })
            
            # Lidar 2 scan (Gazebo → ROS)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/scan2",
                'gz_topic_name': f"/{namespace}/lidar2",
                'ros_type_name': "sensor_msgs/msg/LaserScan",
                'gz_type_name': "gz.msgs.LaserScan",
                'direction': "GZ_TO_ROS"
            })
            
            # Camera image (Gazebo → ROS)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/image_rect",
                'gz_topic_name': f"/world/default/model/{model_name}/link/{namespace}/camera/sensor/camera/image",
                'ros_type_name': "sensor_msgs/msg/Image",
                'gz_type_name': "gz.msgs.Image",
                'direction': "GZ_TO_ROS"
            })
            
            # Camera info (Gazebo → ROS)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/camera_info",
                'gz_topic_name': f"/world/default/model/{model_name}/link/{namespace}/camera/sensor/camera/camera_info",
                'ros_type_name': "sensor_msgs/msg/CameraInfo",
                'gz_type_name': "gz.msgs.CameraInfo",
                'direction': "GZ_TO_ROS"
            })
            
            # Lift controller (ROS → Gazebo)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/lift_cmd",
                'gz_topic_name': f"/{namespace}/lift_cmd",
                'ros_type_name': "std_msgs/msg/Float64",
                'gz_type_name': "gz.msgs.Double",
                'direction': "ROS_TO_GZ"
            })
            
            # Lift servo controller (ROS → Gazebo)
            bridge_config_data.append({
                'ros_topic_name': f"/{namespace}/lift_servo_cmd",
                'gz_topic_name': f"/{namespace}/lift_servo_cmd",
                'ros_type_name': "std_msgs/msg/Float64",
                'gz_type_name': "gz.msgs.Double",
                'direction': "ROS_TO_GZ"
            })
        
        return bridge_config_data

    # Generate dynamic bridge configuration
    dynamic_bridge_config = generate_bridge_config(agv_configs)
    
    # Write dynamic bridge config to temporary file
    temp_bridge_config = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(dynamic_bridge_config, temp_bridge_config, default_flow_style=False)
    temp_bridge_config.close()
    
    # Launch Gazebo with GUI
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],  # -r flag to run immediately
        output='screen'
    )
    
    # ROS-Gazebo bridge with dynamic configuration - delayed start
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_bridge'), '/launch/ros_gz_bridge.launch.py'
        ]),
        launch_arguments={
            'bridge_name': 'ros_gz_bridge',
            'config_file': temp_bridge_config.name
        }.items()
    )
    
    def create_agv_nodes(agv_config):
        """Create all nodes for a single AGV with improved error handling"""
        
        # Robot description for this AGV with unique scene name
        robot_description_content = ParameterValue(
            Command([
                'xacro ', xacro_file, 
                ' robot_name:=', agv_config["name"],
                ' namespace:=', agv_config["namespace"]
            ]), 
            value_type=str
        )
        
        # Robot State Publisher with namespace
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=agv_config['namespace'],
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0,  # Ensure regular publishing
                # Remove frame_prefix since URDF already handles namespace prefixing
            }],
            respawn=True,
            respawn_delay=2.0
        )
        
        # Spawn entity in Gazebo with retry logic
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', agv_config['name'],
                '-topic', f"/{agv_config['namespace']}/robot_description",
                '-x', agv_config['x'],
                '-y', agv_config['y'],
                '-z', agv_config['z'],
                '--timeout', '30.0'  # Add timeout for spawn
            ],
            # namespace=agv_config['namespace'],
            output="screen",
            respawn=False
        )
        
        # Joint state publisher with namespace
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=agv_config['namespace'],
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }],
            respawn=True,
            respawn_delay=2.0
        )
        
        # Frame mapper node
        mapper_node = Node(
            package="hey_agv_new",
            executable="frame_remapper",
            name="frame_remapper",
            namespace=agv_config['namespace'],
            output="screen",
            respawn=False,  # Disable respawn to prevent duplicates
            respawn_delay=2.0
        )
        
        # Scan merger node  
        scan_merger_node = Node(
            package="hey_agv_new",
            executable="scan_merger_v2",
            name="scan_merger",
            namespace=agv_config['namespace'],
            output="screen",
            respawn=False,  # Disable respawn to prevent duplicates
            respawn_delay=2.0
        )
        
        # Static transform publisher for base_footprint (since Gazebo doesn't publish fixed joints)
        static_transform_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_static_tf',
            namespace=agv_config['namespace'],
            arguments=[
                '0', '0', '-0.071', '0', '0', '0',  # x y z roll pitch yaw
                f"{agv_config['namespace']}/base_footprint",
                f"{agv_config['namespace']}/base_link"
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
        
        # Create delayed actions with better sequencing
        delayed_robot_state_publisher = TimerAction(
            period=2.0,  # Start robot state publisher early
            actions=[robot_state_publisher]
        )
        
        delayed_spawn = TimerAction(
            period=agv_config['spawn_delay'],
            actions=[spawn_entity]
        )
        
        delayed_mapper_node = TimerAction(
            period=agv_config['nodes_delay'],
            actions=[mapper_node]
        )
        
        delayed_scan_merger_node = TimerAction(
            period=agv_config['nodes_delay'] + 1.0,  # Start after mapper
            actions=[scan_merger_node]
        )
        
        delayed_static_tf = TimerAction(
            period=1.0,  # Start static transform early
            actions=[static_transform_publisher]
        )
        
        return [
            delayed_static_tf,
            delayed_robot_state_publisher,
            delayed_spawn,
            delayed_mapper_node,
            delayed_scan_merger_node
        ]

    declare_launch_bridge = DeclareLaunchArgument(
        'launch_bridge',
        default_value='true',
        description='Launch the ROS-GZ bridge (set to false if running separately)'
    )
    
    # Create launch description with all components
    launch_entities = [
        declare_use_sim_time,
        declare_launch_bridge,
        gazebo,
        TimerAction(period=4.0, actions=[bridge]),  # Reduced bridge delay
    ]
    
    # Add all AGV nodes with proper sequencing
    for agv_config in agv_configs:
        agv_nodes = create_agv_nodes(agv_config)
        launch_entities.extend(agv_nodes)
    
    return LaunchDescription(launch_entities)