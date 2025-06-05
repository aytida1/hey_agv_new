# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('hey_agv_new')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Default paths
    default_map_file = os.path.join(pkg_share, 'urdf', 'sacramento.yaml')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params_5agv_with_docking.yaml')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the localization stack',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info', 
        description='log level'
    )

    # Single shared map server (no namespace - publishes to global /map)
    shared_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Shared lifecycle manager for the map server
    shared_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
            'bond_timeout': 10.0,  # Increase timeout for slower systems
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0
        }]
    )

    # Function to create AGV group (AMCL + Navigation + Docking)
    def create_agv_group(namespace, delay_seconds):
        nav_config = os.path.join(
            get_package_share_directory('hey_agv_new'),
            'config',
            'nav2_params_5agv_with_docking.yaml'
        )

        # AMCL node for this AGV
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                nav_config,
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('tf', '/tf'),  # Publish to global tf topic with absolute path
                ('tf_static', '/tf_static'),  # Publish to global tf_static topic
                ('map', '/map'),  # Use the global map topic
                ('scan', f'/{namespace}/scan'),  # Subscribe to namespaced scan topic with absolute path
                ('initialpose', f'/{namespace}/initialpose'),  # Publish to namespaced initialpose
                ('amcl_pose', f'/{namespace}/amcl_pose'),  # Publish to namespaced amcl_pose
                ('particle_cloud', f'/{namespace}/particle_cloud')  # Publish to namespaced particle cloud
            ]
        )

        # Controller server for this AGV
        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_config, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('cmd_vel', f'/{namespace}/cmd_vel'),
                ('odom', f'/{namespace}/odom'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        )

        # Planner server for this AGV
        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_config, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('map', '/map'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        )

        # Behavior server for this AGV
        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_config, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('cmd_vel', f'/{namespace}/cmd_vel'),
                ('odom', f'/{namespace}/odom'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        )

        # BT Navigator for this AGV
        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_config, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('cmd_vel', f'/{namespace}/cmd_vel'),
                ('odom', f'/{namespace}/odom'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        )

        # Waypoint follower for this AGV
        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_config, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]
        )

        # Velocity smoother for this AGV
        velocity_smoother = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_config, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('cmd_vel', f'/{namespace}/cmd_vel_nav'),
                ('cmd_vel_smoothed', f'/{namespace}/cmd_vel')
            ]
        )

        # Docking server for this AGV
        docking_server = Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_config, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('cmd_vel', f'/{namespace}/cmd_vel'),
                ('odom', f'/{namespace}/odom'),
                ('detected_dock_pose', f'/{namespace}/detected_dock_pose'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        )

        # AprilTag detection node for this AGV
        apriltag_node = Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            namespace=namespace,
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'image_transport': 'raw',
                    'family': '36h11',
                    'size': 0.07,
                    'max_hamming': 0,
                    'z_up': True,
                    'tag_threads': 2,
                    'tag_decimate': 1.0,
                    'tag_blur': 0.0,
                    'tag_refine_edges': True,
                    'tag_debug': False,
                    'publish_tf': True,
                    'camera_frame_id': f'{namespace}/camera_optical_link'  # Override frame_id
                }
            ],
            remappings=[
                ('image_rect', f'/{namespace}/image_rect'),
                ('camera_info', f'/{namespace}/camera_info'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        )

        # Tag transform node for this AGV
        tag_transform_node = Node(
            package='hey_agv_new',
            executable='tag_transform',
            name='tag_transform',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('detected_dock_pose', f'/{namespace}/detected_dock_pose')
            ]
        )

        # Lifecycle manager for this AGV (all navigation nodes)
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                     'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'docking_server'
                ]
            }]
        )

        # Wrap in TimerAction to delay startup
        return TimerAction(
            period=delay_seconds,
            actions=[
                GroupAction([
                    SetParameter('use_sim_time', use_sim_time),
                     amcl_node,
                    controller_server,
                    planner_server,
                    behavior_server,
                    bt_navigator,
                    waypoint_follower,
                    velocity_smoother,
                    docking_server,
                    apriltag_node,
                    tag_transform_node,
                    lifecycle_manager
                ])
            ]
        )

    # Create groups for each AGV with staggered delays
    # AGV1 starts 5 seconds after launch (giving map server time to initialize)
    agv1_group = create_agv_group('agv1', 5.0)
    
    # AGV2 starts 3 seconds after AGV1
    # agv2_group = create_agv_group('agv2', 8.0)
    
    # # AGV3 starts 3 seconds after AGV2
    # agv3_group = create_agv_group('agv3', 11.0)
    
    # # AGV4 starts 3 seconds after AGV3
    # agv4_group = create_agv_group('agv4', 14.0)
    
    # # AGV5 starts 3 seconds after AGV4
    # agv5_group = create_agv_group('agv5', 17.0)

    # Initial pose setter (starts after AGVs are launched)
    initial_pose_setter = TimerAction(
        period=20.0,  # Start after all AGVs are initialized
        actions=[
            Node(
                package='hey_agv_new',
                executable='initial_pose_setter.py',
                name='initial_pose_setter',
                output='screen'
            )
        ]
    )

    # TF Publisher node (starts after AGVs are initialized)
    tf_publisher_node = TimerAction(
        period=18.0,  # Start after AGVs are initialized
        actions=[
            Node(
                package='hey_agv_new',
                executable='tf_publisher',
                name='tf_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add shared map server first (starts immediately)
    ld.add_action(shared_map_server)
    ld.add_action(shared_lifecycle_manager)

    # Add AGV groups with delays
    ld.add_action(agv1_group)  # Starts after 5 seconds
   # ld.add_action(agv2_group)  # Starts after 8 seconds
    # ld.add_action(agv3_group)  # Starts after 11 seconds
    # ld.add_action(agv4_group)  # Starts after 14 seconds
    # ld.add_action(agv5_group)  # Starts after 17 seconds

    # Add TF publisher (starts after 18 seconds)
    ld.add_action(tf_publisher_node)
    
    # Add initial pose setter (starts after 20 seconds)
    ld.add_action(initial_pose_setter)

    return ld