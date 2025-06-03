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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
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
    default_params_file = os.path.join(pkg_share, 'config', 'amcl_config.yaml')

    lifecycle_nodes = ['map_server', 'amcl']

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
            'node_names': ['map_server']
        }]
    )

    # Function to create AGV group (AMCL only, no map server)
    def create_agv_group(namespace, odom_frame, base_frame):
        # Configure parameters for this namespace
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file,
                root_key=namespace,
                param_rewrites={
                    'global_frame_id': 'map',
                    'odom_frame_id': odom_frame,
                    'base_frame_id': base_frame,
                    'scan_topic': 'scan'
                },
                convert_types=True,
            ),
            allow_substs=True,
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
                configured_params,
                {
                    'use_sim_time': use_sim_time,
                    'global_frame_id': 'map',
                    'odom_frame_id': odom_frame,
                    'base_frame_id': base_frame,
                    'scan_topic': 'scan'
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('map', '/map')  # Remap to use the global map topic
            ]
        )

        # Lifecycle manager for this AGV (AMCL only)
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=namespace,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['amcl']
            }]
        )

        return GroupAction([
            SetParameter('use_sim_time', use_sim_time),
            amcl_node,
            lifecycle_manager
        ])

    # Create groups for each AGV
    agv1_group = create_agv_group('agv1', 'odom', 'base_link')
    agv2_group = create_agv_group('agv2', 'odom', 'base_link')

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

    # Add shared map server first
    ld.add_action(shared_map_server)
    ld.add_action(shared_lifecycle_manager)

    # Add AGV groups
    ld.add_action(agv1_group)
    ld.add_action(agv2_group)

    return ld