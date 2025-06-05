#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    # Get the path to the package
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    return LaunchDescription([
        # Launch ROSBridge on port 9091
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:=9091'],
            output='screen',
            name='rosbridge_server'
        ),
        
        # Launch the Multi-AGV Pharmacy Dock Server (HTTP API server)
        Node(
            package='hey_agv_new',
            executable='multi_agv_pharmacy_dock_server.py',
            name='multi_agv_pharmacy_dock_server',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Launch the web interface server with API proxying
        Node(
            package='hey_agv_new',
            executable='web_server.py',
            name='agv_web_interface',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'rosbridge_port': 9091
            }]
        )
    ])