#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    # Get the path to the package
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    return LaunchDescription([
        # Pharmacy Dock Server
        Node(
            package='hey_agv_new',
            executable='pharmacy_dock_server.py',
            name='pharmacy_dock_server',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        
        # Web Interface Server
        ExecuteProcess(
            cmd=['node', os.path.join(package_dir, 'web_interface', 'server.js')],
            output='screen',
            name='web_interface_server'
        )
    ])
