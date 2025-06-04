#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch ROSBridge on port 9091
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:=9091'],
            output='screen',
            name='rosbridge_server'
        ),
        
        # Launch the web interface server
        Node(
            package='hey_agv_new',
            executable='web_server.py',
            name='agv_web_interface',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'rosbridge_port': 9091
            }]
        ),
    ])