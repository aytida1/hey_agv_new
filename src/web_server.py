#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import threading
import time
import http.server
import socketserver
import os
import webbrowser
from pathlib import Path

class AGVWebInterface(Node):
    def __init__(self):
        super().__init__('agv_web_interface')
        
        # Get the web interface directory - fix path for installed package
        self.web_dir = Path(__file__).parent.parent.parent / 'share' / 'hey_agv_new' / 'web_interface'
        self.port = 3000  # Changed from 8080 to 3000
        
        self.get_logger().info(f"Starting AGV Web Interface on port {self.port}")
        self.get_logger().info(f"Web directory: {self.web_dir}")
        
        # Start ROSBridge server
        self.start_rosbridge()
        
        # Start HTTP server
        self.start_http_server()
        
        # Open browser
        self.open_browser()
        
    def start_rosbridge(self):
        """Start the ROSBridge WebSocket server"""
        def run_rosbridge():
            try:
                # Start rosbridge server for ROS Jazzy on port 9091
                cmd = ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:=9091']
                self.get_logger().info("Starting ROSBridge WebSocket server on port 9091...")
                subprocess.run(cmd)
            except Exception as e:
                self.get_logger().error(f"Failed to start ROSBridge: {e}")
                self.get_logger().info("Please install rosbridge_server: sudo apt install ros-jazzy-rosbridge-server")
        
        # Run in separate thread
        rosbridge_thread = threading.Thread(target=run_rosbridge, daemon=True)
        rosbridge_thread.start()
        
    def start_http_server(self):
        """Start HTTP server to serve web interface"""
        def run_server():
            try:
                os.chdir(self.web_dir)
                handler = http.server.SimpleHTTPRequestHandler
                
                with socketserver.TCPServer(("", self.port), handler) as httpd:
                    self.get_logger().info(f"HTTP server started at http://localhost:{self.port}")
                    httpd.serve_forever()
                    
            except Exception as e:
                self.get_logger().error(f"Failed to start HTTP server: {e}")
        
        # Run in separate thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
    def open_browser(self):
        """Open web browser after a delay"""
        def delayed_open():
            time.sleep(3)  # Wait for servers to start
            try:
                url = f"http://localhost:{self.port}"
                self.get_logger().info(f"Opening browser at: {url}")
                webbrowser.open(url)
            except Exception as e:
                self.get_logger().error(f"Failed to open browser: {e}")
                
        browser_thread = threading.Thread(target=delayed_open, daemon=True)
        browser_thread.start()

def main(args=None):
    rclpy.init(args=args)
    
    node = AGVWebInterface()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AGV Web Interface...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()