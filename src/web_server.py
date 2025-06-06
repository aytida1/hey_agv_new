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
import urllib.request
import urllib.parse
import json
from urllib.error import URLError

class AGVWebInterface(Node):
    def __init__(self):
        super().__init__('agv_web_interface')
        
        # Get the web interface directory - fix path for installed package
        self.web_dir = '/home/meditab/car_ws_new/src/hey_agv_new/web_interface'
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
                handler = self.create_request_handler()
                
                with socketserver.TCPServer(("", self.port), handler) as httpd:
                    self.get_logger().info(f"HTTP server started at http://localhost:{self.port}")
                    httpd.serve_forever()
                    
            except Exception as e:
                self.get_logger().error(f"Failed to start HTTP server: {e}")
        
        # Run in separate thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

    def create_request_handler(self):
        """Create a custom request handler that supports API proxying"""
        logger = self.get_logger()
        
        class CustomRequestHandler(http.server.SimpleHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith('/api/dock/'):
                    # Proxy API requests to the pharmacy dock server
                    self.proxy_to_dock_server()
                elif self.path.startswith('/api/psr-dock/'):
                    # Proxy API requests to the PSR dock server
                    self.proxy_to_psr_dock_server()
                else:
                    # Serve static files
                    super().do_GET()
            
            def do_POST(self):
                if self.path.startswith('/api/dock/'):
                    # Proxy API requests to the pharmacy dock server
                    self.proxy_to_dock_server()
                elif self.path.startswith('/api/psr-dock/'):
                    # Proxy API requests to the PSR dock server
                    self.proxy_to_psr_dock_server()
                else:
                    super().do_POST()
                    
            def proxy_to_dock_server(self):
                """Proxy requests to the pharmacy dock server on port 5000"""
                try:
                    # Convert /api/dock/* to direct dock server path
                    dock_path = self.path.replace('/api/dock', '')
                    if dock_path == '':
                        dock_path = '/'
                    
                    dock_url = f"http://localhost:5000{dock_path}"
                    
                    # Handle GET requests
                    if self.command == 'GET':
                        response = urllib.request.urlopen(dock_url)
                        data = response.read()
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(data)
                        
                    # Handle POST requests
                    elif self.command == 'POST':
                        content_length = int(self.headers.get('Content-Length', 0))
                        post_data = self.rfile.read(content_length)
                        
                        req = urllib.request.Request(dock_url, data=post_data, method='POST')
                        req.add_header('Content-Type', 'application/json')
                        
                        response = urllib.request.urlopen(req)
                        data = response.read()
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(data)
                        
                except URLError as e:
                    logger.error(f"Failed to proxy to dock server: {e}")
                    self.send_response(503)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    error_response = json.dumps({
                        "error": "Dock server unavailable",
                        "message": "Please ensure the pharmacy dock server is running on port 5000"
                    })
                    self.wfile.write(error_response.encode())
                except Exception as e:
                    logger.error(f"Proxy error: {e}")
                    self.send_response(500)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    error_response = json.dumps({
                        "error": "Internal server error",
                        "message": str(e)
                    })
                    self.wfile.write(error_response.encode())
                    
            def proxy_to_psr_dock_server(self):
                """Proxy requests to the PSR dock server on port 5002"""
                try:
                    # Convert /api/psr-dock/* to direct PSR dock server path
                    dock_path = self.path.replace('/api/psr-dock', '')
                    if dock_path == '':
                        dock_path = '/'
                    
                    dock_url = f"http://localhost:5002{dock_path}"
                    
                    # Handle GET requests
                    if self.command == 'GET':
                        response = urllib.request.urlopen(dock_url)
                        data = response.read()
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(data)
                        
                    # Handle POST requests
                    elif self.command == 'POST':
                        content_length = int(self.headers.get('Content-Length', 0))
                        post_data = self.rfile.read(content_length)
                        
                        req = urllib.request.Request(dock_url, data=post_data, method='POST')
                        req.add_header('Content-Type', 'application/json')
                        
                        response = urllib.request.urlopen(req)
                        data = response.read()
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(data)
                        
                except URLError as e:
                    logger.error(f"Failed to proxy to PSR dock server: {e}")
                    self.send_response(503)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    error_response = json.dumps({
                        "error": "PSR dock server unavailable",
                        "message": "Please ensure the PSR dock server is running on port 5002"
                    })
                    self.wfile.write(error_response.encode())
                except Exception as e:
                    logger.error(f"PSR dock proxy error: {e}")
                    self.send_response(500)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    error_response = json.dumps({
                        "error": "Internal server error",
                        "message": str(e)
                    })
                    self.wfile.write(error_response.encode())
        
        return CustomRequestHandler
        
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