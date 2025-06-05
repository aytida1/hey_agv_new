#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from nav2_msgs.action import DockRobot, UndockRobot
from geometry_msgs.msg import PoseStamped


class PharmacyDockServer(Node):
    def __init__(self):
        super().__init__('pharmacy_dock_server')
        
        # Action clients for docking and undocking
        self._dock_action_client = ActionClient(self, DockRobot, 'agv1/dock_robot')
        self._undock_action_client = ActionClient(self, UndockRobot, 'agv1/undock_robot')
        
        self.get_logger().info('Pharmacy Dock Server initialized')
        
        # Track current state
        self.current_dock = None
        self.is_docked = False
        self.operation_in_progress = False
        self.last_operation_status = "Ready"
        self.last_operation_result = None
        
        # Thread pool for handling operations
        self.thread_pool = ThreadPoolExecutor(max_workers=1)
        
        # Flask app
        self.app = Flask(__name__)
        CORS(self.app)  # Enable CORS for web interface
        
        self.setup_routes()
        
        # Start Flask server in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask_server, daemon=True)
        self.flask_thread.start()

    def setup_routes(self):
        """Setup Flask routes for the HTTP API"""
        
        @self.app.route('/status', methods=['GET'])
        def get_status():
            """Get current docking status"""
            return jsonify({
                'current_dock': self.current_dock,
                'is_docked': self.is_docked,
                'operation_in_progress': self.operation_in_progress,
                'last_operation_status': self.last_operation_status,
                'last_operation_result': self.last_operation_result
            })
        
        @self.app.route('/dock/<int:dock_number>', methods=['POST'])
        def dock_to_pharmacy(dock_number):
            """Dock to specified pharmacy dock"""
            if dock_number not in [1, 2, 3, 4]:
                return jsonify({'error': 'Invalid dock number. Must be 1-4'}), 400
            
            if self.operation_in_progress:
                return jsonify({'error': 'Operation already in progress'}), 409
            
            # Start docking operation in background
            self.thread_pool.submit(self.perform_dock_operation, dock_number)
            
            return jsonify({
                'message': f'Docking operation to dock {dock_number} initiated',
                'dock_number': dock_number
            })
        
        @self.app.route('/undock', methods=['POST'])
        def undock_from_current():
            """Undock from current position"""
            if not self.is_docked:
                return jsonify({'error': 'Robot is not currently docked'}), 400
            
            if self.operation_in_progress:
                return jsonify({'error': 'Operation already in progress'}), 409
            
            # Start undocking operation in background
            self.thread_pool.submit(self.perform_undock_operation)
            
            return jsonify({'message': 'Undocking operation initiated'})

    def run_flask_server(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=5000, debug=False)

    def perform_dock_operation(self, dock_number):
        """Perform the complete docking operation"""
        self.operation_in_progress = True
        self.last_operation_status = "In Progress"
        
        try:
            # If already docked and requesting different dock, undock first
            if self.is_docked and self.current_dock != dock_number:
                self.get_logger().info(f"Currently docked at dock {self.current_dock}, undocking first...")
                self.last_operation_status = "Undocking from current dock"
                
                if not self.perform_undock_operation_sync():
                    self.last_operation_status = "Failed to undock from current dock"
                    self.last_operation_result = "error"
                    return
                
                # Wait a bit after undocking
                time.sleep(2.0)
            
            # Now dock to the requested dock
            dock_id = f"pharmacist_dock{dock_number}"
            dock_type = "pharmacist_docks"
            
            self.last_operation_status = f"Docking to {dock_id}"
            self.get_logger().info(f"Initiating docking to {dock_id}")
            
            success = self.send_dock_goal(dock_id, dock_type)
            
            if success:
                self.current_dock = dock_number
                self.is_docked = True
                self.last_operation_status = f"Successfully docked to dock {dock_number}"
                self.last_operation_result = "success"
                self.get_logger().info(f"Successfully docked to dock {dock_number}")
            else:
                self.last_operation_status = f"Failed to dock to dock {dock_number}"
                self.last_operation_result = "error"
                self.get_logger().error(f"Failed to dock to dock {dock_number}")
                
        except Exception as e:
            self.last_operation_status = f"Error during docking: {str(e)}"
            self.last_operation_result = "error"
            self.get_logger().error(f"Error during docking operation: {e}")
        
        finally:
            self.operation_in_progress = False

    def perform_undock_operation(self):
        """Perform undocking operation (async version)"""
        self.operation_in_progress = True
        self.last_operation_status = "Undocking"
        
        try:
            success = self.perform_undock_operation_sync()
            
            if success:
                self.current_dock = None
                self.is_docked = False
                self.last_operation_status = "Successfully undocked"
                self.last_operation_result = "success"
                self.get_logger().info("Successfully undocked")
            else:
                self.last_operation_status = "Failed to undock"
                self.last_operation_result = "error"
                self.get_logger().error("Failed to undock")
                
        except Exception as e:
            self.last_operation_status = f"Error during undocking: {str(e)}"
            self.last_operation_result = "error"
            self.get_logger().error(f"Error during undocking operation: {e}")
        
        finally:
            self.operation_in_progress = False

    def perform_undock_operation_sync(self):
        """Perform undocking operation synchronously"""
        if not self._undock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Undock action server not available!')
            return False
        
        goal_msg = UndockRobot.Goal()
        
        self.get_logger().info('Sending undocking goal')
        
        # Send goal and wait for result
        future = self._undock_action_client.send_goal_async(goal_msg)
        
        # Wait for goal to be accepted
        timeout_counter = 0
        while not future.done() and timeout_counter < 100:  # 10 second timeout
            time.sleep(0.1)
            timeout_counter += 1
            rclpy.spin_once(self, timeout_sec=0.0)
        
        if not future.done():
            self.get_logger().error('Timeout waiting for undock goal acceptance')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        timeout_counter = 0
        while not result_future.done() and timeout_counter < 600:  # 60 second timeout for undocking
            time.sleep(0.1)
            timeout_counter += 1
            rclpy.spin_once(self, timeout_sec=0.0)
            
            # Log progress every 10 seconds
            if timeout_counter % 100 == 0:
                self.get_logger().info(f'Waiting for undock result... ({timeout_counter/10:.0f}s elapsed)')
        
        if not result_future.done():
            self.get_logger().error('Timeout waiting for undock result - cancelling goal')
            # Try to cancel the goal
            goal_handle.cancel_goal_async()
            return False
        
        result = result_future.result()
        return result.result.success if result else False

    def send_dock_goal(self, dock_id, dock_type):
        """Send docking goal and wait for result"""
        if not self._dock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Dock action server not available!')
            return False
        
        goal_msg = DockRobot.Goal()
        goal_msg.dock_id = dock_id
        goal_msg.dock_type = dock_type
        goal_msg.navigate_to_staging_pose = True
        goal_msg.use_dock_id = True
        goal_msg.dock_pose = PoseStamped()
        goal_msg.dock_pose.header.frame_id = 'map'
        goal_msg.dock_pose.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'Sending docking goal: {dock_id} ({dock_type})')
        
        # Send goal and wait for result
        future = self._dock_action_client.send_goal_async(goal_msg)
        
        # Wait for goal to be accepted
        timeout_counter = 0
        while not future.done() and timeout_counter < 100:  # 10 second timeout
            time.sleep(0.1)
            timeout_counter += 1
            rclpy.spin_once(self, timeout_sec=0.0)
        
        if not future.done():
            self.get_logger().error('Timeout waiting for dock goal acceptance')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Dock goal rejected')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        timeout_counter = 0
        while not result_future.done() and timeout_counter < 1200:  # 120 second timeout (2 minutes)
            time.sleep(0.1)
            timeout_counter += 1
            rclpy.spin_once(self, timeout_sec=0.0)
            
            # Log progress every 10 seconds
            if timeout_counter % 100 == 0:
                self.get_logger().info(f'Waiting for dock result... ({timeout_counter/10:.0f}s elapsed)')
        
        if not result_future.done():
            self.get_logger().error('Timeout waiting for dock result - cancelling goal')
            # Try to cancel the goal
            goal_handle.cancel_goal_async()
            return False
        
        result = result_future.result()
        return result.result.success if result else False

    def run(self):
        """Run the server with ROS spinning"""
        self.get_logger().info("Pharmacy Dock Server running on http://0.0.0.0:5000")
        self.get_logger().info("API Endpoints:")
        self.get_logger().info("  GET  /status - Get current status")
        self.get_logger().info("  POST /dock/<dock_number> - Dock to pharmacy (1-4)")
        self.get_logger().info("  POST /undock - Undock from current position")
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.01)  # Small sleep to prevent busy waiting
        except KeyboardInterrupt:
            pass

    def shutdown(self):
        """Cleanup when shutting down"""
        try:
            self.thread_pool.shutdown(wait=False)
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        finally:
            try:
                self.destroy_node()
            except:
                pass


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    server = None
    try:
        server = PharmacyDockServer()
        server.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if server is not None:
            server.shutdown()
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
