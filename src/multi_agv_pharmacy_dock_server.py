#!/usr/bin/env python3
"""
Multi-AGV Pharmacy Dock Server
Handles docking operations for multiple AGVs with dock reservation system
Enhanced with collision and stalling detection for immediate goal cancellation
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Optional, Set
import math

from nav2_msgs.action import DockRobot, UndockRobot
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class MultiAGVPharmacyDockServer(Node):
    def __init__(self):
        super().__init__('multi_agv_pharmacy_dock_server')
        
        self.get_logger().info('Multi-AGV Pharmacy Dock Server with Collision & Stalling Detection initialized')
        
        # Track AGV states - each AGV has its own state
        self.agv_states = {
            'agv1': {
                'current_dock': None,
                'is_docked': False,
                'operation_in_progress': False,
                'last_operation_status': "Ready",
                'last_operation_result': None,
                'collision_detected': False,
                'stalled': False,
                'last_position': None,
                'last_movement_time': time.time(),
                'velocity_threshold': 0.05,  # m/s threshold for stall detection
                'stall_timeout': 10.0,  # seconds without movement before considering stalled
                'current_goal_handle': None,  # Track active navigation goals
                'emergency_stopped': False
            },
            'agv2': {
                'current_dock': None,
                'is_docked': False,
                'operation_in_progress': False,
                'last_operation_status': "Ready",
                'last_operation_result': None,
                'collision_detected': False,
                'stalled': False,
                'last_position': None,
                'last_movement_time': time.time(),
                'velocity_threshold': 0.05,
                'stall_timeout': 10.0,
                'current_goal_handle': None,
                'emergency_stopped': False
            },
            'agv3': {
                'current_dock': None,
                'is_docked': False,
                'operation_in_progress': False,
                'last_operation_status': "Ready",
                'last_operation_result': None,
                'collision_detected': False,
                'stalled': False,
                'last_position': None,
                'last_movement_time': time.time(),
                'velocity_threshold': 0.05,
                'stall_timeout': 10.0,
                'current_goal_handle': None,
                'emergency_stopped': False
            },
            'agv4': {
                'current_dock': None,
                'is_docked': False,
                'operation_in_progress': False,
                'last_operation_status': "Ready",
                'last_operation_result': None,
                'collision_detected': False,
                'stalled': False,
                'last_position': None,
                'last_movement_time': time.time(),
                'velocity_threshold': 0.05,
                'stall_timeout': 10.0,
                'current_goal_handle': None,
                'emergency_stopped': False
            },
            'agv5': {
                'current_dock': None,
                'is_docked': False,
                'operation_in_progress': False,
                'last_operation_status': "Ready",
                'last_operation_result': None,
                'collision_detected': False,
                'stalled': False,
                'last_position': None,
                'last_movement_time': time.time(),
                'velocity_threshold': 0.05,
                'stall_timeout': 10.0,
                'current_goal_handle': None,
                'emergency_stopped': False
            }
        }
        
        # Global dock reservation system - tracks which AGV is using which dock
        self.dock_reservations = {
            1: None,  # None means dock is free, otherwise contains AGV ID
            2: None,
            3: None,
            4: None
        }
        
        # Action clients for each AGV (created dynamically when needed)
        self.dock_action_clients: Dict[str, ActionClient] = {}
        self.undock_action_clients: Dict[str, ActionClient] = {}
        
        # Thread pool for handling operations
        self.thread_pool = ThreadPoolExecutor(max_workers=5)  # Allow multiple concurrent operations
        
        # Flask app
        self.app = Flask(__name__)
        CORS(self.app)  # Enable CORS for web interface
        
        self.setup_routes()
        
        # Start Flask server in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask_server, daemon=True)
        self.flask_thread.start()

    def get_or_create_action_clients(self, agv_id: str):
        """Get or create action clients for the specified AGV"""
        if agv_id not in self.dock_action_clients:
            self.dock_action_clients[agv_id] = ActionClient(
                self, DockRobot, f'{agv_id}/dock_robot'
            )
            self.undock_action_clients[agv_id] = ActionClient(
                self, UndockRobot, f'{agv_id}/undock_robot'
            )
            self.get_logger().info(f'Created action clients for {agv_id}')
        
        return self.dock_action_clients[agv_id], self.undock_action_clients[agv_id]

    def setup_routes(self):
        """Setup Flask routes for the HTTP API"""
        
        @self.app.route('/status', methods=['GET'])
        def get_global_status():
            """Get global docking status for all AGVs"""
            return jsonify({
                'agv_states': self.agv_states,
                'dock_reservations': self.dock_reservations,
                'available_docks': [dock for dock, agv in self.dock_reservations.items() if agv is None]
            })
        
        @self.app.route('/status/<agv_id>', methods=['GET'])
        def get_agv_status(agv_id):
            """Get docking status for specific AGV"""
            if agv_id not in self.agv_states:
                return jsonify({'error': f'Unknown AGV: {agv_id}'}), 400
            
            agv_state = self.agv_states[agv_id].copy()
            agv_state['agv_id'] = agv_id
            return jsonify(agv_state)
        
        @self.app.route('/dock/<agv_id>/<int:dock_number>', methods=['POST'])
        def dock_agv_to_pharmacy(agv_id, dock_number):
            """Dock specified AGV to specified pharmacy dock"""
            if agv_id not in self.agv_states:
                return jsonify({'error': f'Unknown AGV: {agv_id}'}), 400
            
            if dock_number not in [1, 2, 3, 4]:
                return jsonify({'error': 'Invalid dock number. Must be 1-4'}), 400
            
            # Check if AGV is busy
            if self.agv_states[agv_id]['operation_in_progress']:
                return jsonify({'error': f'{agv_id} operation already in progress'}), 409
            
            # Check if dock is already occupied by another AGV
            if self.dock_reservations[dock_number] is not None and self.dock_reservations[dock_number] != agv_id:
                occupying_agv = self.dock_reservations[dock_number]
                return jsonify({
                    'error': f'Dock {dock_number} is already occupied by {occupying_agv}'
                }), 409
            
            # Check if AGV is already docked at this dock
            if (self.agv_states[agv_id]['is_docked'] and 
                self.agv_states[agv_id]['current_dock'] == dock_number):
                return jsonify({
                    'message': f'{agv_id} is already docked at dock {dock_number}'
                })
            
            # Start docking operation in background
            self.thread_pool.submit(self.perform_dock_operation, agv_id, dock_number)
            
            return jsonify({
                'message': f'Docking operation for {agv_id} to dock {dock_number} initiated',
                'agv_id': agv_id,
                'dock_number': dock_number
            })
        
        @self.app.route('/undock/<agv_id>', methods=['POST'])
        def undock_agv_from_current(agv_id):
            """Undock specified AGV from current position"""
            if agv_id not in self.agv_states:
                return jsonify({'error': f'Unknown AGV: {agv_id}'}), 400
            
            if not self.agv_states[agv_id]['is_docked']:
                return jsonify({'error': f'{agv_id} is not currently docked'}), 400
            
            if self.agv_states[agv_id]['operation_in_progress']:
                return jsonify({'error': f'{agv_id} operation already in progress'}), 409
            
            # Start undocking operation in background
            self.thread_pool.submit(self.perform_undock_operation, agv_id)
            
            return jsonify({'message': f'Undocking operation for {agv_id} initiated'})
        
        @self.app.route('/dock_reservations', methods=['GET'])
        def get_dock_reservations():
            """Get current dock reservations"""
            return jsonify(self.dock_reservations)

    def run_flask_server(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=5000, debug=False)

    def perform_dock_operation(self, agv_id: str, dock_number: int):
        """Perform the complete docking operation for specified AGV"""
        self.agv_states[agv_id]['operation_in_progress'] = True
        self.agv_states[agv_id]['last_operation_status'] = "In Progress"
        
        try:
            # If AGV is already docked and requesting different dock, undock first
            if (self.agv_states[agv_id]['is_docked'] and 
                self.agv_states[agv_id]['current_dock'] != dock_number):
                
                self.get_logger().info(f"{agv_id} currently docked at dock {self.agv_states[agv_id]['current_dock']}, undocking first...")
                self.agv_states[agv_id]['last_operation_status'] = "Undocking from current dock"
                
                if not self.perform_undock_operation_sync(agv_id):
                    self.agv_states[agv_id]['last_operation_status'] = "Failed to undock from current dock"
                    self.agv_states[agv_id]['last_operation_result'] = "error"
                    return
                
                # Wait a bit after undocking
                time.sleep(2.0)
            
            # Reserve the dock
            self.dock_reservations[dock_number] = agv_id
            
            # Now dock to the requested dock
            dock_id = f"pharmacist_dock{dock_number}"
            dock_type = "pharmacist_docks"
            
            self.agv_states[agv_id]['last_operation_status'] = f"Docking to {dock_id}"
            self.get_logger().info(f"Initiating docking for {agv_id} to {dock_id}")
            
            success = self.send_dock_goal(agv_id, dock_id, dock_type)
            
            if success:
                self.agv_states[agv_id]['current_dock'] = dock_number
                self.agv_states[agv_id]['is_docked'] = True
                self.agv_states[agv_id]['last_operation_status'] = f"Successfully docked to dock {dock_number}"
                self.agv_states[agv_id]['last_operation_result'] = "success"
                self.get_logger().info(f"{agv_id} successfully docked to dock {dock_number}")
            else:
                # Release the dock reservation on failure
                self.dock_reservations[dock_number] = None
                self.agv_states[agv_id]['last_operation_status'] = f"Failed to dock to dock {dock_number}"
                self.agv_states[agv_id]['last_operation_result'] = "error"
                self.get_logger().error(f"{agv_id} failed to dock to dock {dock_number}")
                
        except Exception as e:
            # Release the dock reservation on error
            if self.dock_reservations.get(dock_number) == agv_id:
                self.dock_reservations[dock_number] = None
            self.agv_states[agv_id]['last_operation_status'] = f"Error during docking: {str(e)}"
            self.agv_states[agv_id]['last_operation_result'] = "error"
            self.get_logger().error(f"Error during docking operation for {agv_id}: {e}")
        
        finally:
            self.agv_states[agv_id]['operation_in_progress'] = False

    def perform_undock_operation(self, agv_id: str):
        """Perform undocking operation for specified AGV"""
        self.agv_states[agv_id]['operation_in_progress'] = True
        self.agv_states[agv_id]['last_operation_status'] = "Undocking in progress"
        
        try:
            success = self.perform_undock_operation_sync(agv_id)
            
            if success:
                self.agv_states[agv_id]['last_operation_status'] = "Successfully undocked"
                self.agv_states[agv_id]['last_operation_result'] = "success"
                self.get_logger().info(f"{agv_id} successfully undocked")
            else:
                self.agv_states[agv_id]['last_operation_status'] = "Failed to undock"
                self.agv_states[agv_id]['last_operation_result'] = "error"
                self.get_logger().error(f"{agv_id} failed to undock")
                
        except Exception as e:
            self.agv_states[agv_id]['last_operation_status'] = f"Error during undocking: {str(e)}"
            self.agv_states[agv_id]['last_operation_result'] = "error"
            self.get_logger().error(f"Error during undocking operation for {agv_id}: {e}")
        
        finally:
            self.agv_states[agv_id]['operation_in_progress'] = False

    def perform_undock_operation_sync(self, agv_id: str) -> bool:
        """Synchronous undocking operation"""
        try:
            success = self.send_undock_goal(agv_id)
            
            if success:
                # Release dock reservation
                current_dock = self.agv_states[agv_id]['current_dock']
                if current_dock is not None:
                    self.dock_reservations[current_dock] = None
                
                # Update AGV state
                self.agv_states[agv_id]['current_dock'] = None
                self.agv_states[agv_id]['is_docked'] = False
                
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error in synchronous undock operation for {agv_id}: {e}")
            return False

    def send_dock_goal(self, agv_id: str, dock_id: str, dock_type: str) -> bool:
        """Send docking goal to the action server for specified AGV"""
        try:
            dock_client, _ = self.get_or_create_action_clients(agv_id)
            
            goal_msg = DockRobot.Goal()
            goal_msg.dock_id = dock_id
            goal_msg.dock_type = dock_type
            goal_msg.navigate_to_staging_pose = True
            goal_msg.use_dock_id = True
            goal_msg.dock_pose = PoseStamped()
            goal_msg.dock_pose.header.frame_id = 'map'
            goal_msg.dock_pose.header.stamp = self.get_clock().now().to_msg()

            if not dock_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f'Dock action server for {agv_id} not available!')
                return False

            self.get_logger().info(f'Sending docking goal for {agv_id}: {dock_id} ({dock_type})')
            
            send_goal_future = dock_client.send_goal_async(goal_msg)
            
            # Wait for goal to be accepted
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            if not send_goal_future.done():
                self.get_logger().error(f'Goal send timeout for {agv_id}')
                return False
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Docking goal rejected for {agv_id}')
                return False

            self.get_logger().info(f'Docking goal accepted for {agv_id}, waiting for result...')
            
            # Wait for result - no timeout, let it run until completion
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            if not result_future.done():
                self.get_logger().error(f'Docking operation failed to complete for {agv_id}')
                return False
            
            result = result_future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info(f'Docking operation succeeded for {agv_id}')
                return True
            else:
                self.get_logger().error(f'Docking operation failed for {agv_id} with status: {result.status}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Exception in send_dock_goal for {agv_id}: {e}')
            return False

    def send_undock_goal(self, agv_id: str) -> bool:
        """Send undocking goal to the action server for specified AGV"""
        try:
            _, undock_client = self.get_or_create_action_clients(agv_id)
            
            goal_msg = UndockRobot.Goal()

            if not undock_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f'Undock action server for {agv_id} not available!')
                return False

            self.get_logger().info(f'Sending undocking goal for {agv_id}')
            
            send_goal_future = undock_client.send_goal_async(goal_msg)
            
            # Wait for goal to be accepted
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            if not send_goal_future.done():
                self.get_logger().error(f'Undock goal send timeout for {agv_id}')
                return False
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Undocking goal rejected for {agv_id}')
                return False

            self.get_logger().info(f'Undocking goal accepted for {agv_id}, waiting for result...')
            
            # Wait for result - no timeout, let it run until completion
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            if not result_future.done():
                self.get_logger().error(f'Undocking operation failed to complete for {agv_id}')
                return False
            
            result = result_future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info(f'Undocking operation succeeded for {agv_id}')
                return True
            else:
                self.get_logger().error(f'Undocking operation failed for {agv_id} with status: {result.status}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Exception in send_undock_goal for {agv_id}: {e}')
            return False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        dock_server = MultiAGVPharmacyDockServer()
        
        dock_server.get_logger().info('🚀 Multi-AGV Pharmacy Dock Server starting...')
        dock_server.get_logger().info('📊 Web server running on http://localhost:5000')
        dock_server.get_logger().info('📋 API Endpoints:')
        dock_server.get_logger().info('   GET  /status - Global status')
        dock_server.get_logger().info('   GET  /status/<agv_id> - AGV-specific status')
        dock_server.get_logger().info('   POST /dock/<agv_id>/<dock_number> - Dock AGV')
        dock_server.get_logger().info('   POST /undock/<agv_id> - Undock AGV')
        dock_server.get_logger().info('   GET  /dock_reservations - Current reservations')
        
        rclpy.spin(dock_server)
        
    except KeyboardInterrupt:
        dock_server.get_logger().info('🛑 Multi-AGV Pharmacy Dock Server shutting down...')
    except Exception as e:
        dock_server.get_logger().error(f'💥 Server error: {e}')
    finally:
        dock_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
