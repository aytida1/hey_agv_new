#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
import threading
from collections import defaultdict

class AGVSpawnMonitor(Node):
    """Monitor AGV spawning progress and health status"""
    
    def __init__(self):
        super().__init__('agv_spawn_monitor')
        
        # Track AGV status
        self.agv_status = {}
        self.expected_agvs = ['agv1', 'agv2', 'agv3']
        self.last_message_time = defaultdict(lambda: defaultdict(float))
        
        # Initialize status tracking
        for agv in self.expected_agvs:
            self.agv_status[agv] = {
                'odom_active': False,
                'lidar1_active': False,
                'lidar2_active': False,
                'spawned': False,
                'last_seen': 0.0
            }
        
        # Create subscribers for each AGV
        self.subscribers = {}
        for agv in self.expected_agvs:
            self.subscribers[agv] = {
                'odom': self.create_subscription(
                    Odometry, f'/{agv}/odom',
                    lambda msg, ns=agv: self.odom_callback(msg, ns), 10
                ),
                'lidar1': self.create_subscription(
                    LaserScan, f'/{agv}/scan1',
                    lambda msg, ns=agv: self.lidar1_callback(msg, ns), 10
                ),
                'lidar2': self.create_subscription(
                    LaserScan, f'/{agv}/scan2',
                    lambda msg, ns=agv: self.lidar2_callback(msg, ns), 10
                )
            }
        
        # Status reporting timer
        self.status_timer = self.create_timer(5.0, self.report_status)
        
        # Health check timer
        self.health_timer = self.create_timer(2.0, self.health_check)
        
        self.get_logger().info("AGV Spawn Monitor started. Monitoring: " + str(self.expected_agvs))
    
    def odom_callback(self, msg, namespace):
        """Handle odometry messages"""
        current_time = time.time()
        self.agv_status[namespace]['odom_active'] = True
        self.agv_status[namespace]['spawned'] = True
        self.agv_status[namespace]['last_seen'] = current_time
        self.last_message_time[namespace]['odom'] = current_time
    
    def lidar1_callback(self, msg, namespace):
        """Handle lidar1 scan messages"""
        current_time = time.time()
        self.agv_status[namespace]['lidar1_active'] = True
        self.last_message_time[namespace]['lidar1'] = current_time
    
    def lidar2_callback(self, msg, namespace):
        """Handle lidar2 scan messages"""
        current_time = time.time()
        self.agv_status[namespace]['lidar2_active'] = True
        self.last_message_time[namespace]['lidar2'] = current_time
    
    def health_check(self):
        """Check health of AGV topics"""
        current_time = time.time()
        
        for agv in self.expected_agvs:
            # Reset active flags if no recent messages
            for topic in ['odom', 'lidar1', 'lidar2']:
                last_msg_time = self.last_message_time[agv][topic]
                if last_msg_time > 0 and (current_time - last_msg_time) > 10.0:
                    if topic == 'odom':
                        self.agv_status[agv]['odom_active'] = False
                    elif topic == 'lidar1':
                        self.agv_status[agv]['lidar1_active'] = False
                    elif topic == 'lidar2':
                        self.agv_status[agv]['lidar2_active'] = False
    
    def report_status(self):
        """Report current status of all AGVs"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("AGV SPAWN STATUS REPORT")
        self.get_logger().info("=" * 60)
        
        all_spawned = True
        all_healthy = True
        
        for agv in self.expected_agvs:
            status = self.agv_status[agv]
            
            # Determine overall health
            spawned = status['spawned']
            odom_ok = status['odom_active']
            lidar1_ok = status['lidar1_active']
            lidar2_ok = status['lidar2_active']
            
            if not spawned:
                all_spawned = False
                health_status = "❌ NOT SPAWNED"
            elif odom_ok and lidar1_ok and lidar2_ok:
                health_status = "✅ HEALTHY"
            elif odom_ok:
                health_status = "⚠️  PARTIAL (Missing Lidar)"
                all_healthy = False
            else:
                health_status = "❌ UNHEALTHY"
                all_healthy = False
            
            self.get_logger().info(f"{agv:>6}: {health_status}")
            self.get_logger().info(f"        Odom: {'✓' if odom_ok else '✗'} | "
                                 f"Lidar1: {'✓' if lidar1_ok else '✗'} | "
                                 f"Lidar2: {'✓' if lidar2_ok else '✗'}")
        
        self.get_logger().info("=" * 60)
        
        if all_spawned and all_healthy:
            self.get_logger().info("🎉 ALL AGVs SUCCESSFULLY SPAWNED AND HEALTHY! 🎉")
        elif all_spawned:
            self.get_logger().info("⚠️  All AGVs spawned but some have sensor issues")
        else:
            self.get_logger().info("⏳ Waiting for remaining AGVs to spawn...")
        
        self.get_logger().info("=" * 60)

def main():
    rclpy.init()
    monitor = AGVSpawnMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()