#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
        
        # Topics to check
        self.topics_to_check = [
            '/map',
            '/agv1/scan', 
            '/agv2/scan',
            '/tf',
            '/tf_static'
        ]
        
        self.topic_status = {}
        self.subscribers = []
        
        # QoS profiles for different topic types
        # Map typically uses transient local for reliability
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Scan topics typically use best effort for performance
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create subscribers for each topic with appropriate QoS
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(LaserScan, '/agv1/scan', self.agv1_scan_callback, scan_qos)
        self.create_subscription(LaserScan, '/agv2/scan', self.agv2_scan_callback, scan_qos)
        
        # Timer to print status
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info("Topic checker started. Monitoring required topics for AMCL...")
        
    def map_callback(self, msg):
        self.topic_status['/map'] = True
        
    def agv1_scan_callback(self, msg):
        self.topic_status['/agv1/scan'] = True
        
    def agv2_scan_callback(self, msg):
        self.topic_status['/agv2/scan'] = True
        
    def print_status(self):
        self.get_logger().info("=== TOPIC STATUS ===")
        for topic in self.topics_to_check:
            if topic in ['/tf', '/tf_static']:
                # For TF topics, just check if they exist
                status = "Available (not checked)"
            else:
                status = "✓ Receiving" if self.topic_status.get(topic, False) else "✗ Missing"
            self.get_logger().info(f"{topic}: {status}")
        self.get_logger().info("==================")

def main(args=None):
    rclpy.init(args=args)
    node = TopicChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()