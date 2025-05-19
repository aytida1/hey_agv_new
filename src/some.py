#!/usr/bin/env python3
# filepath: /home/meditab/car_ws_new/src/hey_agv_new/src/some.py

import rclpy
from rclpy.node import Node
import numpy as np
import math
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_matrix
import tf2_ros
import geometry_msgs.msg

class LidarFusion(Node):
    def __init__(self):
        super().__init__('lidar_fusion_node')
        
        # Parameters
        self.max_range = 12.0  # Max range of individual LiDARs
        self.angle_min = -math.pi  # Full 360 view for combined scan
        self.angle_max = math.pi
        self.angle_increment = 0.01  # Adjust based on your desired resolution
        self.num_points = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher for combined scan
        self.combined_scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Subscribers for individual LiDAR scans
        self.scan1_sub = self.create_subscription(LaserScan, '/scan1', self.scan1_callback, 10)
        self.scan2_sub = self.create_subscription(LaserScan, '/scan2', self.scan2_callback, 10)
        
        # Store the latest scans
        self.latest_scan1 = None
        self.latest_scan2 = None
        
        # Timer for publishing combined scan
        self.timer = self.create_timer(0.1, self.publish_combined_scan)
    
    def scan1_callback(self, scan_msg):
        self.latest_scan1 = scan_msg
    
    def scan2_callback(self, scan_msg):
        self.latest_scan2 = scan_msg
    
    def laser_scan_to_points(self, scan_msg, lidar_frame):
        """Convert LaserScan to point cloud and transform to base_link"""
        points = []
        
        try:
            # Get transform from lidar frame to base_link
            # Fix: Use proper time handling for ROS2
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                lidar_frame,
                now,
                tf2_ros.Duration(seconds=0.1)
            )
            
            # Extract translation and rotation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            
            q = transform.transform.rotation
            rot_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            
            # Convert scan to points in lidar frame, then transform to base_link
            for i, r in enumerate(scan_msg.ranges):
                if r < scan_msg.range_min or r > scan_msg.range_max or not np.isfinite(r):
                    continue
                
                # Calculate point in lidar frame
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x_lidar = r * math.cos(angle)
                y_lidar = r * math.sin(angle)
                z_lidar = 0.0
                
                # Transform to base_link frame
                p_lidar = np.array([x_lidar, y_lidar, z_lidar])
                p_base = rot_matrix.dot(p_lidar) + np.array([tx, ty, tz])
                
                # Store point and its polar angle (for later sorting)
                angle_in_base = math.atan2(p_base[1], p_base[0])
                distance = math.sqrt(p_base[0]**2 + p_base[1]**2)
                
                points.append((angle_in_base, distance))
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF error: {e}")
            
        return points
  
    def publish_combined_scan(self):
        if self.latest_scan1 is None or self.latest_scan2 is None:
            return
        
        # Get points from both LiDARs
        points1 = self.laser_scan_to_points(self.latest_scan1, 'agv/base_link/gpu_lidar_right')
        points2 = self.laser_scan_to_points(self.latest_scan2, 'agv/base_link/gpu_lidar_left')
        
        # Combine points
        all_points = points1 + points2
        
        # Create new LaserScan message
        combined_scan = LaserScan()
        combined_scan.header.stamp = self.latest_scan1.header.stamp  # Use the timestamp of the first scan
        combined_scan.header.frame_id = 'base_link'
        combined_scan.angle_min = self.angle_min
        combined_scan.angle_max = self.angle_max
        combined_scan.angle_increment = self.angle_increment
        combined_scan.time_increment = 0.0
        combined_scan.scan_time = 0.1
        combined_scan.range_min = 0.1  # Adjust as needed
        combined_scan.range_max = 26.0  # Large enough for combined range
        
        # Initialize ranges array with inf
        combined_scan.ranges = [float('inf')] * self.num_points
        
        # Fill in ranges based on combined points
        for angle, distance in all_points:
            # Find the index in the combined scan array
            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < len(combined_scan.ranges):
                # If multiple points map to same index, take the closest one
                combined_scan.ranges[idx] = min(combined_scan.ranges[idx], distance)
        
        # Publish the combined scan
        self.combined_scan_pub.publish(combined_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()