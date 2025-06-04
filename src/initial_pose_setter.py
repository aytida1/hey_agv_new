#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Publishers for initial poses
        self.agv1_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/agv1/initialpose', 
            10
        )
        self.agv2_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/agv2/initialpose', 
            10
        )
        
        # Wait a bit for publishers to be ready
        self.timer = self.create_timer(3.0, self.set_initial_poses)
        self.poses_set = False
        
        self.get_logger().info("Initial pose setter started. Will set poses in 3 seconds...")
        
    def set_initial_poses(self):
        if self.poses_set:
            return
            
        # Create pose message for AGV1
        agv1_pose = PoseWithCovarianceStamped()
        agv1_pose.header.stamp = self.get_clock().now().to_msg()
        agv1_pose.header.frame_id = 'map'
        agv1_pose.pose.pose.position.x = -3.87
        agv1_pose.pose.pose.position.y = -15.81
        agv1_pose.pose.pose.position.z = 0.0
        agv1_pose.pose.pose.orientation.x = 0.0
        agv1_pose.pose.pose.orientation.y = 0.0
        agv1_pose.pose.pose.orientation.z = 0.0
        agv1_pose.pose.pose.orientation.w = 1.0
        
        # Set covariance (diagonal matrix)
        agv1_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068
        ]
        
        # Create pose message for AGV2 (different position)
        agv2_pose = PoseWithCovarianceStamped()
        agv2_pose.header.stamp = self.get_clock().now().to_msg()
        agv2_pose.header.frame_id = 'map'
        agv2_pose.pose.pose.position.x = 0.0  # Different position
        agv2_pose.pose.pose.position.y = 0.0
        agv2_pose.pose.pose.position.z = 0.0
        agv2_pose.pose.pose.orientation.x = 0.0
        agv2_pose.pose.pose.orientation.y = 0.0
        agv2_pose.pose.pose.orientation.z = 0.0
        agv2_pose.pose.pose.orientation.w = 1.0
        agv2_pose.pose.covariance = agv1_pose.pose.covariance
        
        # Publish initial poses
        self.agv1_pose_pub.publish(agv1_pose)
        self.agv2_pose_pub.publish(agv2_pose)
        
        self.get_logger().info("Initial poses set for both AGVs!")
        self.get_logger().info("AGV1: (0.0, 0.0, 0.0)")
        self.get_logger().info("AGV2: (5.0, 0.0, 0.0)")
        
        self.poses_set = True
        
        # Stop the timer
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()