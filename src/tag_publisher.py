#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math

class AprilTagToDockPose(Node):
    def __init__(self):
        super().__init__('apriltag_to_dock_pose')
        
        # Publisher for dock pose
        self.dock_pose_pub = self.create_publisher(
            PoseStamped, 
            'detected_dock_pose', 
            10
        )
        
        # Subscriber to AprilTag detections
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )
        
        # Subscriber to camera info for intrinsic parameters
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )
        
        # TF buffer and listener for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera parameters (will be updated from camera_info)
        self.camera_matrix = None
        self.image_width = 320
        self.image_height = 240
        
        # Known AprilTag parameters
        self.tag_size = 0.2  # 20cm tag size (adjust based on your actual tag)
        
        self.get_logger().info("AprilTag to Dock Pose converter started")
        self.get_logger().info("Waiting for camera info and AprilTag detections...")
    
    def camera_info_callback(self, msg):
        """Update camera intrinsic parameters from camera_info topic"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(f"Camera parameters updated: {msg.width}x{msg.height}")
    
    def estimate_pose_from_corners(self, corners, camera_matrix, tag_size):
        """
        Estimate 3D pose from 2D corner detections using PnP
        This is a simplified version - for production use, consider using OpenCV's solvePnP
        """
        if len(corners) != 4:
            return None
            
        # Define 3D object points for a square tag centered at origin
        half_size = tag_size / 2.0
        object_points = np.array([
            [-half_size, -half_size, 0],  # Top-left
            [ half_size, -half_size, 0],  # Top-right
            [ half_size,  half_size, 0],  # Bottom-right
            [-half_size,  half_size, 0]   # Bottom-left
        ], dtype=np.float32)
        
        # Convert corner pixel coordinates to numpy array
        image_points = np.array([[corner.x, corner.y] for corner in corners], dtype=np.float32)
        
        # For simplified pose estimation, calculate center and approximate distance
        center_x = np.mean(image_points[:, 0])
        center_y = np.mean(image_points[:, 1])
        
        # Estimate distance based on tag size in pixels vs known real size
        pixel_size = np.linalg.norm(image_points[1] - image_points[0])  # Width in pixels
        focal_length = camera_matrix[0, 0]  # fx
        estimated_distance = (tag_size * focal_length) / pixel_size
        
        # Convert pixel coordinates to camera frame coordinates
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]  # Principal point
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]  # Focal lengths
        
        # Calculate 3D position in camera frame
        x_cam = (center_x - cx) * estimated_distance / fx
        y_cam = (center_y - cy) * estimated_distance / fy
        z_cam = estimated_distance
        
        return np.array([x_cam, y_cam, z_cam])
    
    def tag_callback(self, msg):
        if not msg.detections:
            return
            
        # Find tag with ID 0 (our dock tag)
        dock_tag = None
        for detection in msg.detections:
            if detection.id == 0:
                dock_tag = detection
                break
        
        if dock_tag is None:
            return
        
        # Log the 2D detection information
        self.get_logger().info(f"Detected AprilTag {dock_tag.id} at pixel location: "
                             f"({dock_tag.centre.x:.1f}, {dock_tag.centre.y:.1f})")
        
        try:
            if self.camera_matrix is None:
                self.get_logger().warn("No camera parameters available yet")
                return
            
            # Method 1: Use corner detection if available
            if hasattr(dock_tag, 'corners') and len(dock_tag.corners) == 4:
                # Estimate 3D pose from corners
                tag_position_cam = self.estimate_pose_from_corners(
                    dock_tag.corners, self.camera_matrix, self.tag_size
                )
                
                if tag_position_cam is not None:
                    # Create pose in camera frame
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = self.get_clock().now()
                    pose_stamped.header.frame_id = 'camera_optical_link'
                    
                    pose_stamped.pose.position.x = float(tag_position_cam[0])
                    pose_stamped.pose.position.y = float(tag_position_cam[1])
                    pose_stamped.pose.position.z = float(tag_position_cam[2])
                    pose_stamped.pose.orientation.w = 1.0  # Identity quaternion
                    
                    # Transform to base_link frame
                    try:
                        transformed_pose = self.tf_buffer.transform(
                            pose_stamped, 
                            'base_link', 
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        
                        # Publish the dock pose
                        self.dock_pose_pub.publish(transformed_pose)
                        self.get_logger().info(f"Published dock pose: x={transformed_pose.pose.position.x:.2f}, "
                                             f"y={transformed_pose.pose.position.y:.2f}, "
                                             f"z={transformed_pose.pose.position.z:.2f}")
                        
                    except Exception as tf_e:
                        self.get_logger().warn(f"Could not transform to base_link: {tf_e}")
                        # Publish in camera frame as fallback
                        self.dock_pose_pub.publish(pose_stamped)
                        
            else:
                # Method 2: Simplified approach using center point only
                # Estimate distance based on image size and known tag characteristics
                self.get_logger().warn("No corner information available, using simplified estimation")
                
                # Simple distance estimation based on typical AprilTag detection
                # This is a rough approximation - adjust based on your setup
                estimated_distance = 2.0  # meters - rough estimate
                
                # Calculate approximate 3D position from center pixel
                cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
                fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
                
                x_cam = (dock_tag.centre.x - cx) * estimated_distance / fx
                y_cam = (dock_tag.centre.y - cy) * estimated_distance / fy
                z_cam = estimated_distance
                
                # Create pose in camera frame
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now()
                pose_stamped.header.frame_id = 'camera_optical_link'
                
                pose_stamped.pose.position.x = float(x_cam)
                pose_stamped.pose.position.y = float(y_cam)
                pose_stamped.pose.position.z = float(z_cam)
                pose_stamped.pose.orientation.w = 1.0
                
                # Try to transform to base_link
                try:
                    transformed_pose = self.tf_buffer.transform(
                        pose_stamped, 
                        'base_link', 
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    self.dock_pose_pub.publish(transformed_pose)
                    self.get_logger().info(f"Published approximate dock pose in base_link frame")
                except Exception as tf_e:
                    self.get_logger().warn(f"Could not transform to base_link: {tf_e}")
                    self.dock_pose_pub.publish(pose_stamped)
                    self.get_logger().info(f"Published approximate dock pose in camera frame")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing AprilTag detection: {e}")

def main():
    rclpy.init()
    node = AprilTagToDockPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()