#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import DockRobot
from geometry_msgs.msg import PoseStamped


class DockRobotActionClient(Node):
    def __init__(self):
        super().__init__('dock_robot_client')
        self._action_client = ActionClient(self, DockRobot, '/dock_robot')
        self.get_logger().info('DockRobot Action Client initialized')

    def send_goal(self, dock_id='', dock_type='', dock_pose=None, navigate_to_staging_pose=True):
        """Send docking goal to the action server"""
        goal_msg = DockRobot.Goal()
        goal_msg.dock_id = dock_id
        goal_msg.dock_type = dock_type
        goal_msg.navigate_to_staging_pose = navigate_to_staging_pose
        
        if dock_pose is not None:
            goal_msg.dock_pose = dock_pose
        else:
            goal_msg.dock_pose = PoseStamped()
            goal_msg.dock_pose.header.frame_id = 'map'
            goal_msg.dock_pose.header.stamp = self.get_clock().now().to_msg()
        
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return None
        
        self.get_logger().info(f'Sending docking goal: {dock_id} ({dock_type})')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Docking completed: {result.success}')
        elif status == 5:  # ABORTED
            self.get_logger().error(f'Docking aborted: {result.success}')
        else:
            self.get_logger().warn(f'Docking finished with status: {status}')

    def feedback_callback(self, feedback_msg):
        """Handle action feedback"""
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'current_state'):
            self.get_logger().info(f'State: {feedback.current_state}')
        if hasattr(feedback, 'dock_distance'):
            self.get_logger().info(f'Distance: {feedback.dock_distance:.2f}m')

    def create_dock_pose(self, x, y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        
        return pose


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    action_client = DockRobotActionClient()
    
    try:
        # Send docking goal
        future = action_client.send_goal(
            dock_id="home_dock",
            dock_type="simple_charging_dock",
            navigate_to_staging_pose=True
        )
        
        if future is not None:
            # Spin until goal completes
            rclpy.spin_until_future_complete(action_client, future)
            
            # Give some time for result processing by spinning the node
            import time
            start_time = time.time()
            while time.time() - start_time < 2.0:
                rclpy.spin_once(action_client, timeout_sec=0.1)
        else:
            action_client.get_logger().error('Failed to send goal')
        
    except KeyboardInterrupt:
        action_client.get_logger().info('Shutting down...')
    
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()