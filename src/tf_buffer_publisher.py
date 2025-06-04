#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
import threading

class TFBufferPublisher(Node):
    def __init__(self):
        super().__init__('tf_buffer_publisher')
        self.tf_buffer = []
        self.lock = threading.Lock()
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_callback, 10)
        self.pub = self.create_publisher(TFMessage, '/all_transforms', 10)
        self.timer = self.create_timer(0.2, self.publish_buffer)  # 5 Hz

    def tf_callback(self, msg):
        with self.lock:
            self.tf_buffer.extend(msg.transforms)
            # Optionally, keep only the latest N transforms or clear after publish

    def publish_buffer(self):
        with self.lock:
            if self.tf_buffer:
                msg = TFMessage()
                msg.transforms = list(self.tf_buffer)
                self.pub.publish(msg)
                self.tf_buffer.clear()  # Clear after publishing


def main(args=None):
    rclpy.init(args=args)
    node = TFBufferPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
