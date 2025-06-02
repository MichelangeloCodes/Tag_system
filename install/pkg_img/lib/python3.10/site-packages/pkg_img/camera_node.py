#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Use your actual camera device path here (symlink under /dev/by-id)
        self.device_path = '/dev/video0'

        if not os.path.exists(self.device_path):
            self.get_logger().error(f"Device not found: {self.device_path}")
            rclpy.shutdown()
            return

        self.cap = cv2.VideoCapture(self.device_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at {self.device_path}")
            rclpy.shutdown()
            return

        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        # Publish images every second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info("Published image frame")
        else:
            self.get_logger().warn("Failed to capture image")

    def destroy_node(self):
        self.get_logger().info("Releasing camera and shutting down node")
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
