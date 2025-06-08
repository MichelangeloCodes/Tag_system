#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)  # 1 ms delay to allow image to show
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
