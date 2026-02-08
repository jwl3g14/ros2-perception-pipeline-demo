#!/usr/bin/env python3
"""
Simulated Camera Node

Publishes test images to /camera/image_raw for testing the detector
without needing Gazebo or a real camera.

Usage:
  ros2 run perception_demo camera_sim_node --ros-args -p image_path:=/ros_ws/data/test_images/fridge_drinks.jpg
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class CameraSimNode(Node):
    def __init__(self):
        super().__init__('camera_sim_node')

        # Parameters
        self.declare_parameter('image_path', '/ros_ws/data/test_images/fridge_drinks.jpg')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        image_path = self.get_parameter('image_path').value
        publish_rate = self.get_parameter('publish_rate').value

        # Load image
        if not os.path.exists(image_path):
            self.get_logger().error(f'Image not found: {image_path}')
            self.get_logger().info('Available files in /ros_ws/data:')
            for root, dirs, files in os.walk('/ros_ws/data'):
                for f in files:
                    self.get_logger().info(f'  {os.path.join(root, f)}')
            raise FileNotFoundError(image_path)

        self.image = cv2.imread(image_path)
        if self.image is None:
            raise ValueError(f'Failed to load image: {image_path}')

        self.get_logger().info(f'Loaded image: {image_path} ({self.image.shape})')

        # CV Bridge
        self.bridge = CvBridge()

        # Publisher
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_image)

        self.get_logger().info(f'Publishing to /camera/image_raw at {publish_rate} Hz')

    def publish_image(self):
        """Publish the test image."""
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraSimNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
