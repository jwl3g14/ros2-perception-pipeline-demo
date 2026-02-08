#!/usr/bin/env python3
"""
Simulated Camera - Publishes static test images.

Usage:
  ros2 run perception_demo camera_node --ros-args -p source:=sim
  ros2 run perception_demo camera_node --ros-args -p source:=sim -p image_path:=/path/to/image.jpg
"""

import os
import cv2
from .base import BaseCameraNode


class SimCameraNode(BaseCameraNode):
    """Camera node that publishes static test images."""

    def __init__(self):
        # Declare parameters before calling super().__init__
        # (we need publish_rate for the base class)
        from rclpy.node import Node
        Node.__init__(self, 'camera_node')

        self.declare_parameter('image_path', '/ros_ws/data/test_images/fridge_drinks.jpg')
        self.declare_parameter('publish_rate', 1.0)

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

        # Initialize base class components
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_frame)
        self.publish_rate = publish_rate

        self.get_logger().info(f'Publishing to /camera/image_raw at {publish_rate} Hz (source: sim)')

    def get_frame(self):
        """Return the static test image."""
        return self.image
