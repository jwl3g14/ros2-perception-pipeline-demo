#!/usr/bin/env python3
"""
Base Camera Node - Shared logic for all camera sources.
"""

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BaseCameraNode(Node):
    """Base class for camera nodes. Subclasses implement get_frame()."""

    def __init__(self, node_name: str, publish_rate: float = 30.0):
        super().__init__(node_name)

        self.bridge = CvBridge()

        # Publisher
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_frame)
        self.publish_rate = publish_rate

    def get_frame(self):
        """
        Get the next frame. Subclasses must implement this.
        Returns: numpy array (BGR format) or None if no frame available.
        """
        raise NotImplementedError("Subclasses must implement get_frame()")

    def publish_frame(self):
        """Capture frame and publish to ROS topic."""
        frame = self.get_frame()
        if frame is None:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.image_pub.publish(msg)
