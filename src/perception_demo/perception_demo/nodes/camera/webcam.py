#!/usr/bin/env python3
"""
Webcam Camera - Captures from USB webcam.

Usage:
  ros2 run perception_demo camera_node --ros-args -p source:=webcam
  ros2 run perception_demo camera_node --ros-args -p source:=webcam -p device_id:=0 -p fps:=30
"""

import cv2
from .base import BaseCameraNode


class WebcamCameraNode(BaseCameraNode):
    """Camera node that captures from USB webcam."""

    def __init__(self):
        from rclpy.node import Node
        Node.__init__(self, 'camera_node')

        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        device_id = self.get_parameter('device_id').value
        fps = self.get_parameter('fps').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value

        # Open webcam
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open webcam /dev/video{device_id}')
            raise RuntimeError(f'Cannot open webcam {device_id}')

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Get actual values
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f'Webcam opened: {actual_width}x{actual_height} @ {actual_fps} FPS')

        # Initialize base class components
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.publish_rate = fps

        self.get_logger().info(f'Publishing to /camera/image_raw at {fps} Hz (source: webcam)')

    def get_frame(self):
        """Capture frame from webcam."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return None
        return frame

    def destroy_node(self):
        """Clean up webcam on shutdown."""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()
