#!/usr/bin/env python3
"""
Webcam Camera Node

Captures from USB webcam and publishes to /camera/image_raw.
Same interface as camera_sim_node - perception_node doesn't care which is running.

Usage:
  ros2 run perception_demo camera_webcam_node
  ros2 run perception_demo camera_webcam_node --ros-args -p device_id:=0 -p fps:=30
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraWebcamNode(Node):
    def __init__(self):
        super().__init__('camera_webcam_node')

        # Parameters
        self.declare_parameter('device_id', 0)  # /dev/video0
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

        # Get actual values (camera may not support requested)
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f'Webcam opened: {actual_width}x{actual_height} @ {actual_fps} FPS')

        # CV Bridge
        self.bridge = CvBridge()

        # Publisher
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # Timer for capture loop
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

        self.get_logger().info(f'Publishing to /camera/image_raw at {fps} Hz')

    def capture_and_publish(self):
        """Capture frame from webcam and publish."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.image_pub.publish(msg)

    def destroy_node(self):
        """Clean up webcam on shutdown."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraWebcamNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
