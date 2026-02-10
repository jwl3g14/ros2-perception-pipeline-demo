#!/usr/bin/env python3
"""
Visualization Node - Combines multiple image streams into grid views.

Publishes two combined views:
  /viz/pipeline (Image) - Raw | Depth | Tracks (3-up)
  /viz/full (Image) - Raw | Depth | Tracks | Poses | Grasps (2x3 grid)

Usage:
  ros2 run perception_demo viz_node
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VizNode(Node):
    """Combines multiple image streams into grid views."""

    def __init__(self):
        super().__init__('viz_node')

        # Parameters
        self.declare_parameter('width', 320)   # Width per cell
        self.declare_parameter('height', 240)  # Height per cell

        self.cell_width = self.get_parameter('width').value
        self.cell_height = self.get_parameter('height').value

        self.bridge = CvBridge()

        # Store latest images
        self.images = {
            'raw': None,
            'depth': None,
            'tracks': None,
            'poses': None,
            'grasps': None,
        }

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw',
                                  lambda msg: self.image_callback('raw', msg), 10)
        self.create_subscription(Image, '/depth/image',
                                  lambda msg: self.image_callback('depth', msg), 10)
        self.create_subscription(Image, '/tracks/image',
                                  lambda msg: self.image_callback('tracks', msg), 10)
        self.create_subscription(Image, '/poses/image',
                                  lambda msg: self.image_callback('poses', msg), 10)
        self.create_subscription(Image, '/grasps/image',
                                  lambda msg: self.image_callback('grasps', msg), 10)

        # Publishers
        self.pipeline_pub = self.create_publisher(Image, '/viz/pipeline', 10)
        self.full_pub = self.create_publisher(Image, '/viz/full', 10)
        self.combined_pub = self.create_publisher(Image, '/viz/combined', 10)

        # Timer for publishing combined views
        self.create_timer(0.1, self.publish_views)  # 10 Hz

        self.get_logger().info('Viz node ready. Publishing to /viz/pipeline, /viz/full, /viz/combined')

    def image_callback(self, name: str, msg: Image):
        """Store latest image."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.images[name] = img
        except Exception as e:
            self.get_logger().warn(f'Failed to convert {name}: {e}')

    def resize_image(self, img: np.ndarray) -> np.ndarray:
        """Resize image to cell size."""
        if img is None:
            return self.create_placeholder('No Data')
        return cv2.resize(img, (self.cell_width, self.cell_height))

    def create_placeholder(self, text: str) -> np.ndarray:
        """Create placeholder image with text."""
        img = np.zeros((self.cell_height, self.cell_width, 3), dtype=np.uint8)
        img[:] = (40, 40, 40)  # Dark gray

        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 0.6, 1)[0]
        x = (self.cell_width - text_size[0]) // 2
        y = (self.cell_height + text_size[1]) // 2
        cv2.putText(img, text, (x, y), font, 0.6, (100, 100, 100), 1)

        return img

    def add_label(self, img: np.ndarray, label: str) -> np.ndarray:
        """Add label to top-left of image."""
        img = img.copy()
        cv2.rectangle(img, (0, 0), (len(label) * 10 + 10, 25), (0, 0, 0), -1)
        cv2.putText(img, label, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img

    def publish_views(self):
        """Publish combined views."""
        # Get resized images
        raw = self.resize_image(self.images['raw'])
        depth = self.resize_image(self.images['depth'])
        tracks = self.resize_image(self.images['tracks'])
        poses = self.resize_image(self.images['poses'])
        grasps = self.resize_image(self.images['grasps'])

        # Add labels
        raw_labeled = self.add_label(raw, 'Raw')
        depth_labeled = self.add_label(depth, 'Depth')
        tracks_labeled = self.add_label(tracks, 'Tracks')
        poses_labeled = self.add_label(poses, 'Poses')
        grasps_labeled = self.add_label(grasps, 'Grasps')

        # Pipeline view: Raw | Depth | Tracks (horizontal)
        pipeline = np.hstack([raw_labeled, depth_labeled, tracks_labeled])

        # Combined view: Tracks with Poses and Grasps overlaid
        # Use grasps as it has all the info, or tracks if grasps unavailable
        if self.images['grasps'] is not None:
            combined = self.resize_image(self.images['grasps'])
        elif self.images['poses'] is not None:
            combined = self.resize_image(self.images['poses'])
        elif self.images['tracks'] is not None:
            combined = self.resize_image(self.images['tracks'])
        else:
            combined = self.create_placeholder('Waiting...')
        combined_labeled = self.add_label(combined, 'Detection+Pose+Grasp')

        # Full view: 2x3 grid
        # Row 1: Raw | Depth | Tracks
        # Row 2: Poses | Grasps | Combined
        row1 = np.hstack([raw_labeled, depth_labeled, tracks_labeled])
        row2 = np.hstack([poses_labeled, grasps_labeled, combined_labeled])
        full = np.vstack([row1, row2])

        # Publish
        try:
            pipeline_msg = self.bridge.cv2_to_imgmsg(pipeline, encoding='bgr8')
            self.pipeline_pub.publish(pipeline_msg)

            full_msg = self.bridge.cv2_to_imgmsg(full, encoding='bgr8')
            self.full_pub.publish(full_msg)

            combined_msg = self.bridge.cv2_to_imgmsg(combined_labeled, encoding='bgr8')
            self.combined_pub.publish(combined_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
