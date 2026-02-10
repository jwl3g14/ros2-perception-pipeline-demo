#!/usr/bin/env python3
"""
Visualization Node - Combines multiple image streams and data overlays.

Publishes:
  /viz/grid (Image) - Raw | Depth | Combined overlay (3-up)
  /viz/overlay (Image) - Single combined view: raw + tracks + pose + grasp

Usage:
  ros2 run perception_demo viz_node
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


# Colors (BGR)
COLORS = [
    (255, 0, 0),    # Blue
    (0, 255, 0),    # Green
    (0, 0, 255),    # Red
    (255, 255, 0),  # Cyan
    (255, 0, 255),  # Magenta
    (0, 255, 255),  # Yellow
]


class VizNode(Node):
    """Combines multiple image streams and draws overlays."""

    def __init__(self):
        super().__init__('viz_node')

        # Parameters
        self.declare_parameter('cell_width', 320)
        self.declare_parameter('cell_height', 240)

        self.cell_width = self.get_parameter('cell_width').value
        self.cell_height = self.get_parameter('cell_height').value

        self.bridge = CvBridge()

        # Store latest data
        self.raw_image = None
        self.depth_image = None
        self.detections = None  # Detection2DArray
        self.poses = None       # PoseArray
        self.grasps = None      # PoseArray

        # Image subscribers
        self.create_subscription(Image, '/camera/image_raw',
                                  self.raw_callback, 10)
        self.create_subscription(Image, '/depth/image',
                                  self.depth_callback, 10)

        # Data subscribers (for overlay drawing)
        self.create_subscription(Detection2DArray, '/tracks',
                                  self.detection_callback, 10)
        self.create_subscription(PoseArray, '/poses',
                                  self.poses_callback, 10)
        self.create_subscription(PoseArray, '/grasps',
                                  self.grasps_callback, 10)

        # Publishers
        self.grid_pub = self.create_publisher(Image, '/viz/grid', 10)
        self.overlay_pub = self.create_publisher(Image, '/viz/overlay', 10)

        # Timer for publishing
        self.create_timer(0.1, self.publish_views)  # 10 Hz

        self.get_logger().info('Viz node ready. Publishing to /viz/grid and /viz/overlay')

    def raw_callback(self, msg: Image):
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Raw image error: {e}')

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Depth image error: {e}')

    def detection_callback(self, msg: Detection2DArray):
        self.detections = msg

    def poses_callback(self, msg: PoseArray):
        self.poses = msg

    def grasps_callback(self, msg: PoseArray):
        self.grasps = msg

    def create_overlay(self) -> np.ndarray:
        """Create combined overlay: raw + detections + poses + grasps."""
        if self.raw_image is None:
            return self.create_placeholder('Waiting for camera...')

        img = self.raw_image.copy()

        # Draw detections (bounding boxes + labels)
        if self.detections is not None:
            for i, det in enumerate(self.detections.detections):
                color = COLORS[i % len(COLORS)]
                self.draw_detection(img, det, color, i)

        # Draw poses (coordinate axes)
        if self.detections is not None and self.poses is not None:
            for i, (det, pose) in enumerate(zip(self.detections.detections, self.poses.poses)):
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                self.draw_pose_axes(img, cx, cy)

        # Draw grasps (grasp points)
        if self.detections is not None and self.grasps is not None:
            for i, (det, grasp) in enumerate(zip(self.detections.detections, self.grasps.poses)):
                color = COLORS[i % len(COLORS)]
                self.draw_grasp(img, det, color)

        return img

    def draw_detection(self, img: np.ndarray, det, color: tuple, idx: int):
        """Draw bounding box and label."""
        bbox = det.bbox
        cx, cy = int(bbox.center.position.x), int(bbox.center.position.y)
        w, h = int(bbox.size_x), int(bbox.size_y)
        x1, y1 = cx - w // 2, cy - h // 2
        x2, y2 = cx + w // 2, cy + h // 2

        # Bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Label
        if det.results:
            label = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            text = f'{label} ({conf:.2f})'
            cv2.rectangle(img, (x1, y1 - 22), (x1 + len(text) * 8 + 5, y1), color, -1)
            cv2.putText(img, text, (x1 + 2, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_pose_axes(self, img: np.ndarray, cx: int, cy: int, length: int = 25):
        """Draw coordinate axes at center."""
        # X axis (red)
        cv2.arrowedLine(img, (cx, cy), (cx + length, cy), (0, 0, 255), 2, tipLength=0.3)
        # Y axis (green)
        cv2.arrowedLine(img, (cx, cy), (cx, cy + length), (0, 255, 0), 2, tipLength=0.3)
        # Z axis (blue circle - coming out of screen)
        cv2.circle(img, (cx, cy), 4, (255, 0, 0), -1)

    def draw_grasp(self, img: np.ndarray, det, color: tuple):
        """Draw grasp point indicator."""
        bbox = det.bbox
        cx = int(bbox.center.position.x)
        h = int(bbox.size_y)
        y1 = int(bbox.center.position.y - h // 2)

        # Grasp point at 40% from top (side grasp position)
        grasp_y = int(y1 + h * 0.4)

        # Grasp indicator (gripper)
        cv2.circle(img, (cx, grasp_y), 6, (0, 255, 255), -1)
        cv2.circle(img, (cx, grasp_y), 9, (0, 255, 255), 2)

        # Gripper fingers
        cv2.line(img, (cx - 20, grasp_y), (cx + 20, grasp_y), (0, 255, 255), 2)
        cv2.arrowedLine(img, (cx - 30, grasp_y), (cx - 20, grasp_y), (0, 255, 255), 2, tipLength=0.5)
        cv2.arrowedLine(img, (cx + 30, grasp_y), (cx + 20, grasp_y), (0, 255, 255), 2, tipLength=0.5)

    def resize_image(self, img: np.ndarray) -> np.ndarray:
        """Resize image to cell size."""
        if img is None:
            return self.create_placeholder('No Data')
        return cv2.resize(img, (self.cell_width, self.cell_height))

    def create_placeholder(self, text: str) -> np.ndarray:
        """Create placeholder image."""
        img = np.zeros((self.cell_height, self.cell_width, 3), dtype=np.uint8)
        img[:] = (40, 40, 40)
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 0.5, 1)[0]
        x = (self.cell_width - text_size[0]) // 2
        y = (self.cell_height + text_size[1]) // 2
        cv2.putText(img, text, (x, y), font, 0.5, (100, 100, 100), 1)
        return img

    def add_label(self, img: np.ndarray, label: str) -> np.ndarray:
        """Add label to top-left."""
        img = img.copy()
        cv2.rectangle(img, (0, 0), (len(label) * 9 + 8, 22), (0, 0, 0), -1)
        cv2.putText(img, label, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img

    def publish_views(self):
        """Publish combined views."""
        # Create overlay
        overlay = self.create_overlay()

        # Resize for grid
        raw_small = self.resize_image(self.raw_image)
        depth_small = self.resize_image(self.depth_image)
        overlay_small = self.resize_image(overlay)

        # Add labels
        raw_labeled = self.add_label(raw_small, 'Raw')
        depth_labeled = self.add_label(depth_small, 'Depth')
        overlay_labeled = self.add_label(overlay_small, 'Combined')

        # Grid: Raw | Depth | Combined
        grid = np.hstack([raw_labeled, depth_labeled, overlay_labeled])

        # Publish
        try:
            grid_msg = self.bridge.cv2_to_imgmsg(grid, encoding='bgr8')
            self.grid_pub.publish(grid_msg)

            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            self.overlay_pub.publish(overlay_msg)
        except Exception as e:
            self.get_logger().warn(f'Publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
