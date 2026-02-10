#!/usr/bin/env python3
"""
Visualization Node - Combines multiple image streams and data overlays.

Subscribes:
  /camera/image_raw (Image) - raw camera feed
  /depth/image (Image) - depth visualization
  /objects (Detection3DArray) - 3D object state
  /grasps (PoseArray) - grasp points

Publishes:
  /viz/grid (Image) - Raw | Depth | Combined overlay (3-up)
  /viz/overlay (Image) - Single combined view

Usage:
  ros2 run perception_demo viz_node
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from vision_msgs.msg import Detection3DArray
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
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        self.cell_width = self.get_parameter('cell_width').value
        self.cell_height = self.get_parameter('cell_height').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        self.bridge = CvBridge()

        # Store latest data
        self.raw_image = None
        self.depth_image = None
        self.objects = None     # Detection3DArray
        self.grasps = None      # PoseArray

        # Image subscribers
        self.create_subscription(Image, '/camera/image_raw',
                                  self.raw_callback, 10)
        self.create_subscription(Image, '/depth/image',
                                  self.depth_callback, 10)

        # Data subscribers
        self.create_subscription(Detection3DArray, '/objects',
                                  self.objects_callback, 10)
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

    def objects_callback(self, msg: Detection3DArray):
        self.objects = msg

    def grasps_callback(self, msg: PoseArray):
        self.grasps = msg

    def project_to_2d(self, point_3d) -> tuple:
        """Project 3D point to 2D image coordinates."""
        z = point_3d[2]
        if z <= 0:
            return (0, 0)
        u = int(point_3d[0] * self.fx / z + self.cx)
        v = int(point_3d[1] * self.fy / z + self.cy)
        return (u, v)

    def create_overlay(self) -> np.ndarray:
        """Create combined overlay: raw + objects + grasps."""
        if self.raw_image is None:
            return self.create_placeholder('Waiting for camera...')

        img = self.raw_image.copy()

        # Draw objects (from Detection3DArray)
        if self.objects is not None:
            for i, obj in enumerate(self.objects.detections):
                color = COLORS[i % len(COLORS)]
                self.draw_object(img, obj, color)

        # Draw grasps
        if self.grasps is not None:
            for i, grasp in enumerate(self.grasps.poses):
                color = COLORS[i % len(COLORS)]
                self.draw_grasp_from_pose(img, grasp, color)

        return img

    def draw_object(self, img: np.ndarray, obj, color: tuple):
        """Draw 3D object with bbox, axes, and info."""
        bbox3d = obj.bbox

        # Get 3D position and dimensions
        pos = [bbox3d.center.position.x, bbox3d.center.position.y, bbox3d.center.position.z]
        dims = [bbox3d.size.x, bbox3d.size.y, bbox3d.size.z]

        # Project center to 2D
        cx, cy = self.project_to_2d(pos)

        # Estimate 2D bbox size from 3D dimensions
        if pos[2] > 0:
            w = int(dims[0] * self.fx / pos[2])
            h = int(dims[1] * self.fy / pos[2])
        else:
            w, h = 50, 100

        x1, y1 = cx - w // 2, cy - h // 2
        x2, y2 = cx + w // 2, cy + h // 2

        # Draw bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Draw coordinate axes
        axis_len = 25
        cv2.arrowedLine(img, (cx, cy), (cx + axis_len, cy), (0, 0, 255), 2, tipLength=0.3)  # X
        cv2.arrowedLine(img, (cx, cy), (cx, cy + axis_len), (0, 255, 0), 2, tipLength=0.3)  # Y
        cv2.circle(img, (cx, cy), 4, (255, 0, 0), -1)  # Z

        # Draw label
        if obj.results:
            label = obj.results[0].hypothesis.class_id
            conf = obj.results[0].hypothesis.score
            text = f'{label} ({conf:.2f})'
            cv2.rectangle(img, (x1, y1 - 22), (x1 + len(text) * 8 + 5, y1), color, -1)
            cv2.putText(img, text, (x1 + 2, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Draw dimensions
        dim_text = f'{dims[0]*100:.0f}x{dims[1]*100:.0f}cm'
        cv2.putText(img, dim_text, (x1, y2 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # Draw 3D position
        pos_text = f'({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})'
        cv2.putText(img, pos_text, (x1, y2 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    def draw_grasp_from_pose(self, img: np.ndarray, grasp_pose, color: tuple):
        """Draw grasp point from PoseArray."""
        pos = [grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z]
        cx, cy = self.project_to_2d(pos)

        # Check if in bounds
        h, w = img.shape[:2]
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return

        # Determine grasp type from orientation
        # If quaternion is identity, it's a side grasp
        qw = grasp_pose.orientation.w
        if abs(qw - 1.0) < 0.1:
            grasp_type = 'side'
            grasp_color = (0, 255, 255)  # Yellow
        else:
            grasp_type = 'top'
            grasp_color = (255, 0, 255)  # Magenta

        # Draw grasp point
        cv2.circle(img, (cx, cy), 6, grasp_color, -1)
        cv2.circle(img, (cx, cy), 9, grasp_color, 2)

        # Draw gripper indication
        if grasp_type == 'side':
            cv2.line(img, (cx - 20, cy), (cx + 20, cy), grasp_color, 2)
            cv2.arrowedLine(img, (cx - 30, cy), (cx - 20, cy), grasp_color, 2, tipLength=0.5)
            cv2.arrowedLine(img, (cx + 30, cy), (cx + 20, cy), grasp_color, 2, tipLength=0.5)
        else:
            cv2.line(img, (cx - 15, cy), (cx + 15, cy), grasp_color, 2)
            cv2.arrowedLine(img, (cx, cy - 30), (cx, cy - 15), grasp_color, 2, tipLength=0.5)

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
