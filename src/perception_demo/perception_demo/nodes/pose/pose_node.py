#!/usr/bin/env python3
"""
6DoF Pose Estimation Node

Combines 2D detections with depth to estimate 3D pose of objects.

Subscribes:
  /tracks (Detection2DArray) - tracked objects
  /depth/image (Image) - depth map from MiDaS

Publishes:
  /poses (PoseArray) - 3D poses of detected objects
  /poses/image (Image) - visualization with pose axes

Usage:
  ros2 run perception_demo pose_node
  ros2 run perception_demo pose_node --ros-args -p detection_topic:=/detections
"""

import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


@dataclass
class Pose6DoF:
    """6DoF pose with metadata."""
    position: np.ndarray      # [x, y, z] in meters
    quaternion: np.ndarray    # [w, x, y, z]
    class_name: str
    confidence: float


class PoseEstimatorNode(Node):
    """ROS2 node for 6DoF pose estimation."""

    def __init__(self):
        super().__init__('pose_node')

        # Parameters
        self.declare_parameter('detection_topic', '/tracks')
        self.declare_parameter('depth_topic', '/depth/image')
        self.declare_parameter('fx', 600.0)  # Camera focal length x
        self.declare_parameter('fy', 600.0)  # Camera focal length y
        self.declare_parameter('cx', 320.0)  # Principal point x
        self.declare_parameter('cy', 240.0)  # Principal point y
        self.declare_parameter('depth_scale', 1.0)  # Depth scaling factor

        detection_topic = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.depth_scale = self.get_parameter('depth_scale').value

        # CV Bridge
        self.bridge = CvBridge()

        # Latest data (for async processing)
        self.latest_depth = None
        self.latest_detections = None

        # Subscribers with message synchronization
        self.detection_sub = Subscriber(self, Detection2DArray, detection_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)

        # Synchronize detection and depth messages
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub],
            queue_size=10,
            slop=0.5  # Allow 500ms difference
        )
        self.sync.registerCallback(self.synced_callback)

        # Publishers
        self.pose_pub = self.create_publisher(PoseArray, '/poses', 10)
        self.image_pub = self.create_publisher(Image, '/poses/image', 10)

        # Also subscribe to raw image for visualization
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.latest_image = None

        self.get_logger().info(f'Pose node ready. Listening to {detection_topic} + {depth_topic}')

    def image_callback(self, msg: Image):
        """Store latest image for visualization."""
        self.latest_image = msg

    def synced_callback(self, detection_msg: Detection2DArray, depth_msg: Image):
        """Process synchronized detection and depth messages."""
        # Convert depth image
        depth_map = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Handle 3-channel depth (visualization) - convert to single channel
        if len(depth_map.shape) == 3:
            depth_map = cv2.cvtColor(depth_map, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # Normalize depth if needed (MiDaS outputs inverse depth)
        if depth_map.dtype == np.float32 or depth_map.max() > 1:
            # Normalize to 0-1 range
            depth_map = depth_map / (depth_map.max() + 1e-6)

        # Estimate poses for each detection
        poses = []
        for detection in detection_msg.detections:
            pose = self.estimate_pose(detection, depth_map)
            if pose is not None:
                poses.append(pose)

        # Publish PoseArray
        pose_array = PoseArray()
        pose_array.header = detection_msg.header
        pose_array.header.frame_id = 'camera_optical_frame'

        for p in poses:
            ros_pose = Pose()
            ros_pose.position = Point(x=float(p.position[0]),
                                       y=float(p.position[1]),
                                       z=float(p.position[2]))
            ros_pose.orientation = Quaternion(w=float(p.quaternion[0]),
                                               x=float(p.quaternion[1]),
                                               y=float(p.quaternion[2]),
                                               z=float(p.quaternion[3]))
            pose_array.poses.append(ros_pose)

        self.pose_pub.publish(pose_array)

        # Publish visualization
        if self.latest_image is not None:
            self.publish_visualization(poses, detection_msg.detections)

        # Log
        if len(poses) > 0:
            pose_strs = [f'{p.class_name}@({p.position[0]:.2f},{p.position[1]:.2f},{p.position[2]:.2f})'
                         for p in poses]
            self.get_logger().info(f'Poses: {", ".join(pose_strs)}')

    def estimate_pose(self, detection, depth_map: np.ndarray) -> Optional[Pose6DoF]:
        """Estimate 6DoF pose from detection and depth."""
        # Extract bbox
        bbox = detection.bbox
        cx = bbox.center.position.x
        cy = bbox.center.position.y
        w = bbox.size_x
        h = bbox.size_y

        x1 = int(cx - w/2)
        y1 = int(cy - h/2)
        x2 = int(cx + w/2)
        y2 = int(cy + h/2)

        # Get depth at center (weighted)
        depth = self.get_depth_at_bbox(depth_map, [x1, y1, x2, y2])
        if depth <= 0:
            return None

        # Convert 2D center to 3D position
        position = self.pixel_to_3d(cx, cy, depth)

        # Estimate orientation (assume vertical for bottles/cans)
        quaternion = self.estimate_vertical_orientation()

        # Get class name and confidence
        class_name = "unknown"
        confidence = 0.0
        if detection.results:
            hypothesis = detection.results[0].hypothesis
            class_name = hypothesis.class_id
            confidence = hypothesis.score

        return Pose6DoF(
            position=position,
            quaternion=quaternion,
            class_name=class_name,
            confidence=confidence
        )

    def get_depth_at_bbox(self, depth_map: np.ndarray, bbox: List[int]) -> float:
        """Get center-weighted depth in bounding box."""
        x1, y1, x2, y2 = bbox
        h, w = depth_map.shape[:2]

        # Clamp
        x1 = max(0, min(x1, w-1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h-1))
        y2 = max(0, min(y2, h))

        if x1 >= x2 or y1 >= y2:
            return 0.0

        region = depth_map[y1:y2, x1:x2]

        # Center-weighted average
        rh, rw = region.shape[:2]
        if rh == 0 or rw == 0:
            return 0.0

        cy_r, cx_r = rh // 2, rw // 2
        y_weights = np.exp(-((np.arange(rh) - cy_r) ** 2) / max(1, (rh / 2) ** 2))
        x_weights = np.exp(-((np.arange(rw) - cx_r) ** 2) / max(1, (rw / 2) ** 2))
        weights = np.outer(y_weights, x_weights)

        weighted_sum = np.sum(region * weights)
        weight_sum = np.sum(weights)

        return weighted_sum / weight_sum if weight_sum > 0 else 0.0

    def pixel_to_3d(self, u: float, v: float, depth: float) -> np.ndarray:
        """Convert pixel coordinates + depth to 3D point."""
        z = depth * self.depth_scale
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z])

    def estimate_vertical_orientation(self) -> np.ndarray:
        """Return quaternion for vertical object (no rotation)."""
        # Identity quaternion - object is upright
        return np.array([1.0, 0.0, 0.0, 0.0])

    def publish_visualization(self, poses: List[Pose6DoF], detections):
        """Publish image with pose visualization."""
        if self.latest_image is None:
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

        for pose, det in zip(poses, detections):
            # Get 2D center
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)

            # Draw coordinate axes
            axis_len = 30

            # X axis (red)
            cv2.arrowedLine(img, (cx, cy), (cx + axis_len, cy), (0, 0, 255), 2, tipLength=0.2)
            # Y axis (green)
            cv2.arrowedLine(img, (cx, cy), (cx, cy + axis_len), (0, 255, 0), 2, tipLength=0.2)
            # Z axis (blue) - circle for out of screen
            cv2.circle(img, (cx, cy), 4, (255, 0, 0), -1)
            cv2.circle(img, (cx, cy), 7, (255, 0, 0), 2)

            # Position text
            pos_text = f'({pose.position[0]:.2f}, {pose.position[1]:.2f}, {pose.position[2]:.2f})'
            cv2.putText(img, pos_text, (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # Class name
            cv2.putText(img, pose.class_name, (cx + 10, cy + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Publish
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header = self.latest_image.header
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
