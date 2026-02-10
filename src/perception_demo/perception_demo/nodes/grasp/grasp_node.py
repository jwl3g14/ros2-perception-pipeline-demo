#!/usr/bin/env python3
"""
Grasp Point Prediction Node

Predicts optimal grasp points for detected objects.

Subscribes:
  /tracks (Detection2DArray) - tracked objects
  /depth/image (Image) - depth map

Publishes:
  /grasps (PoseArray) - grasp poses (position + approach direction)
  /grasps/image (Image) - visualization with grasp points

Usage:
  ros2 run perception_demo grasp_node
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
class GraspPoint:
    """Grasp point with metadata."""
    position_3d: np.ndarray   # [x, y, z] in meters
    position_2d: np.ndarray   # [u, v] in pixels
    approach_vector: np.ndarray
    grasp_width: float
    confidence: float
    grasp_type: str           # 'side' or 'top'
    class_name: str


class GraspPredictorNode(Node):
    """ROS2 node for grasp point prediction."""

    def __init__(self):
        super().__init__('grasp_node')

        # Parameters
        self.declare_parameter('detection_topic', '/tracks')
        self.declare_parameter('depth_topic', '/depth/image')
        self.declare_parameter('gripper_max_width', 0.1)  # 10cm
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        detection_topic = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.gripper_max_width = self.get_parameter('gripper_max_width').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers with synchronization
        self.detection_sub = Subscriber(self, Detection2DArray, detection_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub],
            queue_size=10,
            slop=0.5
        )
        self.sync.registerCallback(self.synced_callback)

        # Publishers
        self.grasp_pub = self.create_publisher(PoseArray, '/grasps', 10)
        self.image_pub = self.create_publisher(Image, '/grasps/image', 10)

        # Raw image for visualization
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.latest_image = None

        self.get_logger().info(f'Grasp node ready. Listening to {detection_topic} + {depth_topic}')

    def image_callback(self, msg: Image):
        """Store latest image for visualization."""
        self.latest_image = msg

    def synced_callback(self, detection_msg: Detection2DArray, depth_msg: Image):
        """Process synchronized messages."""
        # Convert depth
        depth_map = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Handle 3-channel depth (visualization) - convert to single channel
        if len(depth_map.shape) == 3:
            depth_map = cv2.cvtColor(depth_map, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # Normalize if needed
        if depth_map.dtype == np.float32 or depth_map.max() > 1:
            depth_map = depth_map / (depth_map.max() + 1e-6)

        # Predict grasps
        grasps = []
        for detection in detection_msg.detections:
            grasp = self.predict_grasp(detection, depth_map)
            if grasp is not None:
                grasps.append(grasp)

        # Publish PoseArray
        pose_array = PoseArray()
        pose_array.header = detection_msg.header
        pose_array.header.frame_id = 'camera_optical_frame'

        for g in grasps:
            ros_pose = Pose()
            ros_pose.position = Point(x=float(g.position_3d[0]),
                                       y=float(g.position_3d[1]),
                                       z=float(g.position_3d[2]))
            # Encode approach direction as quaternion
            quat = self.approach_to_quaternion(g.approach_vector)
            ros_pose.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
            pose_array.poses.append(ros_pose)

        self.grasp_pub.publish(pose_array)

        # Publish visualization
        if self.latest_image is not None:
            self.publish_visualization(grasps)

        # Log
        if len(grasps) > 0:
            grasp_strs = [f'{g.class_name}:{g.grasp_type}({g.confidence:.2f})' for g in grasps]
            self.get_logger().info(f'Grasps: {", ".join(grasp_strs)}')

    def predict_grasp(self, detection, depth_map: np.ndarray) -> Optional[GraspPoint]:
        """Predict grasp point for detection."""
        bbox = detection.bbox
        cx = bbox.center.position.x
        cy = bbox.center.position.y
        w = bbox.size_x
        h = bbox.size_y

        x1 = int(cx - w/2)
        y1 = int(cy - h/2)
        x2 = int(cx + w/2)
        y2 = int(cy + h/2)

        # Estimate object size
        obj_width, obj_height = self.estimate_object_size([x1, y1, x2, y2], depth_map)

        # Determine grasp type
        if obj_width > self.gripper_max_width:
            grasp_type = 'top'  # Object too wide, grasp from top
        elif h > 1.5 * w:
            grasp_type = 'side'  # Tall object, side grasp
        else:
            grasp_type = 'side'

        # Compute grasp point
        if grasp_type == 'side':
            grasp_y = y1 + (y2 - y1) * 0.4  # 40% from top
            grasp_x = (x1 + x2) / 2
            approach_vector = np.array([0, 0, -1])  # Approach from camera
        else:
            grasp_y = y1 + (y2 - y1) * 0.15  # 15% from top (neck)
            grasp_x = (x1 + x2) / 2
            approach_vector = np.array([0, -1, 0])  # Approach from above

        # Get depth at grasp point
        depth = self.get_depth_at_point(depth_map, grasp_x, grasp_y)
        if depth <= 0:
            return None

        # Convert to 3D
        position_3d = self.pixel_to_3d(grasp_x, grasp_y, depth)

        # Get class info
        class_name = "unknown"
        confidence = 0.0
        if detection.results:
            hypothesis = detection.results[0].hypothesis
            class_name = hypothesis.class_id
            confidence = hypothesis.score

        return GraspPoint(
            position_3d=position_3d,
            position_2d=np.array([grasp_x, grasp_y]),
            approach_vector=approach_vector,
            grasp_width=obj_width,
            confidence=confidence,
            grasp_type=grasp_type,
            class_name=class_name
        )

    def estimate_object_size(self, bbox: List[int], depth_map: np.ndarray) -> Tuple[float, float]:
        """Estimate physical object size from bbox and depth."""
        x1, y1, x2, y2 = bbox
        pixel_width = x2 - x1
        pixel_height = y2 - y1

        depth = self.get_depth_in_bbox(depth_map, bbox)
        if depth <= 0:
            return 0.065, 0.15  # Default can size

        width_m = pixel_width * depth / self.fx
        height_m = pixel_height * depth / self.fy

        return width_m, height_m

    def get_depth_at_point(self, depth_map: np.ndarray, x: float, y: float) -> float:
        """Get depth at point with bounds checking."""
        h, w = depth_map.shape[:2]
        x, y = int(x), int(y)
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        return float(depth_map[y, x])

    def get_depth_in_bbox(self, depth_map: np.ndarray, bbox: List[int]) -> float:
        """Get median depth in bounding box."""
        x1, y1, x2, y2 = bbox
        h, w = depth_map.shape[:2]
        x1 = max(0, min(x1, w-1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h-1))
        y2 = max(0, min(y2, h))

        if x1 >= x2 or y1 >= y2:
            return 0.0

        region = depth_map[y1:y2, x1:x2]
        return float(np.median(region))

    def pixel_to_3d(self, u: float, v: float, depth: float) -> np.ndarray:
        """Convert pixel + depth to 3D."""
        z = depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z])

    def approach_to_quaternion(self, approach: np.ndarray) -> np.ndarray:
        """Convert approach vector to quaternion (simplified)."""
        # Identity for now - would compute actual rotation in production
        return np.array([1.0, 0.0, 0.0, 0.0])

    def publish_visualization(self, grasps: List[GraspPoint]):
        """Publish image with grasp visualization."""
        if self.latest_image is None:
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

        for grasp in grasps:
            x, y = int(grasp.position_2d[0]), int(grasp.position_2d[1])

            # Color based on grasp type
            color = (0, 255, 255) if grasp.grasp_type == 'side' else (255, 0, 255)

            # Draw grasp point
            cv2.circle(img, (x, y), 8, color, -1)
            cv2.circle(img, (x, y), 12, color, 2)

            # Draw gripper indication
            if grasp.grasp_type == 'side':
                # Horizontal gripper
                cv2.line(img, (x - 25, y), (x + 25, y), color, 2)
                cv2.arrowedLine(img, (x - 40, y), (x - 25, y), color, 2, tipLength=0.4)
                cv2.arrowedLine(img, (x + 40, y), (x + 25, y), color, 2, tipLength=0.4)
            else:
                # Vertical gripper (top grasp)
                cv2.line(img, (x - 15, y), (x + 15, y), color, 2)
                cv2.arrowedLine(img, (x, y - 40), (x, y - 15), color, 2, tipLength=0.4)

            # Label
            label = f'{grasp.grasp_type} ({grasp.confidence:.2f})'
            cv2.putText(img, label, (x + 15, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Publish
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header = self.latest_image.header
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
