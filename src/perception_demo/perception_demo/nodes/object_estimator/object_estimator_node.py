#!/usr/bin/env python3
"""
Object Estimator Node

Combines 2D detections with depth to produce complete 3D object state.

Subscribes:
  /tracks (Detection2DArray) - tracked objects with bounding boxes
  /depth/raw (Image, 32FC1) - raw depth values

Publishes:
  /objects (Detection3DArray) - complete 3D object state:
    - 3D position (bbox.center.position)
    - 3D orientation (bbox.center.orientation)
    - 3D dimensions (bbox.size)
    - class name, confidence (results)
    - track ID (id)
  /objects/image (Image) - visualization with 3D info overlay

Usage:
  ros2 run perception_demo object_estimator_node
"""

import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D
from vision_msgs.msg import BoundingBox3D, ObjectHypothesisWithPose, ObjectHypothesis
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


@dataclass
class ObjectState:
    """Complete 3D object state."""
    # Identity
    track_id: str
    class_name: str
    confidence: float

    # 2D (for visualization)
    bbox_2d: tuple  # (cx, cy, w, h) in pixels

    # 3D Pose
    position: np.ndarray      # [x, y, z] in meters
    orientation: np.ndarray   # [w, x, y, z] quaternion

    # 3D Dimensions
    dimensions: np.ndarray    # [width, height, depth] in meters


class ObjectEstimatorNode(Node):
    """ROS2 node for 3D object state estimation."""

    def __init__(self):
        super().__init__('object_estimator_node')

        # Parameters
        self.declare_parameter('detection_topic', '/tracks')
        self.declare_parameter('depth_topic', '/depth/raw')
        self.declare_parameter('fx', 600.0)  # Camera focal length x
        self.declare_parameter('fy', 600.0)  # Camera focal length y
        self.declare_parameter('cx', 320.0)  # Principal point x
        self.declare_parameter('cy', 240.0)  # Principal point y
        self.declare_parameter('depth_scale', 1.0)

        detection_topic = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.depth_scale = self.get_parameter('depth_scale').value

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers with message synchronization
        self.detection_sub = Subscriber(self, Detection2DArray, detection_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub],
            queue_size=10,
            slop=0.5
        )
        self.sync.registerCallback(self.synced_callback)

        # Publishers
        self.objects_pub = self.create_publisher(Detection3DArray, '/objects', 10)
        self.image_pub = self.create_publisher(Image, '/objects/image', 10)

        # Raw image for visualization
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.latest_image = None

        self.get_logger().info(f'Object estimator ready. Listening to {detection_topic} + {depth_topic}')

    def image_callback(self, msg: Image):
        """Store latest image for visualization."""
        self.latest_image = msg

    def synced_callback(self, detection_msg: Detection2DArray, depth_msg: Image):
        """Process synchronized detection and depth messages."""
        # Convert depth image - expecting 32FC1 raw depth
        depth_map = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # Normalize depth (MiDaS outputs relative depth)
        depth_max = depth_map.max()
        if depth_max > 0:
            depth_normalized = depth_map / (depth_max + 1e-6)
        else:
            depth_normalized = depth_map

        # Estimate 3D state for each detection
        objects = []
        for detection in detection_msg.detections:
            obj = self.estimate_object_state(detection, depth_normalized, depth_map)
            if obj is not None:
                objects.append(obj)

        # Convert to Detection3DArray
        detection3d_array = Detection3DArray()
        detection3d_array.header = detection_msg.header
        detection3d_array.header.frame_id = 'camera_optical_frame'

        for obj in objects:
            det3d = Detection3D()

            # Set bounding box with pose and size
            det3d.bbox = BoundingBox3D()
            det3d.bbox.center = Pose()
            det3d.bbox.center.position = Point(
                x=float(obj.position[0]),
                y=float(obj.position[1]),
                z=float(obj.position[2])
            )
            det3d.bbox.center.orientation = Quaternion(
                w=float(obj.orientation[0]),
                x=float(obj.orientation[1]),
                y=float(obj.orientation[2]),
                z=float(obj.orientation[3])
            )
            det3d.bbox.size = Vector3(
                x=float(obj.dimensions[0]),  # width
                y=float(obj.dimensions[1]),  # height
                z=float(obj.dimensions[2])   # depth
            )

            # Set classification result
            result = ObjectHypothesisWithPose()
            result.hypothesis = ObjectHypothesis()
            result.hypothesis.class_id = obj.class_name
            result.hypothesis.score = float(obj.confidence)
            det3d.results.append(result)

            # Set track ID
            det3d.id = obj.track_id

            detection3d_array.detections.append(det3d)

        self.objects_pub.publish(detection3d_array)

        # Publish visualization
        if self.latest_image is not None:
            self.publish_visualization(objects)

        # Log
        if len(objects) > 0:
            obj_strs = [f'{o.class_name}@({o.position[0]:.2f},{o.position[1]:.2f},{o.position[2]:.2f})'
                        f'[{o.dimensions[0]*100:.0f}x{o.dimensions[1]*100:.0f}cm]'
                        for o in objects]
            self.get_logger().info(f'Objects: {", ".join(obj_strs)}')

    def estimate_object_state(self, detection, depth_normalized: np.ndarray,
                              depth_raw: np.ndarray) -> Optional[ObjectState]:
        """Estimate complete 3D object state from detection and depth."""
        # Extract 2D bbox
        bbox = detection.bbox
        cx = bbox.center.position.x
        cy = bbox.center.position.y
        w = bbox.size_x
        h = bbox.size_y

        x1 = int(cx - w/2)
        y1 = int(cy - h/2)
        x2 = int(cx + w/2)
        y2 = int(cy + h/2)

        # Get depth at object center (normalized for position calculation)
        depth = self.get_depth_at_bbox(depth_normalized, [x1, y1, x2, y2])
        if depth <= 0:
            return None

        # 3D position from center
        position = self.pixel_to_3d(cx, cy, depth)

        # 3D orientation (assume vertical for bottles/cans)
        orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion

        # 3D dimensions from bbox size + depth
        dimensions = self.estimate_dimensions(w, h, depth)

        # Get class info and track ID
        class_name = "unknown"
        confidence = 0.0
        track_id = ""

        if detection.results:
            hypothesis = detection.results[0].hypothesis
            class_name = hypothesis.class_id
            confidence = hypothesis.score

        # Extract track ID if available (from tracker)
        if hasattr(detection, 'id') and detection.id:
            track_id = detection.id
        else:
            # Fallback: use class name
            track_id = class_name

        return ObjectState(
            track_id=track_id,
            class_name=class_name,
            confidence=confidence,
            bbox_2d=(int(cx), int(cy), int(w), int(h)),
            position=position,
            orientation=orientation,
            dimensions=dimensions
        )

    def get_depth_at_bbox(self, depth_map: np.ndarray, bbox: List[int]) -> float:
        """Get center-weighted depth in bounding box."""
        x1, y1, x2, y2 = bbox
        h, w = depth_map.shape[:2]

        # Clamp to image bounds
        x1 = max(0, min(x1, w-1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h-1))
        y2 = max(0, min(y2, h))

        if x1 >= x2 or y1 >= y2:
            return 0.0

        region = depth_map[y1:y2, x1:x2]
        rh, rw = region.shape[:2]
        if rh == 0 or rw == 0:
            return 0.0

        # Center-weighted average
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

    def estimate_dimensions(self, pixel_w: float, pixel_h: float, depth: float) -> np.ndarray:
        """Estimate 3D dimensions from 2D bbox and depth."""
        # Convert pixel size to metric using pinhole camera model
        width = pixel_w * depth * self.depth_scale / self.fx
        height = pixel_h * depth * self.depth_scale / self.fy

        # Estimate depth (front-to-back) - assume cylindrical object
        # For cans/bottles, depth ≈ width
        obj_depth = width

        return np.array([width, height, obj_depth])

    def publish_visualization(self, objects: List[ObjectState]):
        """Publish image with 3D object state visualization."""
        if self.latest_image is None:
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

        for obj in objects:
            cx, cy, w, h = obj.bbox_2d
            x1, y1 = cx - w//2, cy - h//2
            x2, y2 = cx + w//2, cy + h//2

            # Draw bounding box
            color = (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            # Draw coordinate axes at center
            axis_len = 25
            cv2.arrowedLine(img, (cx, cy), (cx + axis_len, cy), (0, 0, 255), 2, tipLength=0.2)  # X
            cv2.arrowedLine(img, (cx, cy), (cx, cy + axis_len), (0, 255, 0), 2, tipLength=0.2)  # Y
            cv2.circle(img, (cx, cy), 4, (255, 0, 0), -1)  # Z

            # Position text
            pos_text = f'({obj.position[0]:.2f}, {obj.position[1]:.2f}, {obj.position[2]:.2f})'
            cv2.putText(img, pos_text, (x1, y1 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # Dimensions text
            dim_text = f'{obj.dimensions[0]*100:.0f}x{obj.dimensions[1]*100:.0f}cm'
            cv2.putText(img, dim_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            # Class name
            cv2.putText(img, f'{obj.class_name} [{obj.track_id}]', (x1, y2 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Publish
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header = self.latest_image.header
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
