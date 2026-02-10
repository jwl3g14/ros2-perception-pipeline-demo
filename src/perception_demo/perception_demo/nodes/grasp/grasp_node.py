#!/usr/bin/env python3
"""
Grasp Point Prediction Node

Predicts optimal grasp points based on 3D object state.

Subscribes:
  /objects (Detection3DArray) - complete 3D object state from object_estimator

Publishes:
  /grasps (PoseArray) - grasp poses (position + approach direction)
  /grasps/image (Image) - visualization with grasp points

Usage:
  ros2 run perception_demo grasp_node
"""

import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge


@dataclass
class GraspPoint:
    """Grasp point with metadata."""
    position_3d: np.ndarray   # [x, y, z] in meters
    position_2d: tuple        # (u, v) in pixels for visualization
    approach_vector: np.ndarray
    grasp_type: str           # 'side' or 'top'
    class_name: str
    confidence: float
    object_dimensions: np.ndarray  # [w, h, d] for gripper feasibility


class GraspPredictorNode(Node):
    """ROS2 node for grasp point prediction."""

    def __init__(self):
        super().__init__('grasp_node')

        # Parameters
        self.declare_parameter('objects_topic', '/objects')
        self.declare_parameter('gripper_max_width', 0.1)  # 10cm
        self.declare_parameter('fx', 600.0)  # For 2D projection
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        objects_topic = self.get_parameter('objects_topic').value
        self.gripper_max_width = self.get_parameter('gripper_max_width').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribe to objects (has everything we need!)
        self.objects_sub = self.create_subscription(
            Detection3DArray, objects_topic, self.objects_callback, 10
        )

        # Publishers
        self.grasp_pub = self.create_publisher(PoseArray, '/grasps', 10)
        self.image_pub = self.create_publisher(Image, '/grasps/image', 10)

        # Raw image for visualization
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.latest_image = None

        self.get_logger().info(f'Grasp node ready. Listening to {objects_topic}')

    def image_callback(self, msg: Image):
        """Store latest image for visualization."""
        self.latest_image = msg

    def objects_callback(self, msg: Detection3DArray):
        """Process 3D objects and predict grasps."""
        grasps = []
        for detection3d in msg.detections:
            grasp = self.predict_grasp(detection3d)
            if grasp is not None:
                grasps.append(grasp)

        # Publish PoseArray
        grasp_array = PoseArray()
        grasp_array.header = msg.header

        for g in grasps:
            ros_pose = Pose()
            ros_pose.position = Point(
                x=float(g.position_3d[0]),
                y=float(g.position_3d[1]),
                z=float(g.position_3d[2])
            )
            quat = self.approach_to_quaternion(g.approach_vector)
            ros_pose.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
            grasp_array.poses.append(ros_pose)

        self.grasp_pub.publish(grasp_array)

        # Publish visualization
        if self.latest_image is not None:
            self.publish_visualization(grasps)

        # Log
        if len(grasps) > 0:
            grasp_strs = [f'{g.class_name}:{g.grasp_type}' for g in grasps]
            self.get_logger().info(f'Grasps: {", ".join(grasp_strs)}')

    def predict_grasp(self, detection3d) -> Optional[GraspPoint]:
        """Predict grasp point from 3D object state."""
        # Extract object state from Detection3D
        bbox3d = detection3d.bbox

        # 3D position (object center)
        obj_position = np.array([
            bbox3d.center.position.x,
            bbox3d.center.position.y,
            bbox3d.center.position.z
        ])

        # 3D dimensions
        obj_dimensions = np.array([
            bbox3d.size.x,  # width
            bbox3d.size.y,  # height
            bbox3d.size.z   # depth
        ])

        # Class and confidence
        class_name = "unknown"
        confidence = 0.0
        if detection3d.results:
            class_name = detection3d.results[0].hypothesis.class_id
            confidence = detection3d.results[0].hypothesis.score

        # Determine grasp type based on object dimensions
        width, height, depth = obj_dimensions

        if width > self.gripper_max_width:
            # Object too wide for side grasp, try top
            grasp_type = 'top'
        elif height > 1.5 * width:
            # Tall object (can, bottle) - side grasp
            grasp_type = 'side'
        else:
            # Default to side grasp
            grasp_type = 'side'

        # Compute grasp position (offset from object center)
        if grasp_type == 'side':
            # Side grasp: at 40% from top (10% above center)
            grasp_offset_y = -height * 0.1  # Move up (negative Y in camera frame)
            grasp_position = obj_position.copy()
            grasp_position[1] += grasp_offset_y
            approach_vector = np.array([0, 0, -1])  # Approach from camera
        else:
            # Top grasp: at 15% from top
            grasp_offset_y = -height * 0.35  # Move up to top
            grasp_position = obj_position.copy()
            grasp_position[1] += grasp_offset_y
            approach_vector = np.array([0, -1, 0])  # Approach from above

        # Project grasp position to 2D for visualization
        grasp_2d = self.project_to_2d(grasp_position)

        return GraspPoint(
            position_3d=grasp_position,
            position_2d=grasp_2d,
            approach_vector=approach_vector,
            grasp_type=grasp_type,
            class_name=class_name,
            confidence=confidence,
            object_dimensions=obj_dimensions
        )

    def project_to_2d(self, point_3d: np.ndarray) -> tuple:
        """Project 3D point to 2D image coordinates."""
        if point_3d[2] <= 0:
            return (0, 0)
        u = int(point_3d[0] * self.fx / point_3d[2] + self.cx)
        v = int(point_3d[1] * self.fy / point_3d[2] + self.cy)
        return (u, v)

    def approach_to_quaternion(self, approach: np.ndarray) -> np.ndarray:
        """Convert approach vector to quaternion.

        For side grasp (approach -Z): identity quaternion
        For top grasp (approach -Y): 90 degree rotation around X
        """
        if approach[2] < 0:  # Side grasp
            return np.array([1.0, 0.0, 0.0, 0.0])
        else:  # Top grasp
            angle = np.pi / 2
            return np.array([np.cos(angle/2), np.sin(angle/2), 0.0, 0.0])

    def publish_visualization(self, grasps: List[GraspPoint]):
        """Publish image with grasp visualization."""
        if self.latest_image is None:
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

        for grasp in grasps:
            x, y = grasp.position_2d

            # Skip if out of bounds
            h, w = img.shape[:2]
            if x < 0 or x >= w or y < 0 or y >= h:
                continue

            # Color based on grasp type
            color = (0, 255, 255) if grasp.grasp_type == 'side' else (255, 0, 255)

            # Check if graspable
            graspable = grasp.object_dimensions[0] <= self.gripper_max_width
            if not graspable:
                color = (0, 0, 255)  # Red for too wide

            # Draw grasp point
            cv2.circle(img, (x, y), 8, color, -1)
            cv2.circle(img, (x, y), 12, color, 2)

            # Draw gripper indication
            if grasp.grasp_type == 'side':
                cv2.line(img, (x - 25, y), (x + 25, y), color, 2)
                cv2.arrowedLine(img, (x - 40, y), (x - 25, y), color, 2, tipLength=0.4)
                cv2.arrowedLine(img, (x + 40, y), (x + 25, y), color, 2, tipLength=0.4)
            else:
                cv2.line(img, (x - 15, y), (x + 15, y), color, 2)
                cv2.arrowedLine(img, (x, y - 40), (x, y - 15), color, 2, tipLength=0.4)

            # Label
            status = "OK" if graspable else "TOO WIDE"
            label = f'{grasp.grasp_type} [{status}]'
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
