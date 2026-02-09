#!/usr/bin/env python3
"""
YOLO Object Tracking Node

Uses YOLOv8's built-in tracking (BoT-SORT or ByteTrack).
Publishes persistent track IDs and trajectory visualization.

Usage:
  ros2 run perception_demo tracker_node --ros-args -p tracker:=botsort
  ros2 run perception_demo tracker_node --ros-args -p tracker:=bytetrack
"""

import os
from collections import defaultdict
import cv2
import numpy as np
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

# Default model path (persistent storage)
DEFAULT_MODEL_DIR = '/ros_ws/external_data/models'
DEFAULT_MODEL_PATH = f'{DEFAULT_MODEL_DIR}/yolov8n.pt'

# Trajectory visualization settings
TRAIL_LENGTH = 30  # Number of past positions to draw
TRAIL_COLORS = [
    (255, 0, 0),    # Blue
    (0, 255, 0),    # Green
    (0, 0, 255),    # Red
    (255, 255, 0),  # Cyan
    (255, 0, 255),  # Magenta
    (0, 255, 255),  # Yellow
    (128, 0, 255),  # Purple
    (255, 128, 0),  # Orange
]


class YoloTrackerNode(Node):
    """Object tracking using YOLO with BoT-SORT or ByteTrack."""

    def __init__(self, tracker_type: str = 'botsort'):
        super().__init__('tracker_node')

        self.tracker_type = tracker_type

        # Parameters
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('show_trails', True)
        self.declare_parameter('trail_length', TRAIL_LENGTH)
        self.declare_parameter('camera_topic', '/camera/image_raw')

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        self.show_trails = self.get_parameter('show_trails').value
        self.trail_length = self.get_parameter('trail_length').value
        camera_topic = self.get_parameter('camera_topic').value

        # Ensure model directory exists
        model_dir = os.path.dirname(model_path)
        if model_dir and not os.path.exists(model_dir):
            os.makedirs(model_dir, exist_ok=True)

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(f'Model loaded on {device}, tracker: {tracker_type}')

        # Track history: {track_id: [(x, y), ...]}
        self.track_history = defaultdict(list)

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to camera topic: {camera_topic}')

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/tracks', 10)
        self.annotated_pub = self.create_publisher(Image, '/tracks/image', 10)

        self.get_logger().info(f'Tracker node ready ({tracker_type}). Waiting for images...')

    def image_callback(self, msg: Image):
        """Process incoming camera image and publish tracked objects."""
        # Convert ROS Image to OpenCV (BGR)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO tracking
        with torch.no_grad():
            results = self.model.track(
                cv_image,
                persist=True,  # Maintain track IDs across frames
                tracker=f'{self.tracker_type}.yaml',
                verbose=False
            )[0]

        # Build Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # Get annotated image
        annotated = results.plot()

        # Process tracked objects
        if results.boxes is not None and results.boxes.id is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            track_ids = results.boxes.id.cpu().numpy().astype(int)
            classes = results.boxes.cls.cpu().numpy().astype(int)
            confs = results.boxes.conf.cpu().numpy()

            for box, track_id, cls, conf in zip(boxes, track_ids, classes, confs):
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Update track history
                self.track_history[track_id].append((center_x, center_y))
                if len(self.track_history[track_id]) > self.trail_length:
                    self.track_history[track_id].pop(0)

                # Draw trajectory trail
                if self.show_trails and len(self.track_history[track_id]) > 1:
                    color = TRAIL_COLORS[track_id % len(TRAIL_COLORS)]
                    points = np.array(self.track_history[track_id], dtype=np.int32)
                    cv2.polylines(annotated, [points], False, color, 2)

                # Draw track ID label
                label = f'ID:{track_id} {self.model.names[cls]}'
                cv2.putText(
                    annotated, label,
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    TRAIL_COLORS[track_id % len(TRAIL_COLORS)], 2
                )

                # Build detection message
                detection = Detection2D()
                detection.header = msg.header

                # Bounding box
                detection.bbox.center.position.x = float(center_x)
                detection.bbox.center.position.y = float(center_y)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # Class, confidence, and track ID in class_id
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = f'{self.model.names[cls]}:ID={track_id}'
                hypothesis.hypothesis.score = float(conf)
                detection.results.append(hypothesis)

                detection_array.detections.append(detection)

        # Publish tracks
        self.detection_pub.publish(detection_array)

        # Publish annotated image with trails
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # Log active tracks
        n_tracks = len(detection_array.detections)
        if n_tracks > 0:
            track_ids_str = ', '.join(
                d.results[0].hypothesis.class_id for d in detection_array.detections
            )
            self.get_logger().info(f'Tracking {n_tracks}: {track_ids_str}')

    def cleanup_old_tracks(self, active_ids: set):
        """Remove tracks that are no longer active."""
        old_ids = set(self.track_history.keys()) - active_ids
        for old_id in old_ids:
            del self.track_history[old_id]
