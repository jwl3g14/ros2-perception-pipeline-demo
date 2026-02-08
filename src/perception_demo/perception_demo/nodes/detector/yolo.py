#!/usr/bin/env python3
"""
YOLO Object Detection Node

Usage:
  ros2 run perception_demo detector_node --ros-args -p model:=yolo
  ros2 run perception_demo detector_node --ros-args -p model:=yolo -p model_path:=/path/to/custom.pt
"""

import os
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

# Default model path (persistent storage)
DEFAULT_MODEL_DIR = '/ros_ws/external_data/models'
DEFAULT_MODEL_PATH = f'{DEFAULT_MODEL_DIR}/yolov8n.pt'


class YoloDetectorNode(Node):
    """Object detection using YOLO."""

    def __init__(self):
        super().__init__('detector_node')

        # Parameters
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value

        # Ensure model directory exists
        model_dir = os.path.dirname(model_path)
        if model_dir and not os.path.exists(model_dir):
            os.makedirs(model_dir, exist_ok=True)
            self.get_logger().info(f'Created model directory: {model_dir}')

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(f'Model loaded on {device}')

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/detections/image', 10)

        self.get_logger().info('Detector node ready (model: yolo). Waiting for images...')

    def image_callback(self, msg: Image):
        """Process incoming camera image and publish detections."""
        # Convert ROS Image to OpenCV (BGR)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        with torch.no_grad():
            results = self.model(cv_image, verbose=False)[0]

        # Build Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in results.boxes:
            if box.conf[0] < self.conf_threshold:
                continue

            detection = Detection2D()
            detection.header = msg.header

            # Bounding box (center x, center y, width, height)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            detection.bbox.center.position.x = float((x1 + x2) / 2)
            detection.bbox.center.position.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            # Class and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.names[int(box.cls[0])]
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        # Publish detections
        self.detection_pub.publish(detection_array)

        # Publish annotated image
        annotated = results.plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # Log detection count
        n_detections = len(detection_array.detections)
        if n_detections > 0:
            self.get_logger().info(f'Detected {n_detections} objects')
