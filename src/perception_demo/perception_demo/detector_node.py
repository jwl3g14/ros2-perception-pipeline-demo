#!/usr/bin/env python3
"""
YOLO Object Detection Node

Subscribes to: /camera/image_raw (sensor_msgs/Image)
Publishes to:  /detections (vision_msgs/Detection2DArray)

This is the core perception node - similar to what retail robotics uses
for detecting products on store shelves.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(f'Model loaded on {device}')

        # CV Bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()

        # Subscriber: camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # QoS queue size
        )

        # Publisher: detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        # Publisher: annotated image (for visualization)
        self.annotated_pub = self.create_publisher(
            Image,
            '/detections/image',
            10
        )

        self.get_logger().info('Detector node ready. Waiting for images on /camera/image_raw...')

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
        annotated = results.plot()  # YOLO's built-in visualization
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # Log detection count
        n_detections = len(detection_array.detections)
        if n_detections > 0:
            self.get_logger().info(f'Detected {n_detections} objects')


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
