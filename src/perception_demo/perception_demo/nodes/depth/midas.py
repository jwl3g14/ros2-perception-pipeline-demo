#!/usr/bin/env python3
"""
MiDaS Depth Estimation Node

Monocular depth estimation - infers depth from a single RGB image.

Usage:
  ros2 run perception_demo depth_node --ros-args -p method:=midas
  ros2 run perception_demo depth_node --ros-args -p method:=midas -p model_type:=DPT_Large
"""

import os
import cv2
import numpy as np
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Default model path (persistent storage)
DEFAULT_MODEL_DIR = '/ros_ws/external_data/models'


class MidasDepthNode(Node):
    """Depth estimation using MiDaS (monocular)."""

    def __init__(self):
        super().__init__('depth_node')

        # Parameters
        self.declare_parameter('model_type', 'MiDaS_small')  # MiDaS_small, DPT_Hybrid, DPT_Large
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')

        model_type = self.get_parameter('model_type').value
        device = self.get_parameter('device').value

        # Ensure model directory exists
        if not os.path.exists(DEFAULT_MODEL_DIR):
            os.makedirs(DEFAULT_MODEL_DIR, exist_ok=True)

        # Set torch hub cache to persistent storage
        torch.hub.set_dir(DEFAULT_MODEL_DIR)

        # Load MiDaS model
        self.get_logger().info(f'Loading MiDaS model: {model_type}')
        self.model = torch.hub.load('intel-isl/MiDaS', model_type, trust_repo=True)
        self.model.to(device)
        self.model.eval()
        self.device = device
        self.get_logger().info(f'MiDaS loaded on {device}')

        # Load transforms
        midas_transforms = torch.hub.load('intel-isl/MiDaS', 'transforms', trust_repo=True)
        if model_type == 'MiDaS_small':
            self.transform = midas_transforms.small_transform
        else:
            self.transform = midas_transforms.dpt_transform

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
        self.depth_viz_pub = self.create_publisher(Image, '/depth/image', 10)
        self.depth_raw_pub = self.create_publisher(Image, '/depth/raw', 10)

        self.get_logger().info('Depth node ready (method: midas). Waiting for images...')

    def image_callback(self, msg: Image):
        """Process image and publish depth estimation."""
        # Convert ROS Image to OpenCV (BGR -> RGB for MiDaS)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Transform for MiDaS
        input_batch = self.transform(rgb_image).to(self.device)

        # Run inference
        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=rgb_image.shape[:2],
                mode='bicubic',
                align_corners=False,
            ).squeeze()

        depth = prediction.cpu().numpy()

        # Normalize for visualization (0-255)
        depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_colormap = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_MAGMA)

        # Publish depth visualization
        depth_viz_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
        depth_viz_msg.header = msg.header
        self.depth_viz_pub.publish(depth_viz_msg)

        # Publish raw depth (32-bit float)
        depth_raw_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
        depth_raw_msg.header = msg.header
        self.depth_raw_pub.publish(depth_raw_msg)
