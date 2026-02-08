#!/usr/bin/env python3
"""
Detector Node - Entry Point

Selects detection model based on 'model' parameter.

Usage:
  ros2 run perception_demo detector_node --ros-args -p model:=yolo
  ros2 run perception_demo detector_node --ros-args -p model:=yolo -p model_path:=/path/to/custom.pt
"""

import rclpy


def main(args=None):
    rclpy.init(args=args)

    # Create temporary node to read parameter
    from rclpy.node import Node
    temp_node = Node('_detector_param_reader')
    temp_node.declare_parameter('model', 'yolo')
    model = temp_node.get_parameter('model').value
    temp_node.destroy_node()

    # Select detector implementation based on model
    if model == 'yolo':
        from .yolo import YoloDetectorNode
        node = YoloDetectorNode()
    else:
        rclpy.shutdown()
        raise ValueError(f"Unknown detector model: {model}. Use 'yolo'.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
