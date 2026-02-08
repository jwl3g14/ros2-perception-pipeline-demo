#!/usr/bin/env python3
"""
Camera Node - Entry Point

Selects camera source based on 'source' parameter.

Usage:
  ros2 run perception_demo camera_node --ros-args -p source:=sim
  ros2 run perception_demo camera_node --ros-args -p source:=webcam
  ros2 run perception_demo camera_node --ros-args -p source:=gazebo  (future)
"""

import rclpy


def main(args=None):
    rclpy.init(args=args)

    # Create temporary node to read parameter
    from rclpy.node import Node
    temp_node = Node('_camera_param_reader')
    temp_node.declare_parameter('source', 'sim')
    source = temp_node.get_parameter('source').value
    temp_node.destroy_node()

    # Select camera implementation based on source
    if source == 'sim':
        from .sim import SimCameraNode
        node = SimCameraNode()
    elif source == 'webcam':
        from .webcam import WebcamCameraNode
        node = WebcamCameraNode()
    else:
        rclpy.shutdown()
        raise ValueError(f"Unknown camera source: {source}. Use 'sim' or 'webcam'.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
