#!/usr/bin/env python3
"""
Depth Node - Entry Point

Selects depth estimation method based on 'method' parameter.

Usage:
  ros2 run perception_demo depth_node --ros-args -p method:=midas
  ros2 run perception_demo depth_node --ros-args -p method:=realsense  (future)
"""

import rclpy


def main(args=None):
    rclpy.init(args=args)

    # Create temporary node to read parameter
    from rclpy.node import Node
    temp_node = Node('_depth_param_reader')
    temp_node.declare_parameter('method', 'midas')
    method = temp_node.get_parameter('method').value
    temp_node.destroy_node()

    # Select depth implementation based on method
    if method == 'midas':
        from .midas import MidasDepthNode
        node = MidasDepthNode()
    else:
        rclpy.shutdown()
        raise ValueError(f"Unknown depth method: {method}. Use 'midas'.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
