#!/usr/bin/env python3
"""
Tracker Node - Entry Point

Selects tracking algorithm based on 'tracker' parameter.

Usage:
  ros2 run perception_demo tracker_node --ros-args -p tracker:=botsort
  ros2 run perception_demo tracker_node --ros-args -p tracker:=bytetrack
"""

import rclpy


def main(args=None):
    rclpy.init(args=args)

    # Create temporary node to read parameter
    from rclpy.node import Node
    temp_node = Node('_tracker_param_reader')
    temp_node.declare_parameter('tracker', 'botsort')
    tracker = temp_node.get_parameter('tracker').value
    temp_node.destroy_node()

    # Select tracker implementation
    if tracker in ('botsort', 'bytetrack'):
        from .yolo_tracker import YoloTrackerNode
        node = YoloTrackerNode(tracker_type=tracker)
    else:
        rclpy.shutdown()
        raise ValueError(f"Unknown tracker: {tracker}. Use 'botsort' or 'bytetrack'.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
