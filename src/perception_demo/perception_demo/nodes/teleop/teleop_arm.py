#!/usr/bin/env python3
"""
Keyboard teleoperation for robot arm.

Key mappings:
  Q/W - shoulder_pan (rotate base left/right)
  A/S - shoulder_lift (tilt up/down)
  Z/X - elbow (bend)
  1/2 - wrist (rotate)

  SPACE - reset to home position
  ESC/Ctrl+C - quit

Hold keys to move continuously. Release to stop.
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TeleopArm(Node):
    def __init__(self):
        super().__init__('teleop_arm')

        # Joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint'
        ]

        # Current positions (start at home)
        self.positions = [0.0, 0.0, 0.0, 0.0]

        # Joint limits (radians)
        self.limits = {
            'shoulder_pan_joint': (-3.14, 3.14),
            'shoulder_lift_joint': (-1.57, 1.57),
            'elbow_joint': (-2.5, 2.5),
            'wrist_joint': (-3.14, 3.14),
        }

        # Movement speed (radians per keypress)
        self.step = 0.1

        # Key mappings: key -> (joint_index, direction)
        self.key_map = {
            'q': (0, 1),   # shoulder_pan +
            'w': (0, -1),  # shoulder_pan -
            'a': (1, 1),   # shoulder_lift +
            's': (1, -1),  # shoulder_lift -
            'z': (2, 1),   # elbow +
            'x': (2, -1),  # elbow -
            '1': (3, 1),   # wrist +
            '2': (3, -1),  # wrist -
        }

        # Action client
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Connected!')

    def send_position(self):
        """Send current positions to controller."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions.copy()
        point.time_from_start = Duration(sec=0, nanosec=200_000_000)  # 200ms

        goal.trajectory.points = [point]

        # Send goal (don't wait for result)
        self.action_client.send_goal_async(goal)

    def clamp_position(self, joint_idx, value):
        """Clamp position to joint limits."""
        joint_name = self.joint_names[joint_idx]
        low, high = self.limits[joint_name]
        return max(low, min(high, value))

    def process_key(self, key):
        """Process a keypress."""
        if key == ' ':
            # Reset to home
            self.positions = [0.0, 0.0, 0.0, 0.0]
            self.send_position()
            self.get_logger().info('Reset to home position')
            return True

        if key in self.key_map:
            joint_idx, direction = self.key_map[key]
            new_pos = self.positions[joint_idx] + (direction * self.step)
            self.positions[joint_idx] = self.clamp_position(joint_idx, new_pos)
            self.send_position()

            # Show current positions
            pos_str = ' '.join([f'{p:.2f}' for p in self.positions])
            print(f'\rPositions: [{pos_str}]', end='', flush=True)
            return True

        return False


def get_key(settings, timeout=0.1):
    """Get a single keypress with timeout."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TeleopArm()

    print('\n' + '='*50)
    print('Robot Arm Keyboard Teleop')
    print('='*50)
    print('Key mappings:')
    print('  Q/W - shoulder_pan (rotate base)')
    print('  A/S - shoulder_lift (tilt)')
    print('  Z/X - elbow (bend)')
    print('  1/2 - wrist (rotate)')
    print('')
    print('  SPACE - reset to home')
    print('  ESC or Ctrl+C - quit')
    print('='*50 + '\n')

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
                print('\nExiting...')
                break

            if key:
                node.process_key(key.lower())

            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(f'\nError: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
