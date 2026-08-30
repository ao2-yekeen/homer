#!/usr/bin/env python3
"""Explicit, range-limited command for the camera/neck tilt servo.

This never runs automatically.  The operator must select a measured angle in
the conservative 80--150 degree range and keep the emergency stop available.
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


MIN_ANGLE = 80
MAX_ANGLE = 150


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Command one safe camera tilt angle.")
    parser.add_argument("--angle", type=int, required=True, help="Camera angle in degrees (80--150).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not MIN_ANGLE <= args.angle <= MAX_ANGLE:
        raise SystemExit(f"--angle must be within {MIN_ANGLE}..{MAX_ANGLE}; no command was sent")
    rclpy.init()
    node = Node("camera_tilt_command")
    publisher = node.create_publisher(Int32, "/teleop/neck_angle", 1)
    # Let ROS discovery complete before publishing exactly one intentional move.
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    publisher.publish(Int32(data=args.angle))
    node.get_logger().info(f"Commanded camera tilt angle {args.angle} degrees")
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
