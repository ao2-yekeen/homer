#!/usr/bin/env python3
"""Capture the live SO-ARM tick values as a named pose without commanding motion."""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


POSE_NAMES = ("home", "approach", "grasp", "lift")
JOINT_ORDER = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
CONFIG_PATH = Path(__file__).with_name("soarm_named_poses.json")


class StateReader(Node):
    def __init__(self) -> None:
        super().__init__("soarm_pose_capture")
        self.state: Optional[list[int]] = None
        self.create_subscription(Int16MultiArray, "/soarm/state_ticks", self._callback, 1)

    def _callback(self, message: Int16MultiArray) -> None:
        if len(message.data) == len(JOINT_ORDER):
            self.state = [int(value) for value in message.data]


def read_state(timeout_s: float) -> list[int]:
    rclpy.init()
    node = StateReader()
    try:
        deadline = node.get_clock().now().nanoseconds / 1_000_000_000 + timeout_s
        while node.state is None and node.get_clock().now().nanoseconds / 1_000_000_000 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.state is None:
            raise RuntimeError("No /soarm/state_ticks message received")
        return node.state
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose", choices=POSE_NAMES)
    parser.add_argument(
        "--confirm-safe",
        action="store_true",
        help="required acknowledgement that this exact physical pose was inspected",
    )
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if not args.confirm_safe:
        parser.error("refusing to save a pose without --confirm-safe")
    if not CONFIG_PATH.exists():
        raise RuntimeError(f"Missing config: {CONFIG_PATH}")

    state = read_state(args.timeout)
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    config.setdefault("poses", {})[args.pose] = state
    config.setdefault("captured_at_utc", {})[args.pose] = datetime.now(timezone.utc).isoformat()
    CONFIG_PATH.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    print(f"Captured {args.pose}: {dict(zip(JOINT_ORDER, state))}")
    print("Named-pose motion remains disabled until physical path validation is complete.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
