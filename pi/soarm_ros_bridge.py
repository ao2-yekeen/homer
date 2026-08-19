#!/usr/bin/env python3
"""Guarded ROS 2 bridge for a six-servo SO-ARM follower.

Topics:
  /soarm/state_ticks          std_msgs/msg/Int16MultiArray
  /soarm/command_delta_ticks  std_msgs/msg/Int16MultiArray

Command order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll, gripper. Each command is relative to the current servo position
and is clipped to +/-20 ticks. The bridge starts read-only.
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

sys.path.insert(0, "/home/robot-b")
from STservo_sdk import COMM_SUCCESS, PortHandler, sts  # type: ignore


SERVO_IDS = (1, 2, 3, 4, 5, 6)
SERVO_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
# The previously available calibration belongs to a different arm pose/setup and
# must not be used to correct live positions. STS position registers are 0–4095.
SERVO_TICK_MIN = 0
SERVO_TICK_MAX = 4095
MAX_DELTA_TICKS = 20
SLOW_SPEED = 25
SLOW_ACCELERATION = 5
COMMAND_TIMEOUT_S = 0.25


class SoArmBridge(Node):
    def __init__(self) -> None:
        super().__init__("soarm_bridge")
        self.lock = threading.Lock()
        self.port = PortHandler("/dev/ttyACM0")
        if not self.port.openPort() or not self.port.setBaudRate(1_000_000):
            raise RuntimeError("Cannot open SO-ARM at /dev/ttyACM0, 1,000,000 baud")
        self.packet = sts(self.port)
        self.motion_active = False
        self.last_motion_command_s = time.monotonic()
        self._verify_servos()
        self.state_pub = self.create_publisher(Int16MultiArray, "/soarm/state_ticks", 10)
        self.create_subscription(
            Int16MultiArray, "/soarm/command_delta_ticks", self.command_callback, 10
        )
        self.create_timer(1.0, self.publish_state)
        self.create_timer(0.05, self.enforce_command_watchdog)
        self.publish_state()
        self.get_logger().info(
            "Ready. Commands are relative, clipped to +/-20 ticks, speed=25. "
            f"Order: {', '.join(SERVO_NAMES)}"
        )

    def _verify_servos(self) -> None:
        found = []
        for servo_id in SERVO_IDS:
            _, result, error = self.packet.ping(servo_id)
            if result != COMM_SUCCESS or error != 0:
                raise RuntimeError(f"Servo {servo_id} did not respond (result={result}, error={error})")
            found.append(str(servo_id))
        self.get_logger().info("Verified SO-ARM servo IDs: " + ", ".join(found))

    def read_positions(self) -> list[int]:
        positions = []
        for servo_id in SERVO_IDS:
            position, _, result, error = self.packet.ReadPosSpeed(servo_id)
            if result != COMM_SUCCESS or error != 0:
                raise RuntimeError(f"Read failed for servo {servo_id} (result={result}, error={error})")
            positions.append(position)
        return positions

    def publish_state(self) -> None:
        try:
            with self.lock:
                positions = self.read_positions()
            self.state_pub.publish(Int16MultiArray(data=positions))
        except Exception as exc:
            self.get_logger().error(f"State read failed: {exc}")

    def hold_current_positions(self) -> None:
        """Cancel any remaining position trajectory by targeting each live position."""
        with self.lock:
            current = self.read_positions()
            for servo_id, position in zip(SERVO_IDS, current):
                result, error = self.packet.WritePosEx(
                    servo_id, position, SLOW_SPEED, SLOW_ACCELERATION
                )
                if result != COMM_SUCCESS or error != 0:
                    raise RuntimeError(
                        f"Hold failed for servo {servo_id} (result={result}, error={error})"
                    )

    def stop_motion(self, reason: str) -> None:
        if not self.motion_active:
            return
        try:
            self.hold_current_positions()
            self.get_logger().info(f"Motion stopped: {reason}")
        except Exception as exc:
            self.get_logger().error(f"Unable to stop motion: {exc}")
        finally:
            self.motion_active = False

    def enforce_command_watchdog(self) -> None:
        if self.motion_active and time.monotonic() - self.last_motion_command_s > COMMAND_TIMEOUT_S:
            self.stop_motion("command watchdog expired")

    def command_callback(self, message: Int16MultiArray) -> None:
        if len(message.data) != len(SERVO_IDS):
            self.get_logger().error("Expected exactly six delta ticks; command ignored")
            return
        requested = [max(-MAX_DELTA_TICKS, min(MAX_DELTA_TICKS, int(value))) for value in message.data]
        if not any(requested):
            self.stop_motion("zero command")
            self.publish_state()
            return
        try:
            with self.lock:
                current = self.read_positions()
                targets = [
                    max(SERVO_TICK_MIN, min(SERVO_TICK_MAX, position + delta))
                    for position, delta in zip(current, requested)
                ]
                for servo_id, delta, target in zip(SERVO_IDS, requested, targets):
                    if delta == 0:
                        continue
                    result, error = self.packet.WritePosEx(
                        servo_id, target, SLOW_SPEED, SLOW_ACCELERATION
                    )
                    if result != COMM_SUCCESS or error != 0:
                        raise RuntimeError(
                            f"Write failed for servo {servo_id} "
                            f"(result={result}, error={error})"
                        )
            self.motion_active = True
            self.last_motion_command_s = time.monotonic()
            self.get_logger().info(f"Applied capped delta {requested}; targets {targets}")
            self.publish_state()
        except Exception as exc:
            self.get_logger().error(f"Command rejected after read/write failure: {exc}")

    def destroy_node(self) -> bool:
        self.port.closePort()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SoArmBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
