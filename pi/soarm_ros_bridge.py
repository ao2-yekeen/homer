#!/usr/bin/env python3
"""Guarded ROS 2 bridge for a six-servo SO-ARM follower.

Topics:
  /soarm/state_ticks          std_msgs/msg/Int16MultiArray
  /soarm/command_delta_ticks  std_msgs/msg/Int16MultiArray
  /soarm/command_pose_ticks   std_msgs/msg/Int16MultiArray
  /soarm/command_named_pose   std_msgs/msg/String

Command order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll, gripper. Each command is relative to the current servo position
and is clipped to +/-20 ticks. The bridge starts read-only.
"""

import json
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String

sys.path.insert(0, "/home/robot-b")
from STservo_sdk import (  # type: ignore
    COMM_SUCCESS,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    PortHandler,
    sts,
)


SERVO_IDS = (1, 2, 3, 4, 5, 6)
SERVO_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
# STS position registers are 0–4095. Joint limits are read from the EEPROM of
# each servo at bridge startup. Keep these tick limits separate from URDF
# radians: the simulation's joint-reference poses have not been aligned to the
# physical arm yet.
SERVO_TICK_MIN = 0
SERVO_TICK_MAX = 4095
MAX_DELTA_TICKS = 20
SLOW_SPEED = 25
SLOW_ACCELERATION = 5
COMMAND_TIMEOUT_S = 0.25
# Named poses are deliberately slower than manual jog commands.  They are
# captured from the live arm and remain disabled in configuration until an
# operator has checked every required clearance.
POSE_STEP_TICKS = 5
POSE_STEP_PERIOD_S = 0.10
POSE_COMPLETE_TOLERANCE_TICKS = 12
POSE_CONFIG_PATH = Path(__file__).with_name("soarm_named_poses.json")


class SoArmBridge(Node):
    def __init__(self) -> None:
        super().__init__("soarm_bridge")
        self.lock = threading.Lock()
        self.port = PortHandler("/dev/robot-soarm")
        if not self.port.openPort() or not self.port.setBaudRate(1_000_000):
            raise RuntimeError("Cannot open SO-ARM at /dev/robot-soarm, 1,000,000 baud")
        self.packet = sts(self.port)
        self.motion_active = False
        self.last_motion_command_s = time.monotonic()
        self.pose_target: Optional[list[int]] = None
        self._verify_servos()
        self.joint_tick_limits = self.read_eeprom_limits()
        self.targets = self.read_positions()
        self.state_pub = self.create_publisher(Int16MultiArray, "/soarm/state_ticks", 10)
        self.create_subscription(
            Int16MultiArray, "/soarm/command_delta_ticks", self.command_callback, 10
        )
        self.create_subscription(
            Int16MultiArray, "/soarm/command_pose_ticks", self.pose_command_callback, 10
        )
        self.create_subscription(String, "/soarm/command_named_pose", self.named_pose_callback, 10)
        self.create_timer(1.0, self.publish_state)
        self.create_timer(0.05, self.enforce_command_watchdog)
        self.create_timer(POSE_STEP_PERIOD_S, self.advance_named_pose)
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

    def read_eeprom_limits(self) -> tuple[tuple[int, int], ...]:
        """Load the configured travel range from every servo's EEPROM."""
        limits = []
        for servo_id, name in zip(SERVO_IDS, SERVO_NAMES):
            minimum, result, error = self.packet.read2ByteTxRx(
                servo_id, STS_MIN_ANGLE_LIMIT_L
            )
            if result != COMM_SUCCESS or error != 0:
                raise RuntimeError(
                    f"Could not read EEPROM minimum for {name} "
                    f"(result={result}, error={error})"
                )
            maximum, result, error = self.packet.read2ByteTxRx(
                servo_id, STS_MAX_ANGLE_LIMIT_L
            )
            if result != COMM_SUCCESS or error != 0:
                raise RuntimeError(
                    f"Could not read EEPROM maximum for {name} "
                    f"(result={result}, error={error})"
                )
            if not SERVO_TICK_MIN <= minimum <= maximum <= SERVO_TICK_MAX:
                raise RuntimeError(f"Invalid EEPROM limits for {name}: {minimum}–{maximum}")
            limits.append((minimum, maximum))
        self.get_logger().info(
            "Loaded EEPROM limits: "
            + ", ".join(
                f"{name}={minimum}–{maximum}"
                for name, (minimum, maximum) in zip(SERVO_NAMES, limits)
            )
        )
        return tuple(limits)

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
            self.targets = current
            for servo_id, position in zip(SERVO_IDS, current):
                result, error = self.packet.WritePosEx(
                    servo_id, position, SLOW_SPEED, SLOW_ACCELERATION
                )
                if result != COMM_SUCCESS or error != 0:
                    raise RuntimeError(
                        f"Hold failed for servo {servo_id} (result={result}, error={error})"
                    )

    @staticmethod
    def bounded_target(current: int, delta: int, minimum: int, maximum: int) -> int:
        """Constrain a target without forcing a sudden recovery move.

        A servo found outside its allowed interval may only move toward the
        interval. This retains the normal per-command delta cap while blocking
        any command that would move farther into a forbidden region.
        """
        proposed = current + delta
        if current < minimum:
            return max(current, proposed)
        if current > maximum:
            return min(current, proposed)
        return max(minimum, min(maximum, proposed))

    def stop_motion(self, reason: str) -> None:
        if not self.motion_active:
            return
        try:
            self.hold_current_positions()
            self.get_logger().info(f"Motion stopped: {reason}")
        except Exception as exc:
            self.get_logger().error(f"Unable to stop motion: {exc}")
        finally:
            self.pose_target = None
            self.motion_active = False

    def enforce_command_watchdog(self) -> None:
        if (
            self.motion_active
            and self.pose_target is None
            and time.monotonic() - self.last_motion_command_s > COMMAND_TIMEOUT_S
        ):
            self.stop_motion("command watchdog expired")

    def _valid_pose(self, values: object) -> Optional[list[int]]:
        if not isinstance(values, list) or len(values) != len(SERVO_IDS):
            return None
        try:
            pose = [int(value) for value in values]
        except (TypeError, ValueError):
            return None
        if any(
            value < minimum or value > maximum
            for value, (minimum, maximum) in zip(pose, self.joint_tick_limits)
        ):
            return None
        return pose

    def load_named_pose(self, name: str) -> Optional[list[int]]:
        try:
            config = json.loads(POSE_CONFIG_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Cannot read named-pose config: {exc}")
            return None
        if config.get("motion_enabled") is not True:
            self.get_logger().error(
                "Named-pose motion is disabled. Capture and physically validate poses first."
            )
            return None
        pose = self._valid_pose(config.get("poses", {}).get(name))
        if pose is None:
            self.get_logger().error(f"Named pose '{name}' is missing or violates joint limits")
        return pose

    def named_pose_callback(self, message: String) -> None:
        name = message.data.strip().lower()
        if name == "stop":
            self.stop_motion("named-pose stop command")
            self.publish_state()
            return
        if name not in {"home", "approach", "grasp", "lift"}:
            self.get_logger().error("Unknown named pose; use home, approach, grasp, lift, or stop")
            return
        pose = self.load_named_pose(name)
        if pose is None:
            return
        with self.lock:
            # Ramp from the last target we send, not from live feedback. A
            # servo can legitimately lag a low-speed command by several ticks;
            # repeatedly commanding only a few ticks ahead of feedback can
            # leave it inside its position deadband indefinitely.
            self.targets = self.read_positions()
            self.pose_target = pose
            self.motion_active = True
        self.get_logger().info(
            f"Starting named pose '{name}' at {POSE_STEP_TICKS} ticks/step; target={pose}"
        )

    def pose_command_callback(self, message: Int16MultiArray) -> None:
        """Follow a validated absolute six-joint target at named-pose speed.

        This is intentionally separate from manual delta commands so a recorded
        position trajectory can be replayed without bypassing EEPROM limits or
        the bridge's slow, incremental motion path.
        """
        pose = self._valid_pose(list(message.data))
        if pose is None:
            self.get_logger().error("Absolute pose must contain six in-limit tick values")
            return
        try:
            with self.lock:
                self.targets = self.read_positions()
                self.pose_target = pose
                self.motion_active = True
            self.get_logger().info(f"Starting recorded pose target={pose}")
        except Exception as exc:
            self.get_logger().error(f"Recorded pose rejected after read failure: {exc}")

    def advance_named_pose(self) -> None:
        if self.pose_target is None:
            return
        try:
            with self.lock:
                current = self.read_positions()
                target = self.pose_target
                if all(
                    abs(position - desired) <= POSE_COMPLETE_TOLERANCE_TICKS
                    for position, desired in zip(current, target)
                ):
                    self.targets = target
                    self.pose_target = None
                    self.motion_active = False
                    self.get_logger().info("Named pose complete")
                    return
                next_targets = [
                    commanded + max(
                        -POSE_STEP_TICKS, min(POSE_STEP_TICKS, desired - commanded)
                    )
                    for commanded, desired in zip(self.targets, target)
                ]
                for servo_id, commanded, next_target in zip(SERVO_IDS, self.targets, next_targets):
                    if commanded == next_target:
                        continue
                    result, error = self.packet.WritePosEx(
                        servo_id, next_target, SLOW_SPEED, SLOW_ACCELERATION
                    )
                    if result != COMM_SUCCESS or error != 0:
                        raise RuntimeError(
                            f"Named pose write failed for servo {servo_id} "
                            f"(result={result}, error={error})"
                        )
                self.targets = next_targets
        except Exception as exc:
            self.get_logger().error(f"Named pose stopped after read/write failure: {exc}")
            self.stop_motion("named-pose read/write failure")

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
                if self.pose_target is not None:
                    self.get_logger().info("Manual jog command cancelled named-pose motion")
                    self.pose_target = None
                    self.targets = self.read_positions()
                targets = [
                    self.bounded_target(position, delta, minimum, maximum)
                    for position, delta, (minimum, maximum) in zip(
                        self.targets, requested, self.joint_tick_limits
                    )
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
                self.targets = targets
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
