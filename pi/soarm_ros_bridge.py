#!/usr/bin/env python3
"""Guarded ROS 2 bridge for a six-servo SO-ARM follower.

Topics:
  /soarm/state_ticks          std_msgs/msg/Int16MultiArray
  /soarm/command_delta_ticks  std_msgs/msg/Int16MultiArray
  /soarm/command_pose_ticks   std_msgs/msg/Int16MultiArray
  /soarm/command_named_pose   std_msgs/msg/String
  /soarm/gripper/telemetry    std_msgs/msg/String (JSON)
  /soarm/gripper/contact      std_msgs/msg/String
  /soarm/gripper/command      std_msgs/msg/String (close or stop)
  /soarm/gripper/auto_status  std_msgs/msg/String

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

from gripper_contact import (
    AutoCloseAction,
    AutoCloseConfig,
    AutomaticCloseController,
    ContactConfig,
    ContactState,
    GripperContactDetector,
)

sys.path.insert(0, "/home/robot-b")
from STservo_sdk import (  # type: ignore
    COMM_SUCCESS,
    STS_MOVING,
    STS_PRESENT_CURRENT_L,
    STS_PRESENT_LOAD_L,
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
GRIPPER_ID = SERVO_IDS[-1]
GRIPPER_FEEDBACK_PERIOD_S = 0.1
GRIPPER_CONFIG_PATH = Path(__file__).with_name("gripper_contact.json")
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
        self.gripper_closing_motion = False
        self.last_motion_command_s = time.monotonic()
        self.pose_target: Optional[list[int]] = None
        self._verify_servos()
        self.joint_tick_limits = self.read_eeprom_limits()
        self.targets = self.read_positions()
        self.contact_detector, self.auto_close = self._load_gripper_control()
        self.state_pub = self.create_publisher(Int16MultiArray, "/soarm/state_ticks", 10)
        self.gripper_telemetry_pub = self.create_publisher(
            String, "/soarm/gripper/telemetry", 10
        )
        self.gripper_contact_pub = self.create_publisher(
            String, "/soarm/gripper/contact", 10
        )
        self.gripper_auto_status_pub = self.create_publisher(
            String, "/soarm/gripper/auto_status", 10
        )
        self.create_subscription(
            Int16MultiArray, "/soarm/command_delta_ticks", self.command_callback, 10
        )
        self.create_subscription(
            Int16MultiArray, "/soarm/command_pose_ticks", self.pose_command_callback, 10
        )
        self.create_subscription(String, "/soarm/command_named_pose", self.named_pose_callback, 10)
        self.create_subscription(String, "/soarm/gripper/command", self.gripper_command_callback, 10)
        self.create_timer(1.0, self.publish_state)
        self.create_timer(GRIPPER_FEEDBACK_PERIOD_S, self.publish_gripper_feedback)
        self.create_timer(0.05, self.enforce_command_watchdog)
        self.create_timer(POSE_STEP_PERIOD_S, self.advance_named_pose)
        self.publish_state()
        self.publish_auto_close_status()
        self.get_logger().info(
            "Ready. Commands are relative, clipped to +/-20 ticks, speed=25. "
            f"Order: {', '.join(SERVO_NAMES)}"
        )

    def _load_gripper_control(
        self,
    ) -> tuple[GripperContactDetector, AutomaticCloseController]:
        try:
            values = json.loads(GRIPPER_CONFIG_PATH.read_text(encoding="utf-8"))
            config = ContactConfig.from_dict(values)
            auto_config = AutoCloseConfig.from_dict(values)
        except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Gripper contact detection disabled: invalid config: {exc}"
            )
            config = ContactConfig(
                enabled=False,
                closing_direction=1,
                minimum_load_raw=1,
                minimum_current_raw=1,
                minimum_position_error_ticks=1,
                empty_closed_position_ticks=0,
                minimum_object_gap_ticks=1,
                confirmation_samples=1,
                maximum_load_raw=2,
                maximum_current_raw=2,
            )
            auto_config = AutoCloseConfig(
                enabled=False,
                speed=10,
                acceleration=3,
                maximum_duration_s=5.0,
                endpoint_tolerance_ticks=3,
                relief_ticks=0,
            )
        if not config.enabled:
            self.get_logger().warning(
                "Gripper contact detection is disabled pending hardware calibration"
            )
        if not auto_config.enabled:
            self.get_logger().warning("Automatic gripper close is disabled")
        return GripperContactDetector(config), AutomaticCloseController(
            auto_config,
            closing_direction=config.closing_direction,
            empty_closed_position_ticks=config.empty_closed_position_ticks,
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

    def _read_gripper_feedback(self) -> dict[str, int]:
        position, _, result, error = self.packet.ReadPosSpeed(GRIPPER_ID)
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Gripper position read failed (result={result}, error={error})"
            )
        load_encoded, result, error = self.packet.read2ByteTxRx(
            GRIPPER_ID, STS_PRESENT_LOAD_L
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(f"Gripper load read failed (result={result}, error={error})")
        current_raw, result, error = self.packet.read2ByteTxRx(
            GRIPPER_ID, STS_PRESENT_CURRENT_L
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Gripper current read failed (result={result}, error={error})"
            )
        moving, result, error = self.packet.read1ByteTxRx(GRIPPER_ID, STS_MOVING)
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(f"Gripper moving read failed (result={result}, error={error})")
        return {
            "position_ticks": position,
            "target_ticks": self.targets[-1],
            # STS load is sign-magnitude with bit 10 as its direction bit.
            "load_raw": self.packet.sts_tohost(load_encoded, 10),
            # Keep current in device-native units until the exact installed
            # servo model and its scale have been verified.
            "current_raw": current_raw,
            "moving": int(moving),
        }

    def publish_gripper_feedback(self) -> None:
        """Publish passive feedback and an evidence-based contact state."""
        try:
            with self.lock:
                feedback = self._read_gripper_feedback()
            state = self.contact_detector.observe(
                position_ticks=feedback["position_ticks"],
                target_ticks=feedback["target_ticks"],
                load_raw=feedback["load_raw"],
                current_raw=feedback["current_raw"],
            )
            action = self.auto_close.observe(
                position_ticks=feedback["position_ticks"],
                contact_state=state,
                now_s=time.monotonic(),
            )
            feedback["position_error_ticks"] = abs(
                feedback["target_ticks"] - feedback["position_ticks"]
            )
            feedback["object_gap_ticks"] = (
                self.contact_detector.config.empty_closed_position_ticks
                - feedback["position_ticks"]
            ) * self.contact_detector.config.closing_direction
            feedback["contact_state"] = state.value
            feedback["automatic_close_state"] = self.auto_close.state.value
            self.gripper_telemetry_pub.publish(
                String(data=json.dumps(feedback, separators=(",", ":")))
            )
            self.gripper_contact_pub.publish(String(data=state.value))
            self.publish_auto_close_status()
            if action is not AutoCloseAction.NONE:
                self.finish_automatic_close(action, feedback["position_ticks"])
            elif state in (
                ContactState.GRIPPED,
                ContactState.PROTECTIVE_STOP,
            ) and self.gripper_closing_motion:
                # Contact protection applies to manual, recorded, and named-pose
                # gripper closure too. Arm-only motion remains possible while an
                # object is held.
                self.stop_motion("gripper contact detected")
        except Exception as exc:
            self.get_logger().error(f"Gripper feedback read failed: {exc}")

    def publish_auto_close_status(self) -> None:
        self.gripper_auto_status_pub.publish(String(data=self.auto_close.state.value))

    def finish_automatic_close(
        self, action: AutoCloseAction, position_ticks: int
    ) -> None:
        relief = (
            self.auto_close.config.relief_ticks
            if action is AutoCloseAction.HOLD_GRIPPED
            else 0
        )
        hold_target = (
            position_ticks - self.contact_detector.config.closing_direction * relief
        )
        minimum, maximum = self.joint_tick_limits[-1]
        hold_target = max(minimum, min(maximum, hold_target))
        try:
            with self.lock:
                result, error = self.packet.WritePosEx(
                    GRIPPER_ID,
                    hold_target,
                    self.auto_close.config.speed,
                    self.auto_close.config.acceleration,
                )
                if result != COMM_SUCCESS or error != 0:
                    raise RuntimeError(
                        f"automatic hold failed (result={result}, error={error})"
                    )
                self.targets[-1] = hold_target
            self.get_logger().info(
                f"Automatic close ended: {self.auto_close.state.value}; "
                f"position={position_ticks}; hold_target={hold_target}"
            )
        except Exception as exc:
            self.get_logger().error(f"Unable to hold after automatic close: {exc}")
            try:
                self.hold_current_positions()
            except Exception as stop_exc:
                self.get_logger().error(
                    f"CRITICAL: automatic close could not be stopped: {stop_exc}"
                )
        finally:
            self.motion_active = False
            self.gripper_closing_motion = False
            if action not in (
                AutoCloseAction.HOLD_GRIPPED,
                AutoCloseAction.HOLD_PROTECTIVE_STOP,
            ):
                self.contact_detector.stop()
            self.publish_auto_close_status()

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
            self.gripper_closing_motion = False
            self.contact_detector.stop()
            if self.auto_close.active:
                self.auto_close.stop()
                self.publish_auto_close_status()

    def enforce_command_watchdog(self) -> None:
        if (
            self.motion_active
            and self.pose_target is None
            and not self.auto_close.active
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
        if self.auto_close.active:
            self.stop_motion("named-pose command interrupted automatic close")
        with self.lock:
            # Ramp from the last target we send, not from live feedback. A
            # servo can legitimately lag a low-speed command by several ticks;
            # repeatedly commanding only a few ticks ahead of feedback can
            # leave it inside its position deadband indefinitely.
            self.targets = self.read_positions()
            gripper_delta = pose[-1] - self.targets[-1]
            if (
                gripper_delta * self.contact_detector.config.closing_direction > 0
                and self.contact_detector.state in (
                    ContactState.GRIPPED,
                    ContactState.PROTECTIVE_STOP,
                )
            ):
                self.get_logger().error(
                    "Named pose rejected: open the gripper before commanding further closure"
                )
                return
            self.contact_detector.command(gripper_delta)
            self.gripper_closing_motion = (
                gripper_delta * self.contact_detector.config.closing_direction > 0
            )
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
        if len(message.data) != len(SERVO_IDS):
            self.get_logger().error("Absolute pose must contain exactly six tick values")
            return
        try:
            requested = [int(value) for value in message.data]
        except (TypeError, ValueError):
            self.get_logger().error("Absolute pose contains a non-integer tick value")
            return
        # Feedback can differ from an EEPROM endpoint by a tick because of
        # servo resolution. Clamp recorded feedback rather than rejecting the
        # whole sample, while never issuing an out-of-limit target.
        pose = [
            max(minimum, min(value, maximum))
            for value, (minimum, maximum) in zip(requested, self.joint_tick_limits)
        ]
        if pose != requested:
            self.get_logger().warning(
                f"Clamped recorded pose to EEPROM limits: requested={requested}; target={pose}"
            )
        if self.auto_close.active:
            self.stop_motion("recorded pose interrupted automatic close")
        try:
            with self.lock:
                self.targets = self.read_positions()
                gripper_delta = pose[-1] - self.targets[-1]
                if (
                    gripper_delta * self.contact_detector.config.closing_direction > 0
                    and self.contact_detector.state in (
                        ContactState.GRIPPED,
                        ContactState.PROTECTIVE_STOP,
                    )
                ):
                    self.get_logger().error(
                        "Recorded pose rejected: open the gripper before further closure"
                    )
                    return
                self.contact_detector.command(gripper_delta)
                self.gripper_closing_motion = (
                    gripper_delta * self.contact_detector.config.closing_direction > 0
                )
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
                    self.gripper_closing_motion = False
                    self.contact_detector.stop()
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
        gripper_is_closing = (
            requested[-1] * self.contact_detector.config.closing_direction > 0
        )
        if gripper_is_closing and self.contact_detector.state in (
            ContactState.GRIPPED,
            ContactState.PROTECTIVE_STOP,
        ):
            self.get_logger().error(
                "Closing command blocked: open the gripper to clear latched contact"
            )
            return
        if self.auto_close.active:
            self.stop_motion("manual command interrupted automatic close")
        try:
            with self.lock:
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
            # Track explicit operator intent even if the gripper is already at
            # a travel limit. An opening request must always clear a latched
            # contact state.
            self.contact_detector.command(requested[-1])
            self.gripper_closing_motion = gripper_is_closing
            if requested[-1] and not gripper_is_closing:
                self.auto_close.reset()
                self.publish_auto_close_status()
            self.motion_active = True
            self.last_motion_command_s = time.monotonic()
            self.get_logger().info(f"Applied capped delta {requested}; targets {targets}")
            self.publish_state()
        except Exception as exc:
            self.get_logger().error(f"Command rejected after read/write failure: {exc}")

    def gripper_command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command == "stop":
            self.stop_motion("automatic gripper stop command")
            self.publish_state()
            return
        if command != "close":
            self.get_logger().error("Unknown gripper command; use close or stop")
            return
        if not self.contact_detector.config.enabled or not self.auto_close.config.enabled:
            self.get_logger().error("Automatic close is disabled by gripper configuration")
            return
        if self.contact_detector.state in (
            ContactState.GRIPPED,
            ContactState.PROTECTIVE_STOP,
        ):
            self.get_logger().error(
                "Automatic close rejected: open the gripper to clear latched contact"
            )
            return
        if self.motion_active:
            self.stop_motion("automatic gripper close superseded active motion")
        endpoint = self.contact_detector.config.empty_closed_position_ticks
        minimum, maximum = self.joint_tick_limits[-1]
        if not minimum <= endpoint <= maximum:
            self.get_logger().error(
                f"Automatic close endpoint {endpoint} violates EEPROM limits {minimum}-{maximum}"
            )
            return
        try:
            at_endpoint = False
            with self.lock:
                self.targets = self.read_positions()
                current_position = self.targets[-1]
                remaining_ticks = (
                    endpoint - current_position
                ) * self.contact_detector.config.closing_direction
                if remaining_ticks <= self.auto_close.config.endpoint_tolerance_ticks:
                    at_endpoint = True
                else:
                    result, error = self.packet.WritePosEx(
                        GRIPPER_ID,
                        endpoint,
                        self.auto_close.config.speed,
                        self.auto_close.config.acceleration,
                    )
                    if result != COMM_SUCCESS or error != 0:
                        raise RuntimeError(
                            f"automatic close write failed (result={result}, error={error})"
                        )
                    self.targets[-1] = endpoint
            if at_endpoint:
                self.auto_close.start(time.monotonic())
                action = self.auto_close.observe(
                    position_ticks=current_position,
                    contact_state=self.contact_detector.state,
                    now_s=time.monotonic(),
                )
                self.finish_automatic_close(action, current_position)
                return
            self.contact_detector.command(self.contact_detector.config.closing_direction)
            self.auto_close.start(time.monotonic())
            self.motion_active = True
            self.gripper_closing_motion = True
            self.publish_auto_close_status()
            self.get_logger().info(
                f"Automatic close started: endpoint={endpoint}, "
                f"speed={self.auto_close.config.speed}, "
                f"timeout={self.auto_close.config.maximum_duration_s}s"
            )
        except Exception as exc:
            self.get_logger().error(f"Automatic close rejected after read/write failure: {exc}")

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
