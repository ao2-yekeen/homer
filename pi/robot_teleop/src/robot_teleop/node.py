#!/usr/bin/env python3
"""ROS adapter that coordinates safe base and SO-ARM gamepad teleoperation."""
from __future__ import annotations

from time import monotonic

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16MultiArray, Int32

from robot_teleop.control_model import (
    ArmController,
    BaseController,
    GamepadLayout,
    GamepadSnapshot,
    NeckController,
    ModeSelector,
    RobotMode,
)


class RobotTeleopNode(Node):
    """Owns the gamepad state and publishes commands for each robot subsystem."""

    def __init__(self) -> None:
        super().__init__("robot_teleop")
        self.layout = GamepadLayout()
        self.mode_selector = ModeSelector(self.layout)
        self.base_controller = BaseController(self.layout)
        self.arm_controller = ArmController(self.layout)
        self.neck_controller = NeckController(self.layout)
        self.snapshot = GamepadSnapshot()
        self.last_update = monotonic()

        self.base_publisher = self.create_publisher(Twist, "/teleop/cmd_vel", 10)
        self.arm_publisher = self.create_publisher(
            Int16MultiArray, "/soarm/command_delta_ticks", 10
        )
        self.neck_publisher = self.create_publisher(Int32, "/teleop/neck_angle", 10)
        self.mode_publisher = self.create_publisher(Bool, "/set_autonomous", 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_timer(0.05, self._publish_commands)  # 20 Hz dead-man heartbeat
        self.get_logger().info(
            "Ready. Select TELEOP before use; hold L1 for base or R1 for arm. "
            "Arm mapping must be verified before service enablement."
        )

    def _on_joy(self, message: Joy) -> None:
        self.snapshot = GamepadSnapshot(tuple(message.axes), tuple(message.buttons))
        changed_mode = self.mode_selector.update(self.snapshot)
        if changed_mode is not None:
            self.mode_publisher.publish(Bool(data=changed_mode is RobotMode.AUTONOMOUS))
            self.get_logger().info(f"Mode -> {changed_mode.value.upper()}")

    def _publish_commands(self) -> None:
        now = monotonic()
        elapsed_seconds = min(0.1, max(0.0, now - self.last_update))
        self.last_update = now

        base = self.base_controller.command(self.snapshot, self.mode_selector.mode)
        self.base_publisher.publish(
            Twist(
                linear=Vector3(x=base.linear),
                angular=Vector3(z=base.angular),
            )
        )

        arm_deltas = self.arm_controller.command(
            self.snapshot, self.mode_selector.mode, elapsed_seconds
        )
        if any(arm_deltas):
            self.arm_publisher.publish(Int16MultiArray(data=list(arm_deltas)))

        neck_angle = self.neck_controller.command(
            self.snapshot, self.mode_selector.mode, elapsed_seconds
        )
        if neck_angle is not None:
            self.neck_publisher.publish(Int32(data=neck_angle))


def main() -> None:
    rclpy.init()
    node = RobotTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
