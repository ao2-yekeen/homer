#!/usr/bin/env python3
"""ROS 2 adapter for the ESP32 HOM1 serial controller."""

from __future__ import annotations

import math
import os
import time
from pathlib import Path

import rclpy
import serial
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32

from autonomous_lease import AutonomousLease
from esp32_port import candidates
from serial_protocol import Decoder, encode


class Esp32SerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp32_serial_bridge")
        self.port_link = Path(os.environ.get("ESP32_PORT_LINK", f"/run/user/{os.getuid()}/mobile-robot-devices/esp32"))
        self.port: serial.Serial | None = None
        self.port_name = ""
        self.decoder = Decoder()
        self.connected = False
        self.boot_id: str | None = None
        self.opened_at = 0.0
        self.last_rx = 0.0
        self.next_attempt = 0.0
        self.sequence = 0
        self.mode_ready = False
        self.override_active = False
        self.autonomous_mode = False
        self.approach_mode = self.declare_parameter("approach_mode", False).value
        self.lease = AutonomousLease()
        self.reconnects = 0
        self.candidate_list: list[str] = []
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_pub = self.create_publisher(Twist, "/drive/status", status_qos)
        link_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.link_pub = self.create_publisher(Bool, "/drive/link", link_qos)
        self.link_pub.publish(Bool(data=False))
        self.authority_pub = self.create_publisher(Bool, "/drive/approach_authority", 1)
        if self.approach_mode:
            # No /cmd_vel subscription exists in this profile.
            self.create_subscription(TwistStamped, "/approach_object/drive", self._leased_autonomous, 1)
        else:
            self.create_subscription(Twist, "/cmd_vel", self._autonomous, 10)
        self.create_subscription(Twist, "/teleop/cmd_vel", self._teleop, 10)
        self.create_subscription(Bool, "/set_autonomous", self._mode, 10)
        self.create_subscription(Int32, "/mode_override", self._override, 10)
        self.create_subscription(Int32, "/teleop/neck_angle", self._neck, 10)
        self.create_timer(0.05, self._tick)

    def _send(self, kind: str, *fields: object) -> None:
        if not self.connected or self.port is None:
            return
        try:
            frame = encode(kind, self.sequence, *fields)
            self.sequence = (self.sequence + 1) & 0xFFFF
            if self.port.write(frame) != len(frame):
                raise serial.SerialTimeoutException("partial serial write")
        except (OSError, serial.SerialException, ValueError) as error:
            self._disconnect(str(error))

    def _drive(self, kind: str, message: Twist) -> None:
        linear, angular = message.linear.x, message.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            return
        if abs(linear) > 5 or abs(angular) > 5:
            return
        if kind == "T" and (not self.mode_ready or self.override_active):
            return
        if kind == "A" and not (self.mode_ready or self.override_active):
            return
        self._send(kind, f"{linear:.4f}", f"{angular:.4f}")

    def _autonomous(self, message: Twist) -> None:
        self._drive("A", message)

    def _leased_autonomous(self, message: TwistStamped) -> None:
        if not self.connected or not self.mode_ready or self.override_active:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.lease.accept(stamp, self.get_clock().now().nanoseconds * 1e-9,
                             time.monotonic(), message.twist.linear.x, message.twist.angular.z):
            self._drive("A", message.twist)

    def _teleop(self, message: Twist) -> None:
        self._drive("T", message)

    def _mode(self, message: Bool) -> None:
        if self.connected:
            self._send("M", int(message.data))
            self.mode_ready = True
            self.autonomous_mode = bool(message.data)
            self.lease.mode(self.autonomous_mode and not self.override_active, time.monotonic())

    def _override(self, message: Int32) -> None:
        if self.connected and -1 <= message.data <= 2:
            self._send("V", message.data)
            self.override_active = message.data >= 0
            if self.approach_mode and self.override_active:
                self._send("A", "0.0000", "0.0000")
            self.lease.mode(self.autonomous_mode and not self.override_active, time.monotonic())

    def _neck(self, message: Int32) -> None:
        if self.connected and 80 <= message.data <= 150:
            self._send("N", message.data)

    def _disconnect(self, reason: str) -> None:
        if self.connected and self.port is not None:
            try:
                self.port.write(encode("A", self.sequence, "0.0000", "0.0000"))
            except (OSError, serial.SerialException):
                pass
        if self.connected and rclpy.ok():
            self.get_logger().warning(f"ESP32 disconnected: {reason}; frame errors={self.decoder.errors}")
            self.link_pub.publish(Bool(data=False))
        self.connected = False
        self.autonomous_mode = False
        self.lease.mode(False, time.monotonic())
        self.boot_id = None
        self.mode_ready = False
        self.override_active = False
        if self.port is not None:
            try:
                self.port.close()
            except OSError:
                pass
        self.port = None
        self.decoder = Decoder()
        self.next_attempt = time.monotonic() + 0.25

    def _open_next(self, now: float) -> None:
        if not self.candidate_list:
            self.candidate_list = candidates(self.port_link)
            if not self.candidate_list:
                self.next_attempt = now + 1.0
                return
        name = self.candidate_list.pop(0)
        try:
            self.port = serial.Serial(name, baudrate=115200, timeout=0, write_timeout=0, exclusive=True)
            self.port.reset_input_buffer()
            self.port.reset_output_buffer()
            self.port_name = name
            self.opened_at = now
            self.decoder = Decoder()
        except (OSError, serial.SerialException) as error:
            self.get_logger().warning(f"Cannot open {name}: {error}")
            self.next_attempt = now + 0.1

    def _ready(self, boot_id: str) -> None:
        if self.connected and self.boot_id == boot_id:
            return
        self.connected = True
        self.autonomous_mode = False
        self.lease.mode(False, time.monotonic())
        self.boot_id = boot_id
        self.mode_ready = False
        self.override_active = False
        self.reconnects += 1
        self.get_logger().info(f"ESP32 ready on {self.port_name}; boot={boot_id}; connections={self.reconnects}")
        self.link_pub.publish(Bool(data=True))
        self._send("S")
        self._send("V", -1)

    def _handle(self, kind: str, fields: list[str]) -> None:
        try:
            if kind == "R" and len(fields) == 1:
                self._ready(str(int(fields[0])))
            elif self.connected and kind == "O" and len(fields) == 6:
                _, x, y, heading, linear, angular = (float(value) for value in fields)
                if not all(math.isfinite(value) for value in (x, y, heading, linear, angular)):
                    raise ValueError("nonfinite odometry")
                message = Odometry()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = "odom"
                message.child_frame_id = "base_link"
                message.pose.pose.position.x = x
                message.pose.pose.position.y = y
                message.pose.pose.orientation.z = math.sin(heading / 2)
                message.pose.pose.orientation.w = math.cos(heading / 2)
                message.twist.twist.linear.x = linear
                message.twist.twist.angular.z = angular
                self.odom_pub.publish(message)
            elif self.connected and kind == "D" and len(fields) == 7:
                _, linear, left, right, angular, mode, age = (float(value) for value in fields)
                if not all(math.isfinite(value) for value in (linear, left, right, angular, mode, age)):
                    raise ValueError("nonfinite status")
                message = Twist()
                message.linear.x = linear
                message.linear.y = left
                message.linear.z = right
                message.angular.x = mode
                message.angular.y = age
                message.angular.z = angular
                self.status_pub.publish(message)
        except ValueError:
            self.decoder.errors += 1

    def _tick(self) -> None:
        now = time.monotonic()
        self.authority_pub.publish(Bool(data=bool(self.approach_mode and self.connected and self.mode_ready and self.autonomous_mode and not self.override_active and not self.lease.expired)))
        if self.approach_mode and self.connected and self.lease.tick(now):
            self._send("A", "0.0000", "0.0000")
        if self.port is None:
            if now >= self.next_attempt:
                self._open_next(now)
            return
        if not self.connected and now - self.opened_at > 2.5:
            self._disconnect("no HOM1 greeting")
            return
        if self.connected and now - self.last_rx > 2.0:
            self._disconnect("telemetry timeout")
            return
        try:
            data = self.port.read(min(self.port.in_waiting, 1024))
        except (OSError, serial.SerialException) as error:
            self._disconnect(str(error))
            return
        if data:
            for kind, _, fields in self.decoder.feed(data):
                self.last_rx = now
                self._handle(kind, fields)


def main() -> None:
    rclpy.init()
    node = Esp32SerialBridge()
    try:
        rclpy.spin(node)
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node._disconnect("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
