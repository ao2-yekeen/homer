#!/usr/bin/env python3
"""Safe all-robot gamepad teleoperation."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Int32


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')
        # Aurora receiver's Xbox layout: A/B/X/Y = 0/1/2/3 and LT is axis 2.
        # Face buttons directly command the base; both sticks remain for arm.
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 0.4)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('neck_speed_dps', 20.0)
        self.declare_parameter('neck_min_angle', 80)
        self.declare_parameter('neck_max_angle', 150)
        self.declare_parameter('publish_rate_hz', 20.0)

        get = lambda name: self.get_parameter(name).value
        self.max_linear = get('max_linear')
        self.max_angular = get('max_angular')
        self.deadzone = get('deadzone')
        self.neck_speed_dps = get('neck_speed_dps')
        self.neck_min_angle = get('neck_min_angle')
        self.neck_max_angle = get('neck_max_angle')
        rate_hz = get('publish_rate_hz')

        self.drive_pub = self.create_publisher(Twist, 'teleop/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Int16MultiArray, 'soarm/command_delta_ticks', 10)
        self.neck_pub = self.create_publisher(Int32, 'teleop/neck_angle', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.axes = []
        self.buttons = []
        self.arm_residual = [0.0] * 6
        self.neck_angle = 90.0
        self.last_tick_ns = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / rate_hz, self.publish_commands)
        self.get_logger().info('Y/A/X/B drive; hold LT for arm and neck controls.')

    @staticmethod
    def value(values, index):
        return values[index] if 0 <= index < len(values) else 0.0

    def joy_callback(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        # No gamepad mode switch: buttons only command labelled robot actions.

    def shaped_axis(self, axis):
        value = self.value(self.axes, axis)
        if abs(value) <= self.deadzone:
            return 0.0
        return math.copysign((abs(value) - self.deadzone) / (1.0 - self.deadzone), value)

    def publish_commands(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = max(0.0, (now_ns - self.last_tick_ns) / 1e9)
        self.last_tick_ns = now_ns

        # Y forward, A reverse, X left, B right. Releasing all buttons sends zero.
        drive = Twist()
        drive.linear.x = (self.value(self.buttons, 3) - self.value(self.buttons, 0)) * self.max_linear
        drive.angular.z = (self.value(self.buttons, 2) - self.value(self.buttons, 1)) * self.max_angular
        self.drive_pub.publish(drive)

        # LT is the arm dead-man. On this controller it rests near +1 and
        # becomes negative when held. SO-ARM order follows its live bridge.
        arm_enabled = self.value(self.axes, 2) < 0.0
        if not arm_enabled:
            self.arm_residual = [0.0] * 6
            return
        arm_axes = (self.shaped_axis(0), self.shaped_axis(1), self.shaped_axis(3),
                    self.shaped_axis(4), self.shaped_axis(6), self.shaped_axis(7))
        arm_deltas = []
        for index, axis in enumerate(arm_axes):
            total = self.arm_residual[index] + axis * 10.0 * elapsed_s
            delta = max(-2, min(2, int(total)))
            self.arm_residual[index] = total - delta
            arm_deltas.append(delta)
        if any(arm_deltas):
            self.arm_pub.publish(Int16MultiArray(data=arm_deltas))

        # LB/RB lower/raise neck while LT is held.
        neck_axis = self.value(self.buttons, 5) - self.value(self.buttons, 4)
        if neck_axis:
            self.neck_angle += neck_axis * self.neck_speed_dps * elapsed_s
            self.neck_angle = min(self.neck_max_angle, max(self.neck_min_angle, self.neck_angle))
            neck = Int32()
            neck.data = round(self.neck_angle)
            self.neck_pub.publish(neck)


def main():
    rclpy.init()
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
