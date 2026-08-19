#!/usr/bin/env python3
"""Safe all-robot gamepad teleoperation."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')
        # Default PS3 joystick indices. All controls are ROS parameters, so a
        # different controller can be mapped without editing this program.
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('neck_axis', 7)
        self.declare_parameter('drive_enable_button', 10)  # L1 on common PS3 mappings
        self.declare_parameter('teleop_button', 0)         # Select
        self.declare_parameter('autonomous_button', 1)     # L3
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('neck_speed_dps', 35.0)
        self.declare_parameter('neck_min_angle', 80)
        self.declare_parameter('neck_max_angle', 150)
        self.declare_parameter('publish_rate_hz', 20.0)

        get = lambda name: self.get_parameter(name).value
        self.linear_axis = get('linear_axis')
        self.angular_axis = get('angular_axis')
        self.neck_axis = get('neck_axis')
        self.drive_enable_button = get('drive_enable_button')
        self.teleop_button = get('teleop_button')
        self.autonomous_button = get('autonomous_button')
        self.max_linear = get('max_linear')
        self.max_angular = get('max_angular')
        self.deadzone = get('deadzone')
        self.neck_speed_dps = get('neck_speed_dps')
        self.neck_min_angle = get('neck_min_angle')
        self.neck_max_angle = get('neck_max_angle')
        rate_hz = get('publish_rate_hz')

        self.drive_pub = self.create_publisher(Twist, 'teleop/cmd_vel', 10)
        self.neck_pub = self.create_publisher(Int32, 'teleop/neck_angle', 10)
        self.mode_pub = self.create_publisher(Bool, 'set_autonomous', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.axes = []
        self.buttons = []
        self.prev_buttons = []
        self.neck_angle = 90.0
        self.last_tick_ns = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / rate_hz, self.publish_commands)
        self.get_logger().info('Gamepad ready: hold L1 to drive; D-pad up/down moves neck.')

    @staticmethod
    def value(values, index):
        return values[index] if 0 <= index < len(values) else 0.0

    def joy_callback(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        if len(self.prev_buttons) == len(self.buttons):
            for button, autonomous in ((self.teleop_button, False),
                                       (self.autonomous_button, True)):
                if self.value(self.buttons, button) and not self.value(self.prev_buttons, button):
                    out = Bool()
                    out.data = autonomous
                    self.mode_pub.publish(out)
                    mode_name = 'AUTONOMOUS' if autonomous else 'TELEOP'
                    self.get_logger().info(f'Mode -> {mode_name}')
        self.prev_buttons = self.buttons.copy()

    def shaped_axis(self, axis):
        value = self.value(self.axes, axis)
        if abs(value) <= self.deadzone:
            return 0.0
        return math.copysign((abs(value) - self.deadzone) / (1.0 - self.deadzone), value)

    def publish_commands(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = max(0.0, (now_ns - self.last_tick_ns) / 1e9)
        self.last_tick_ns = now_ns

        # Publish at a fixed rate, including zero, so releasing L1 stops the
        # base and a stale /joy stream times out on the ESP32.
        drive = Twist()
        if self.value(self.buttons, self.drive_enable_button):
            drive.linear.x = self.shaped_axis(self.linear_axis) * self.max_linear
            drive.angular.z = self.shaped_axis(self.angular_axis) * self.max_angular
        self.drive_pub.publish(drive)

        neck_axis = self.shaped_axis(self.neck_axis)
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
