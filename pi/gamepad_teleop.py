#!/usr/bin/env python3
"""Safe all-robot gamepad teleoperation."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16MultiArray, Int32


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
        self.declare_parameter('autonomous_exit_hold_s', 1.5)

        get = lambda name: self.get_parameter(name).value
        self.max_linear = get('max_linear')
        self.max_angular = get('max_angular')
        self.deadzone = get('deadzone')
        self.neck_speed_dps = get('neck_speed_dps')
        self.neck_min_angle = get('neck_min_angle')
        self.neck_max_angle = get('neck_max_angle')
        self.autonomous_exit_hold_s = get('autonomous_exit_hold_s')
        rate_hz = get('publish_rate_hz')

        self.drive_pub = self.create_publisher(Twist, 'teleop/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Int16MultiArray, 'soarm/command_delta_ticks', 10)
        self.neck_pub = self.create_publisher(Int32, 'teleop/neck_angle', 10)
        self.mode_pub = self.create_publisher(Bool, 'set_autonomous', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.axes = []
        self.buttons = []
        self.previous_buttons = []
        self.arm_residual = [0.0] * 6
        self.arm_command_active = False
        self.neck_angle = 90.0
        self.autonomous = True
        self.b_hold_started_ns = None
        self.last_tick_ns = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / rate_hz, self.publish_commands)
        self.get_logger().info('Autonomous: press A for Teleop. Hold B for 1.5 s to return to Autonomous.')

    @staticmethod
    def value(values, index):
        return values[index] if 0 <= index < len(values) else 0.0

    def joy_callback(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

    def set_autonomous(self, enabled):
        self.autonomous = enabled
        message = Bool()
        message.data = enabled
        self.mode_pub.publish(message)
        self.get_logger().info(f"Mode -> {'AUTONOMOUS' if enabled else 'TELEOP'}")

    def shaped_axis(self, axis):
        value = self.value(self.axes, axis)
        if abs(value) <= self.deadzone:
            return 0.0
        return math.copysign((abs(value) - self.deadzone) / (1.0 - self.deadzone), value)

    def stop_arm(self):
        """Tell the bridge to hold at its current measured position once."""
        if self.arm_command_active:
            self.arm_pub.publish(Int16MultiArray(data=[0] * 6))
            self.arm_command_active = False

    def publish_commands(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = max(0.0, (now_ns - self.last_tick_ns) / 1e9)
        self.last_tick_ns = now_ns

        a_pressed = bool(self.value(self.buttons, 0))
        b_pressed = bool(self.value(self.buttons, 1))
        a_was_pressed = bool(self.value(self.previous_buttons, 0))

        # A enters Teleop from the safe Autonomous default. A short B remains
        # turn-right; holding B returns to Autonomous without a vague mode key.
        if self.autonomous and a_pressed and not a_was_pressed:
            self.set_autonomous(False)
            self.previous_buttons = self.buttons.copy()
            return
        if not self.autonomous and b_pressed:
            if self.b_hold_started_ns is None:
                self.b_hold_started_ns = now_ns
            elif (now_ns - self.b_hold_started_ns) / 1e9 >= self.autonomous_exit_hold_s:
                self.set_autonomous(True)
                self.b_hold_started_ns = None
                self.previous_buttons = self.buttons.copy()
                return
        else:
            self.b_hold_started_ns = None

        self.previous_buttons = self.buttons.copy()
        mode_message = Bool()
        mode_message.data = self.autonomous
        self.mode_pub.publish(mode_message)
        if self.autonomous:
            # No base, arm, or neck command is emitted until A enters Teleop.
            return

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
            self.stop_arm()
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
            self.arm_command_active = True
        elif not any(arm_axes):
            self.stop_arm()

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
