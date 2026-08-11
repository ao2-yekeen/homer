#!/usr/bin/env python3
"""Watches /joy for two gamepad buttons and publishes /set_autonomous
(std_msgs/Bool) on rising edge, so the ESP32's Teleop/Auto mode can be
switched from the controller instead of only over BLE from a phone.

Button indices default to a guess and will need adjusting once the actual
controller's index-to-button mapping is known (check with `ros2 topic echo
/joy` while pressing buttons).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class ModeToggle(Node):
    def __init__(self):
        super().__init__('mode_toggle')
        self.declare_parameter('autonomous_button', 1)
        self.declare_parameter('teleop_button', 0)
        self.autonomous_button = self.get_parameter('autonomous_button').value
        self.teleop_button = self.get_parameter('teleop_button').value
        self.pub = self.create_publisher(Bool, 'set_autonomous', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.prev_buttons = []

    def joy_callback(self, msg):
        buttons = msg.buttons
        if self.prev_buttons and len(self.prev_buttons) == len(buttons):
            for idx, want_autonomous in (
                (self.autonomous_button, True),
                (self.teleop_button, False),
            ):
                if idx < len(buttons) and buttons[idx] == 1 and self.prev_buttons[idx] == 0:
                    out = Bool()
                    out.data = want_autonomous
                    self.pub.publish(out)
                    self.get_logger().info(
                        'Mode -> %s' % ('AUTONOMOUS' if want_autonomous else 'TELEOP')
                    )
        self.prev_buttons = list(buttons)


def main():
    rclpy.init()
    node = ModeToggle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
