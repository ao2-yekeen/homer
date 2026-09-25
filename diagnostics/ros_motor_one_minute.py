#!/usr/bin/env python3
"""Bounded lifted-wheel test for the temporary motor-only micro-ROS image."""

import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


TEST_SECONDS = 60.0
COMMAND_PERIOD_SECONDS = 0.1
FORWARD_COMMAND = 0.7


def main() -> None:
    stopped = False

    def request_stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGHUP, request_stop)

    rclpy.init()
    node = Node("motor_one_minute_test")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    zero = Twist()
    forward = Twist()
    forward.linear.x = FORWARD_COMMAND

    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not stopped:
            subscribers = node.get_subscriptions_info_by_topic("/cmd_vel")
            if len(subscribers) == 1 and subscribers[0].node_name == "motor_test":
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("Expected exactly one /cmd_vel subscriber: motor_test")

        other_publishers = [
            endpoint
            for endpoint in node.get_publishers_info_by_topic("/cmd_vel")
            if endpoint.node_name != node.get_name()
        ]
        if other_publishers:
            raise RuntimeError("Another /cmd_vel publisher is active")

        for _ in range(10):
            publisher.publish(zero)
            time.sleep(COMMAND_PERIOD_SECONDS)

        print("Sending forward commands for up to 60 seconds", flush=True)
        end_time = time.monotonic() + TEST_SECONDS
        while time.monotonic() < end_time and not stopped:
            publisher.publish(forward)
            time.sleep(COMMAND_PERIOD_SECONDS)
    finally:
        print("Sending zero commands", flush=True)
        for _ in range(30):
            publisher.publish(zero)
            time.sleep(COMMAND_PERIOD_SECONDS)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
