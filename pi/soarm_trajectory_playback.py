#!/usr/bin/env python3
"""Replay recorded SO-ARM joint-feedback poses through the guarded bridge."""

import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from std_msgs.msg import Int16MultiArray, String


STATE_TOPIC = "/soarm/state_ticks"
POSE_TOPIC = "/soarm/command_pose_ticks"
NAMED_POSE_TOPIC = "/soarm/command_named_pose"
JOINT_COUNT = 6


def recorded_states(bag_path: str) -> list[tuple[int, list[int]]]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    states: list[tuple[int, list[int]]] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic != STATE_TOPIC:
            continue
        message = deserialize_message(serialized, Int16MultiArray)
        pose = [int(value) for value in message.data]
        if len(pose) != JOINT_COUNT or any(value < 0 or value > 4095 for value in pose):
            raise ValueError(f"Invalid recorded state: {pose}")
        states.append((timestamp_ns, pose))
    if not states:
        raise ValueError(f"No {STATE_TOPIC} messages found in {bag_path}")
    return states


def trim_leading_idle(
    states: list[tuple[int, list[int]]], threshold_ticks: int
) -> list[tuple[int, list[int]]]:
    """Drop initial samples that are only held-pose feedback noise."""
    if threshold_ticks == 0:
        return states
    baseline = states[0][1]
    for index, (_, pose) in enumerate(states):
        if max(abs(value - initial) for value, initial in zip(pose, baseline)) >= threshold_ticks:
            return states[index:]
    return states


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="ros2 bag directory containing manual state recording")
    parser.add_argument("--rate", type=float, default=1.0, help="timeline multiplier (default: 1.0)")
    parser.add_argument(
        "--leading-idle-threshold",
        type=int,
        default=10,
        help="drop leading samples below this pose change in ticks; use 0 to keep all (default: 10)",
    )
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")
    if args.leading_idle_threshold < 0:
        parser.error("--leading-idle-threshold cannot be negative")

    states = trim_leading_idle(recorded_states(args.bag_path), args.leading_idle_threshold)
    rclpy.init()
    node = Node("soarm_trajectory_playback")
    pose_publisher = node.create_publisher(Int16MultiArray, POSE_TOPIC, 10)
    stop_publisher = node.create_publisher(String, NAMED_POSE_TOPIC, 10)
    stopped = False

    def stop_playback(*_: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_playback)
    signal.signal(signal.SIGTERM, stop_playback)
    try:
        # Give ROS discovery a moment so the first recorded pose is not lost.
        deadline = time.monotonic() + 1.0
        while not stopped and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        first_time = states[0][0]
        started = time.monotonic()
        for timestamp_ns, pose in states:
            due = started + ((timestamp_ns - first_time) / 1_000_000_000) / args.rate
            while not stopped and time.monotonic() < due:
                rclpy.spin_once(node, timeout_sec=min(0.05, due - time.monotonic()))
            if stopped:
                break
            pose_publisher.publish(Int16MultiArray(data=pose))
        node.get_logger().info("Trajectory replay stopped" if stopped else "Trajectory replay complete")
    finally:
        # A stopped or completed playback holds the live joint positions.
        stop_publisher.publish(String(data="stop"))
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
