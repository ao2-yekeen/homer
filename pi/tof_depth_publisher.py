#!/usr/bin/env python3
"""Publish CSI Arducam ToF depth frames as ROS ``sensor_msgs/Image``.

The camera SDK supplies a 240x180 float32 depth array in millimetres.  This
node preserves those values on ``/tof_camera/depth/image_raw`` using the ROS
``32FC1`` encoding; it does not command any robot hardware.
"""

from __future__ import annotations

import json
from pathlib import Path

import ArducamDepthCamera as ac
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from tof_camera_metadata import read_camera_metadata


def depth_image(depth_mm: np.ndarray, stamp, frame_id: str) -> Image:
    """Convert a two-dimensional float32 millimetre image into a ROS message."""
    if depth_mm.ndim != 2:
        raise ValueError("ToF depth frame must be a two-dimensional array")
    depth_mm = np.ascontiguousarray(depth_mm, dtype=np.float32)
    height, width = depth_mm.shape
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = height
    message.width = width
    message.encoding = "32FC1"
    message.is_bigendian = False
    message.step = width * depth_mm.dtype.itemsize
    message.data = depth_mm.tobytes()
    return message


class TofDepthPublisher(Node):
    def __init__(self) -> None:
        super().__init__("tof_depth_publisher")
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("frame_id", "tof_camera")
        self.declare_parameter("metadata_path", str(Path.home() / '.local/state/homer/tof-camera-metadata.json'))
        fps = float(self.get_parameter("fps").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        if fps <= 0:
            raise ValueError("fps must be positive")

        self.publisher = self.create_publisher(Image, "/tof_camera/depth/image_raw", 5)
        self.amplitude_publisher = self.create_publisher(Image, "/tof_camera/amplitude/image_raw", 5)
        self.confidence_publisher = self.create_publisher(Image, "/tof_camera/confidence/image_raw", 5)
        self.camera = ac.ArducamCamera()
        result = self.camera.open(ac.Connection.CSI, 0)
        if result != 0:
            raise RuntimeError(f"Cannot open CSI ToF camera (SDK code {result})")
        result = self.camera.start(ac.FrameType.DEPTH)
        if result != 0:
            self.camera.close()
            raise RuntimeError(f"Cannot start CSI ToF depth stream (SDK code {result})")
        self.get_logger().info("CSI ToF camera opened")
        # Read the device already owned by this process; never open a second owner.
        try:
            camera_metadata = read_camera_metadata(self.camera, ac.Control)
        except Exception as error:
            camera_metadata = dict(factory_intrinsics_available=False, error=str(error),
                                   physical_accuracy_verified=False, motion_authority=False)
            self.get_logger().warning('Factory metadata unavailable: ' + str(error))
        camera_metadata['publisher_start_stamp'] = self.get_clock().now().nanoseconds * 1e-9
        try:
            destination = Path(str(self.get_parameter('metadata_path').value))
            destination.parent.mkdir(parents=True, exist_ok=True)
            partial = destination.with_suffix('.tmp')
            partial.write_text(json.dumps(camera_metadata, indent=2, allow_nan=False) + '\n')
            partial.replace(destination)
            self.get_logger().info('Camera metadata saved: ' + str(destination))
        except OSError as error:
            self.get_logger().warning('Cannot save factory metadata: ' + str(error))
        self.create_timer(1.0 / fps, self.publish_frame)

    def publish_frame(self) -> None:
        frame = self.camera.requestFrame(1000)
        if frame is None:
            self.get_logger().warning("ToF frame request timed out")
            return
        acquisition_stamp = self.get_clock().now().to_msg()
        try:
            # The SDK owns the frame buffer, so copy before releasing it.
            depth_mm = frame.depth_data.copy()
            amplitude = frame.amplitude_data.copy()
            confidence = frame.confidence_data.copy()
        finally:
            self.camera.releaseFrame(frame)
        self.publisher.publish(depth_image(depth_mm, acquisition_stamp, self.frame_id))
        # Raw SDK values, not millimetres. Shared stamp identifies the same exposure.
        self.amplitude_publisher.publish(depth_image(amplitude, acquisition_stamp, self.frame_id))
        self.confidence_publisher.publish(depth_image(confidence, acquisition_stamp, self.frame_id))

    def destroy_node(self) -> bool:
        self.camera.stop()
        self.camera.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: TofDepthPublisher | None = None
    try:
        node = TofDepthPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
