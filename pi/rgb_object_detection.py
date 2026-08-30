#!/usr/bin/env python3
"""Publish a Logitech RGB stream and detect one deliberately coloured object.

This is the Week 2 bootstrap perception path: it deliberately has no depth
dependency and makes no arm or base command.  Detection is a transparent HSV
colour threshold intended for a saturated yellow first target on the raised
platform.  It is not a general object detector.

Topics:
  /rgb_camera/image_raw       sensor_msgs/Image (BGR8)
  /rgb_camera/detection       std_msgs/String (one JSON object per frame)
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass(frozen=True)
class Detection:
    """A single 2-D detection in camera pixel coordinates."""

    detected: bool
    confidence: float
    bbox_xywh: tuple[int, int, int, int] | None
    center_uv: tuple[int, int] | None
    area_px: int

    def as_dict(self, width: int, height: int) -> dict[str, object]:
        return {
            "detected": self.detected,
            "confidence": round(self.confidence, 3),
            "bbox_xywh": list(self.bbox_xywh) if self.bbox_xywh else None,
            "center_uv": list(self.center_uv) if self.center_uv else None,
            "area_px": self.area_px,
            "image_width": width,
            "image_height": height,
        }


def detect_yellow_object(
    frame_bgr: np.ndarray, lower_hsv: tuple[int, int, int], upper_hsv: tuple[int, int, int], min_area_px: int
) -> Detection:
    """Return the largest connected HSV region, or an explicit no-detection."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_hsv, dtype=np.uint8), np.array(upper_hsv, dtype=np.uint8))
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return Detection(False, 0.0, None, None, 0)

    contour = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(contour))
    if area < min_area_px:
        return Detection(False, 0.0, None, None, area)
    x, y, width, height = cv2.boundingRect(contour)
    # Confidence is intentionally an interpretable size score, not a learned
    # probability: one min-area target scores 0.5 and it saturates at 2x.
    confidence = min(1.0, area / (2.0 * min_area_px))
    return Detection(True, confidence, (x, y, width, height), (x + width // 2, y + height // 2), area)


class RgbObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__("rgb_object_detection")
        self.declare_parameter("device", "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_C1D2D6AE-video-index0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("yellow_lower_hsv", [20, 100, 80])
        self.declare_parameter("yellow_upper_hsv", [38, 255, 255])
        self.declare_parameter("min_area_px", 500)

        get = lambda name: self.get_parameter(name).value
        self.device = str(get("device"))
        self.width, self.height = int(get("width")), int(get("height"))
        self.fps = float(get("fps"))
        self.lower_hsv = tuple(int(value) for value in get("yellow_lower_hsv"))
        self.upper_hsv = tuple(int(value) for value in get("yellow_upper_hsv"))
        self.min_area_px = int(get("min_area_px"))
        if len(self.lower_hsv) != 3 or len(self.upper_hsv) != 3 or self.min_area_px <= 0 or self.fps <= 0:
            raise ValueError("HSV bounds must have three values; min_area_px and fps must be positive")

        self.image_pub = self.create_publisher(Image, "/rgb_camera/image_raw", 5)
        self.detection_pub = self.create_publisher(String, "/rgb_camera/detection", 10)
        self.capture: cv2.VideoCapture | None = None
        self.last_open_attempt = 0.0
        self.create_timer(1.0 / self.fps, self.process_frame)

    def open_capture(self) -> bool:
        if time.monotonic() - self.last_open_attempt < 2.0:
            return False
        self.last_open_attempt = time.monotonic()
        if self.capture is not None:
            self.capture.release()
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        # Keep the camera's native negotiated rate. Some UVC devices accept a
        # requested fractional rate but then delay or fail their first frame.
        # ``fps`` controls only this node's publish timer.
        if not capture.isOpened():
            self.get_logger().error(f"Cannot open RGB camera: {self.device}")
            capture.release()
            self.capture = None
            return False
        self.capture = capture
        self.get_logger().info(f"RGB camera opened: {self.device}")
        return True

    def process_frame(self) -> None:
        if not rclpy.ok():
            return
        if self.capture is None and not self.open_capture():
            return
        assert self.capture is not None
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning("RGB frame read failed; reopening camera")
            self.capture.release()
            self.capture = None
            return
        height, width = frame.shape[:2]
        detection = detect_yellow_object(frame, self.lower_hsv, self.upper_hsv, self.min_area_px)
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = "rgb_camera"
        image.height, image.width = height, width
        image.encoding = "bgr8"
        image.is_bigendian = False
        image.step = width * 3
        image.data = frame.tobytes()
        try:
            self.image_pub.publish(image)
            payload = detection.as_dict(width, height)
            payload["stamp_ns"] = self.get_clock().now().nanoseconds
            self.detection_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        except Exception:
            # SIGINT/SIGTERM can invalidate ROS between capture and publish.
            # Do not turn normal service shutdown into a restart loop.
            if rclpy.ok():
                raise

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RgbObjectDetection()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
