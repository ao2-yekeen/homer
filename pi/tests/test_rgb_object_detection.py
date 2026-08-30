"""Run on the Pi, where OpenCV and ROS Python dependencies are installed."""

import unittest

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - development hosts may not have OpenCV
    cv2 = None

if cv2 is not None:
    from rgb_object_detection import detect_coloured_object


@unittest.skipIf(cv2 is None, "OpenCV is required (available on robot-pi)")
class BlackDetectionTests(unittest.TestCase):
    def test_detects_large_black_region(self) -> None:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (240, 240, 240)
        cv2.rectangle(frame, (100, 120), (220, 260), (20, 20, 20), -1)
        result = detect_coloured_object(frame, (0, 0, 0), (179, 255, 80), 500)
        self.assertTrue(result.detected)
        self.assertGreaterEqual(result.confidence, 0.99)
        self.assertEqual(result.center_uv, (160, 190))

    def test_rejects_small_or_bright_regions(self) -> None:
        frame = np.full((240, 320, 3), 240, dtype=np.uint8)
        cv2.rectangle(frame, (10, 10), (20, 20), (20, 20, 20), -1)
        cv2.rectangle(frame, (50, 50), (200, 200), (0, 255, 255), -1)
        result = detect_coloured_object(frame, (0, 0, 0), (179, 255, 80), 500)
        self.assertFalse(result.detected)
        self.assertIsNone(result.bbox_xywh)
