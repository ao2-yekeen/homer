"""Unit tests for the ROS ToF image conversion helper.

Run on robot-pi, where the Arducam SDK and ROS Python packages are installed.
"""

import unittest

try:
    import numpy as np
    from builtin_interfaces.msg import Time
    from tof_depth_publisher import depth_image
except ImportError:  # pragma: no cover - development hosts may lack ROS/SDK
    np = None


@unittest.skipIf(np is None, "ROS Python dependencies are required")
class DepthImageTests(unittest.TestCase):
    def test_preserves_float32_millimetres(self) -> None:
        source = np.array([[123.5, 456.0], [789.25, 1000.0]], dtype=np.float32)
        message = depth_image(source, Time(sec=12, nanosec=34), "tof_camera")
        self.assertEqual((message.height, message.width), (2, 2))
        self.assertEqual(message.encoding, "32FC1")
        self.assertEqual(message.step, 8)
        self.assertEqual(message.header.frame_id, "tof_camera")
        received = np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.width)
        np.testing.assert_array_equal(received, source)

    def test_rejects_non_image_array(self) -> None:
        with self.assertRaises(ValueError):
            depth_image(np.zeros((1, 2, 3), dtype=np.float32), Time(), "tof_camera")
