import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wait_for_lidar import ensure_lidar_link


class WaitForLidarTests(unittest.TestCase):
    def test_recreates_fixed_link_without_opening_serial_port(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            lidar = root / "lidar-port"
            esp32 = root / "esp32-port"
            lidar.symlink_to("/dev/null")
            esp32.symlink_to("/dev/zero")
            runtime = root / "runtime"
            with patch("wait_for_lidar.LIDAR_PORT", lidar), patch("wait_for_lidar.ESP32_PORT", esp32):
                self.assertTrue(ensure_lidar_link(runtime))
            self.assertEqual((runtime / "lidar").resolve(), Path("/dev/null"))

    def test_missing_or_colliding_fixed_port_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            lidar = root / "lidar-port"
            esp32 = root / "esp32-port"
            runtime = root / "runtime"
            with patch("wait_for_lidar.LIDAR_PORT", lidar), patch("wait_for_lidar.ESP32_PORT", esp32):
                self.assertFalse(ensure_lidar_link(runtime))
                lidar.symlink_to("/dev/null")
                esp32.symlink_to("/dev/null")
                self.assertFalse(ensure_lidar_link(runtime))


if __name__ == "__main__":
    unittest.main()
