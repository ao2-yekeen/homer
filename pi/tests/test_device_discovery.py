import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from device_discovery import discover


class DeviceDiscoveryTests(unittest.TestCase):
    def test_fixed_links_do_not_probe_serial_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            lidar = root / "lidar-port"
            esp32 = root / "esp32-port"
            camera = root / "camera"
            lidar.symlink_to("/dev/null")
            esp32.symlink_to("/dev/zero")
            camera.touch()
            with patch("device_discovery.LIDAR_PORT", lidar), patch("device_discovery.ESP32_PORT", esp32), patch("device_discovery.RGB_CAMERA", camera):
                devices = discover(root / "runtime")
            self.assertEqual(devices["lidar"], str(lidar))
            self.assertEqual(devices["esp32"], str(esp32))
            self.assertEqual((root / "runtime/lidar").resolve(), Path("/dev/null"))
            self.assertEqual((root / "runtime/esp32").resolve(), Path("/dev/zero"))


if __name__ == "__main__":
    unittest.main()
