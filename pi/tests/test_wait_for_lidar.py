import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wait_for_lidar import ensure_lidar_link


class WaitForLidarTests(unittest.TestCase):
    def test_recreates_link_when_known_lidar_responds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            port = root / "ttyUSB0"
            port.touch()
            routes = root / "routes.json"
            routes.write_text(json.dumps({"lidar": str(port)}))
            runtime = root / "runtime"
            with patch("wait_for_lidar.is_rplidar", return_value=True) as probe:
                self.assertTrue(ensure_lidar_link(routes, runtime, 1.5))
            probe.assert_called_once_with(str(port), 1.5)
            self.assertEqual((runtime / "lidar").resolve(), port)

    def test_never_probes_port_owned_by_esp32(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            port = root / "ttyUSB0"
            port.touch()
            routes = root / "routes.json"
            routes.write_text(json.dumps({"lidar": str(port)}))
            runtime = root / "runtime"
            runtime.mkdir()
            (runtime / "esp32").symlink_to(port)
            with patch("wait_for_lidar.is_rplidar") as probe:
                self.assertFalse(ensure_lidar_link(routes, runtime, 1.5))
            probe.assert_not_called()


if __name__ == "__main__":
    unittest.main()
