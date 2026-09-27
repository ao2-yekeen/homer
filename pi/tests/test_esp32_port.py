import tempfile
import unittest
from pathlib import Path

import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from esp32_port import candidates


class PortTests(unittest.TestCase):
    def test_only_fixed_esp32_path_is_a_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            esp32 = root / "esp32-port"
            lidar = root / "lidar-port"
            esp32.symlink_to("/dev/zero")
            lidar.symlink_to("/dev/null")
            link = root / "runtime-esp32"
            link.symlink_to(esp32)
            with patch("esp32_port.ESP32_PORT", esp32), patch("esp32_port.LIDAR_PORT", lidar):
                self.assertEqual(candidates(link), [str(link)])
                link.unlink()
                link.symlink_to(lidar)
                self.assertEqual(candidates(link), [])
                link.unlink()
                self.assertEqual(candidates(link), [])


if __name__ == "__main__":
    unittest.main()
