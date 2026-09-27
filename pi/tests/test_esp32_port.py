import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from esp32_port import candidates, record_esp32


class PortTests(unittest.TestCase):
    def test_runtime_link_replaced_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "ttyUSB0"
            second = root / "ttyUSB1"
            first.touch()
            second.touch()
            link = root / "runtime" / "esp32"
            record_esp32(link, str(first))
            self.assertEqual(link.resolve(), first)
            record_esp32(link, str(second))
            self.assertEqual(link.resolve(), second)
            self.assertEqual(candidates(link)[0], str(link))


if __name__ == "__main__":
    unittest.main()
