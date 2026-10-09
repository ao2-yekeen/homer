import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('record', Path(__file__).parents[1] / 'runtime/record.py')
record = importlib.util.module_from_spec(spec)
spec.loader.exec_module(record)


class RecorderTests(unittest.TestCase):
    def test_failed_servo_read_is_rejected(self):
        class Packet:
            def ReadPosSpeed(self, servo):
                return (100, 0, -1, 0)
        with self.assertRaises(RuntimeError):
            record.read_positions(Packet(), {'servo_ids': [1], 'limits': [[0, 200]]})

    def test_outside_limits_is_rejected(self):
        class Packet:
            def ReadPosSpeed(self, servo):
                return (201, 0, 0, 0)
        with self.assertRaises(RuntimeError):
            record.read_positions(Packet(), {'servo_ids': [1], 'limits': [[0, 200]]})

    def test_episode_links_image_ticks_and_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'episode'
            episode = record.Episode(path, {})
            ticks = [1, 2, 3, 4, 5, 6]
            episode.append(np.zeros((20, 20, 3), dtype=np.uint8), ticks, 1, 2, 3, 4)
            episode.finish('invalid', 'read failure')
            row = json.loads((path / 'samples.jsonl').read_text())
            self.assertTrue((path / row['image']).exists())
            self.assertEqual(row['state_ticks'], ticks)
            self.assertEqual(row['read_end_ns'], 4)
            self.assertEqual(json.loads((path / 'metadata.json').read_text())['outcome'], 'invalid')


if __name__ == '__main__':
    unittest.main()
