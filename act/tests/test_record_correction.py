import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / 'runtime'))
import record_correction as correction


class Packet:
    def __init__(self):
        self.writes = []

    def ReadPosSpeed(self, servo):
        return 2000 + servo, 0, 0, 0

    def write1ByteTxRx(self, servo, register, value):
        self.writes.append((servo, register, value))
        return 0, 0

    def read1ByteTxRx(self, servo, register):
        return 0, 0, 0


class Camera:
    def read(self):
        return True, np.zeros((16, 16, 3), dtype=np.uint8)


class CorrectionTests(unittest.TestCase):
    def run_keys(self, keys, failure=None):
        packet = Packet()
        cfg = dict(servo_ids=list(range(1, 7)), joint_names=[str(i) for i in range(6)], fps=10)
        keys = iter(keys)
        with tempfile.TemporaryDirectory() as tmp, patch.object(correction.time, 'sleep'):
            def poll():
                key = next(keys)
                if key == 'interrupt':
                    raise KeyboardInterrupt()
                return ord(key) if isinstance(key, str) else key

            cameras = [('wrist', Camera()), ('overview', Camera())]
            if failure:
                with self.assertRaises(failure):
                    correction.record_loop(cfg, packet, 40, cameras, poll, Path(tmp), 'test-run')
            else:
                correction.record_loop(cfg, packet, 40, cameras, poll, Path(tmp), 'test-run')
            results = []
            for directory in Path(tmp).glob('episode-*'):
                metadata = json.loads((directory / 'metadata.json').read_text())
                with gzip.open(directory / 'samples.jsonl.gz', 'rt') as stream:
                    rows = [json.loads(line) for line in stream]
                for row in rows:
                    self.assertTrue((directory / row['image']).is_file())
                    self.assertTrue((directory / row['overview_image']).is_file())
                results.append((metadata, rows))
            return packet.writes, results

    def test_waiting_and_exit_do_not_write_servos_or_save_data(self):
        writes, results = self.run_keys([-1, -1, 'q'])
        self.assertEqual(writes, [])
        self.assertEqual(results, [])

    def test_only_human_segment_saved_with_paired_images_and_provenance(self):
        writes, [(metadata, rows)] = self.run_keys([-1, -1, 'h', -1, 'f', 'q'])
        self.assertEqual(writes, [(i, 40, 0) for i in range(1, 7)])
        self.assertEqual(metadata['outcome'], 'success')
        self.assertEqual(metadata['samples'], 2)
        self.assertIsNone(metadata['return_completed_ns'])
        self.assertEqual(metadata['assistance']['mode'], 'manual-correction')
        self.assertEqual(metadata['assistance']['source_run'], 'test-run')
        self.assertTrue(all(row['control']['action_source'] == 'human' for row in rows))

    def test_interrupt_never_marks_partial_correction_success_or_enables_torque(self):
        writes, [(metadata, _)] = self.run_keys(['h', 'interrupt'], KeyboardInterrupt)
        self.assertEqual(metadata['outcome'], 'invalid')
        self.assertTrue(all(value == 0 for _, _, value in writes))

    def test_quit_and_discard_exclude_partial_corrections_from_training(self):
        for keys in (['h', -1, 'q'], ['h', -1, 'd', 'q']):
            _, [(metadata, _)] = self.run_keys(keys)
            self.assertEqual(metadata['outcome'], 'discard')


if __name__ == '__main__':
    unittest.main()
