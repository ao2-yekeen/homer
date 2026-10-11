"""Check that pause shortening preserves meaningful motion and compression."""
import gzip
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

PATH = Path(__file__).resolve().parents[1] / 'training/prepare_corrections.py'
spec = importlib.util.spec_from_file_location('prepare_corrections', PATH)
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


class PauseTests(unittest.TestCase):
    def select(self, ticks, views=None):
        rows = [{'state_ticks': pose} for pose in ticks]
        if views is None:
            views = [np.zeros((len(rows), 10, 10), dtype=np.uint8)] * 2
        return prepare.retained_indices(rows, views)

    def test_long_hold_keeps_context_and_last_frame(self):
        kept, removed = self.select([[100]*6]*30)
        self.assertEqual(kept, [0, 1, 2, 28, 29])
        self.assertEqual(removed, [[3, 28]])

    def test_slow_cumulative_motion_and_short_holds_survive(self):
        ticks = [[100+i, 100, 100, 100, 100, 100] for i in range(30)]
        self.assertEqual(self.select(ticks)[0], list(range(30)))
        self.assertEqual(self.select([[100]*6]*9)[0], list(range(9)))

    def test_visual_motion_protects_still_joints(self):
        views = [np.full((10, 10), i*5, dtype=np.uint8) for i in range(30)]
        self.assertEqual(self.select([[100]*6]*30, [np.stack(views)])[0], list(range(30)))

    def test_gripper_change_preserves_surrounding_context(self):
        ticks = [[100]*5+[100 if i < 20 else 110] for i in range(45)]
        kept, _ = self.select(ticks)
        self.assertTrue(set(range(15, 26)).issubset(kept))
        self.assertLess(len(kept), len(ticks))

    def test_export_preserves_source_rows_and_existing_sized_jpeg(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'episode-source'
            source.mkdir()
            Image.new('RGB', (320, 240), 'red').save(source/'frame.jpg', quality=75)
            original = (source/'frame.jpg').read_bytes()
            rows = [dict(state_ticks=[100]*6, read_end_ns=i*100000000,
                         image='frame.jpg', overview_image='frame.jpg') for i in range(3)]
            metadata = {'samples': 3, 'config': {'overview_camera': {}}, 'assistance': {'source_run': 'run'}}
            destination = Path(temporary) / 'copy'
            updated = prepare.export_episode(source, destination, metadata, rows, [0, 2], (320, 240))
            with gzip.open(destination/'samples.jsonl.gz', 'rt') as stream:
                exported = [json.loads(line) for line in stream]
            self.assertEqual([r['source_frame'] for r in exported], [0, 2])
            self.assertEqual(exported[-1]['read_end_ns'], rows[-1]['read_end_ns'])
            self.assertNotIn('source_frame', rows[0])
            self.assertEqual((destination/'frame.jpg').read_bytes(), original)
            self.assertEqual(metadata['samples'], 3)
            self.assertEqual(updated['samples'], 2)
            self.assertEqual(updated['assistance'], metadata['assistance'])


if __name__ == '__main__':
    unittest.main()
