"""Check pause removal preserves real motion and synchronized camera samples."""
import copy
import gzip
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).parents[1] / 'runtime'))
from trim_handover import trim_handover, trim_dataset
from record import Episode


def fixture():
    # Replay arrives at frame 5, waits, then a gradual manual movement begins.
    ticks = [0, 20, 40, 60, 80] + [100+(i % 3)-1 for i in range(30)]
    ticks += [102, 104, 108, 112, 116, 120, 130, 140]
    # A later stationary grasp hold must survive.
    ticks += [140]*20
    rows = [dict(state_ticks=[value]*6, image=f'{i:06d}.jpg',
                 overview_image=f'{i:06d}-overview.jpg',
                 capture_end_ns=i*100_000_000, marker=i)
            for i, value in enumerate(ticks)]
    metadata = dict(outcome='success', samples=len(rows), approach_end_frame=10,
                    assistance={'mode': 'replay-then-hand'},
                    config=dict(fps=10, width=16, height=12,
                                overview_camera=dict(width=16, height=12)))
    return rows, metadata


class HandoverTests(unittest.TestCase):
    def test_trim_preserves_alignment_motion_and_later_hold(self):
        rows, metadata = fixture()
        original = copy.deepcopy((rows, metadata))
        kept, updated, removed = trim_handover(rows, metadata)
        self.assertGreater(len(removed), 20)
        self.assertEqual(kept[:5], rows[:5])
        self.assertEqual(kept[-28:], rows[-28:])
        self.assertEqual(updated['samples'], len(kept))
        self.assertEqual(updated['approach_end_frame'], 5)
        self.assertEqual((rows, metadata), original)
        self.assertEqual(trim_handover(kept, updated), (kept, updated, []))
        for row in kept:
            self.assertEqual(row, rows[row['marker']])

    def test_skip_source_unmarked_failed_and_short_pauses(self):
        rows, metadata = fixture()
        for change in ({'joint_only': True}, {'approach_end_frame': None},
                       {'outcome': 'invalid'}, {'assistance': {'mode': 'joystick'}}):
            changed = dict(metadata, **change)
            self.assertEqual(trim_handover(rows, changed), (rows, changed, []))
        short = rows[:5]+rows[30:]
        changed = dict(metadata, samples=len(short), approach_end_frame=5)
        self.assertEqual(trim_handover(short, changed), (short, changed, []))

    def test_feedback_spike_does_not_end_pause(self):
        rows, metadata = fixture()
        rows[15]['state_ticks'] = [180]*6
        kept, updated, removed = trim_handover(rows, metadata)
        self.assertGreater(len(removed), 20)
        self.assertEqual(updated['samples'], len(kept))

    def test_recorder_saves_only_retained_joint_and_camera_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            rows, metadata = fixture()
            directory = Path(temporary) / 'episode'
            episode = Episode(directory, metadata['config'], [0]*6, None, [0]*6, 0)
            episode.approach_end_frame = metadata['approach_end_frame']
            episode.assistance = metadata['assistance']
            frame = np.zeros((12, 16, 3), dtype=np.uint8)
            for i, row in enumerate(rows):
                t=i*100_000_000
                episode.append(frame, frame, row['state_ticks'], t, t+1, t+2, t+3, t+4, t+5)
            episode.finish('success')
            saved = json.loads((directory/'metadata.json').read_text())
            with gzip.open(directory/'samples.jsonl.gz', 'rt') as stream:
                samples = [json.loads(line) for line in stream]
            self.assertEqual(saved['samples'], len(samples))
            self.assertLess(len(samples), len(rows))
            self.assertEqual(len(list(directory.glob('*.jpg'))), 2*len(samples))
            for row in samples:
                self.assertTrue((directory/row['image']).exists())
                self.assertTrue((directory/row['overview_image']).exists())

    def test_packager_trims_existing_demos_without_changing_originals(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root/'data/episode-test'
            directory.mkdir(parents=True)
            rows, metadata = fixture()
            (directory/'metadata.json').write_text(json.dumps(metadata))
            with gzip.open(directory/'samples.jsonl.gz', 'wt') as stream:
                for row in rows:
                    stream.write(json.dumps(row)+'\n')
                    for field in ('image', 'overview_image'):
                        Image.new('RGB', (16, 12)).save(directory/row[field])
            before = {p.name: p.read_bytes() for p in directory.iterdir()}
            (root/'episodes.txt').write_text('episode-test\n')
            subprocess.run([sys.executable, str(Path(__file__).parents[1]/'training/package_training.py'),
                            '--data', str(root/'data'), '--episodes-file', str(root/'episodes.txt'),
                            '--output', str(root/'training.zip')], check=True, capture_output=True)
            with zipfile.ZipFile(root/'training.zip') as archive:
                prefix = 'act-gpu-training/data/episode-test/'
                saved = json.loads(archive.read(prefix+'metadata.json'))
                samples = [json.loads(line) for line in gzip.decompress(
                    archive.read(prefix+'samples.jsonl.gz')).decode().splitlines()]
                expected, _, _ = trim_handover(rows, metadata)
                self.assertEqual(samples, expected)
                self.assertEqual(saved['samples'], len(expected))
                self.assertEqual(sum(name.endswith('.jpg') for name in archive.namelist()), 2*len(expected))
            self.assertEqual(before, {p.name: p.read_bytes() for p in directory.iterdir()})

    def test_existing_data_trim_is_aligned_and_idempotent(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root/'episode-test'
            directory.mkdir()
            rows, metadata = fixture()
            (directory/'metadata.json').write_text(json.dumps(metadata))
            with gzip.open(directory/'samples.jsonl.gz', 'wt') as stream:
                for row in rows:
                    stream.write(json.dumps(row)+'\n')
                    for field in ('image', 'overview_image'):
                        (directory/row[field]).write_bytes(b'image')
            expected, updated, _ = trim_handover(rows, metadata)
            trim_dataset(root)
            with gzip.open(directory/'samples.jsonl.gz', 'rt') as stream:
                self.assertEqual([json.loads(line) for line in stream], expected)
            self.assertEqual(json.loads((directory/'metadata.json').read_text()), updated)
            self.assertEqual({p.name for p in directory.glob('*.jpg')},
                             {row[field] for row in expected
                              for field in ('image', 'overview_image')})
            before = {p.name: p.read_bytes() for p in directory.iterdir()}
            trim_dataset(root)
            self.assertEqual(before, {p.name: p.read_bytes() for p in directory.iterdir()})


if __name__ == '__main__':
    unittest.main()
