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
    def run_keys(self, keys, failure=None, start_pose=None):
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
                    correction.record_loop(cfg, packet, 40, cameras, poll, Path(tmp), 'test-run', start_pose)
            else:
                correction.record_loop(cfg, packet, 40, cameras, poll, Path(tmp), 'test-run', start_pose)
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

    def test_saved_pose_returns_on_start_and_after_success_outside_recording(self):
        pose = dict(state_ticks=[2100]*6, source_run='test-run')
        with patch.object(correction, 'return_to_pose', return_value=-1) as move:
            _, [(metadata, rows)] = self.run_keys(['h', -1, 'f', 'q'], start_pose=pose)
        self.assertEqual(move.call_count, 2)
        self.assertEqual(metadata['correction_start_pose'], pose)
        self.assertEqual(len(rows), 2)
        self.assertTrue(all(call.kwargs['keep_powered'] for call in move.call_args_list))

    def test_startup_reaches_initial_pose_then_replays_before_manual_recording(self):
        pose = dict(initial_ticks=[2000]*6, state_ticks=[2100]*6,
                    replay_actions=[[2050]*6, [2090]*6])
        events = []
        def move(*args, **kwargs):
            events.append(('move', args[3]))
            return -1
        def send(*args):
            events.append(('replay', args[2]))
        with patch.object(correction, 'return_to_pose', side_effect=move), \
                patch.object(correction, 'write_targets', side_effect=send):
            writes, [(metadata, rows)] = self.run_keys([-1, -1, 'h', -1, 'f', 'q'], start_pose=pose)
        self.assertEqual(events, [('move', [2000]*6), ('replay', [2050]*6),
                                  ('replay', [2090]*6), ('move', [2100]*6),
                                  ('move', [2100]*6)])
        self.assertEqual(len(rows), 2)
        self.assertEqual(writes, [(i, 40, 0) for i in range(1, 7)])

    def test_cancel_replay_does_not_record_or_reach_handover(self):
        pose = dict(initial_ticks=[2000]*6, state_ticks=[2100]*6,
                    replay_actions=[[2050]*6, [2090]*6])
        with patch.object(correction, 'return_to_pose', return_value=-1) as move, \
                patch.object(correction, 'write_targets') as send:
            writes, results = self.run_keys(['q'], start_pose=pose)
        self.assertEqual(move.call_count, 1)
        send.assert_not_called()
        self.assertEqual(results, [])
        self.assertEqual(writes, [(i, 40, 0) for i in range(1, 7)])

    def test_cancel_initial_move_never_starts_recording(self):
        with patch.object(correction, 'return_to_pose', return_value=ord('q')) as move:
            writes, results = self.run_keys([], start_pose=dict(state_ticks=[2100]*6))
        self.assertEqual(move.call_count, 1)
        self.assertEqual(writes, [])
        self.assertEqual(results, [])

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
