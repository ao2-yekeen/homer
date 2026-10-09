import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

spec = importlib.util.spec_from_file_location(
    'record', Path(__file__).parents[1] / 'runtime/record.py')
record = importlib.util.module_from_spec(spec)
spec.loader.exec_module(record)


class FakePacket:
    def __init__(self, positions):
        self.positions = list(positions)
        self.torque = {i: 0 for i in range(1, len(positions) + 1)}
        self.events = []
        self.fail_write_number = None
        self.position_writes = 0

    def ReadPosSpeed(self, servo_id):
        return self.positions[servo_id - 1], 0, 0, 0

    def WritePosEx(self, servo_id, target, speed, acceleration):
        self.position_writes += 1
        if self.position_writes == self.fail_write_number:
            return -1, 0
        self.events.append(('position', servo_id, target, speed, acceleration))
        self.positions[servo_id - 1] = target
        return 0, 0

    def read1ByteTxRx(self, servo_id, register):
        return self.torque[servo_id], 0, 0

    def write1ByteTxRx(self, servo_id, register, value):
        self.events.append(('torque', servo_id, value))
        self.torque[servo_id] = value
        return 0, 0


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {
            'servo_ids': [1, 2, 3, 4, 5, 6],
            'joint_names': ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                            'wrist_flex', 'wrist_roll', 'gripper'],
        }

    def test_saved_live_pose_is_reused_instead_of_replaced(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'start-pose.json'
            saved = [2264, 150, 3029, 1446, 3884, 2032]
            self.assertEqual(record.get_session_start_pose(path, self.cfg, saved, 10),
                             (saved, 10))
            self.assertEqual(record.get_session_start_pose(path, self.cfg,
                                                           [1000] * 6, 20),
                             (saved, None))
            self.assertEqual(json.loads(path.read_text())['state_ticks'], saved)

    def test_return_stages_live_goals_moves_in_small_steps_and_releases_torque(self):
        start = [1000, 2000, 3000, 400, 500, 600]
        target = [1055, 1960, 3000, 420, 500, 580]
        packet = FakePacket(start)
        with patch.object(record.time, 'sleep'):
            self.assertEqual(record.return_to_pose(packet, 40, self.cfg,
                                                    target, lambda: -1), -1)
        self.assertEqual(packet.positions, target)
        self.assertEqual(list(packet.torque.values()), [0] * 6)
        first_enable = next(i for i, event in enumerate(packet.events)
                            if event[:2] == ('torque', 1) and event[2] == 1)
        staged = packet.events[:first_enable]
        self.assertEqual([event[2] for event in staged], start)
        goals = {i: start[i - 1] for i in self.cfg['servo_ids']}
        for event in packet.events:
            if event[0] == 'position':
                servo_id, next_goal = event[1], event[2]
                self.assertLessEqual(abs(next_goal - goals[servo_id]), 20)
                goals[servo_id] = next_goal
        self.assertEqual(packet.events[-1], ('torque', 6, 0))

    def test_cancel_and_write_failure_both_release_torque(self):
        target = [1100] * 6
        cancelled = FakePacket([1000] * 6)
        self.assertEqual(record.return_to_pose(cancelled, 40, self.cfg,
                                                target, lambda: ord('x')), ord('x'))
        self.assertEqual(list(cancelled.torque.values()), [0] * 6)
        failed = FakePacket([1000] * 6)
        failed.fail_write_number = 7
        with self.assertRaises(RuntimeError):
            record.return_to_pose(failed, 40, self.cfg, target, lambda: -1)
        self.assertEqual(list(failed.torque.values()), [0] * 6)

    def test_episode_records_both_cameras_and_fixed_start_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            start = [2264, 150, 3029, 1446, 3884, 2032]
            current = [2266, 151, 3027, 1448, 3884, 2032]
            directory = Path(tmp) / 'episode'
            episode = record.Episode(directory, self.cfg, start, None, current, 99)
            frame = np.zeros((20, 20, 3), dtype=np.uint8)
            episode.append(frame, frame, current, 1, 2, 3, 4, 5, 6)
            episode.finish('success')
            row = json.loads((directory / 'samples.jsonl').read_text())
            metadata = json.loads((directory / 'metadata.json').read_text())
            self.assertTrue((directory / row['image']).exists())
            self.assertTrue((directory / row['overview_image']).exists())
            self.assertEqual(metadata['start_state_ticks'], start)
            self.assertEqual(metadata['recording_start_ticks'], current)
            self.assertEqual(row['overview_capture_end_ns'], 4)

    def test_invalid_servo_read_is_rejected(self):
        packet = FakePacket([4096] * 6)
        with self.assertRaises(RuntimeError):
            record.read_positions(packet, self.cfg)


if __name__ == '__main__':
    unittest.main()
