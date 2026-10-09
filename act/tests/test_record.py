import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock
import yaml
import numpy as np

spec = importlib.util.spec_from_file_location('record', Path(__file__).parents[1] / 'runtime/record.py')
record = importlib.util.module_from_spec(spec)
spec.loader.exec_module(record)


class RecorderTests(unittest.TestCase):
    def test_continuous_collection_and_review_with_simulated_inputs(self):
        # Exercise the actual CLI loop without hardware; never a training demo.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cfg = dict(servo_ids=list(range(1, 7)), joint_names=[f'joint-{i}' for i in range(6)],
                       limits=[[0, 4095]] * 6, fps=10, camera='simulated', width=64,
                       height=48, guidance_verified=True)
            config = root / 'rig.yaml'
            config.write_text(yaml.safe_dump(cfg))
            cap = Mock()
            cap.isOpened.return_value = True
            cap.read.side_effect = lambda: (True, np.zeros((48, 64, 3), dtype=np.uint8))
            packet = Mock()
            packet.ReadPosSpeed.return_value = (2000, 0, 0, 0)
            packet.read1ByteTxRx.return_value = (0, 0, 0)
            port, lock = Mock(), Mock()
            keys = iter(map(ord, ['s', 'x', 'f', 's', 'x', 'd', 's', 'x', 'q']))
            with patch.object(record, 'ROOT', root), \
                 patch.object(record.sys, 'argv', ['record.py', '--config', str(config)]), \
                 patch.object(record, 'open_arm', return_value=(port, packet, 40, lock)), \
                 patch.object(record.cv2, 'VideoCapture', return_value=cap), \
                 patch.object(record.cv2, 'imshow'), \
                 patch.object(record.cv2, 'waitKey', side_effect=lambda _: next(keys)), \
                 patch.object(record.cv2, 'destroyAllWindows'):
                record.main()
            episodes = sorted((root / 'data').glob('episode-*'))
            self.assertEqual(len(episodes), 3)
            outcomes = [json.loads((e / 'metadata.json').read_text())['outcome'] for e in episodes]
            self.assertEqual(outcomes, ['success', 'discard', 'discard'])
            record.review(episodes[0])
            self.assertGreater((episodes[0] / 'review.avi').stat().st_size, 0)
            self.assertTrue((episodes[0] / 'joints.png').exists())
            packet.WritePosEx.assert_not_called()
            packet.write1ByteTxRx.assert_not_called()
            port.closePort.assert_called_once()
            lock.close.assert_called_once()
            cap.release.assert_called_once()

    def test_failed_servo_read_is_rejected(self):
        class Packet:
            def ReadPosSpeed(self, servo):
                return (100, 0, -1, 0)
        with self.assertRaises(RuntimeError):
            record.read_positions(Packet(), {'servo_ids': [1], 'limits': [[0, 200]]})

    def test_invalid_raw_ticks_are_rejected(self):
        class Packet:
            def ReadPosSpeed(self, servo):
                return (4096, 0, 0, 0)
        with self.assertRaises(RuntimeError):
            record.read_positions(Packet(), {'servo_ids': [1], 'limits': [[0, 200]]})

    def test_old_task_limits_do_not_restrict_recording(self):
        class Packet:
            def ReadPosSpeed(self, servo):
                return (1580, 0, 0, 0)
        positions, _, _ = record.read_positions(Packet(), {'servo_ids': [4], 'limits': [[2438, 3233]]})
        self.assertEqual(positions, [1580])

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
