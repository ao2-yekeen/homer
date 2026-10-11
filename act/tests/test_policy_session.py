import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / 'runtime'))
import policy_session as session


class PolicySessionTests(unittest.TestCase):
    def test_overload_identifies_servo_and_stops_reading(self):
        packet = MagicMock()
        packet.ReadPosSpeed.side_effect = [(2000, 0, 0, 0), (1942, 0, 0, 32)]
        packet.getRxPacketError.return_value = '[ServoStatus] Overload error!'
        cfg = dict(servo_ids=[1, 2, 3], joint_names=['pan', 'lift', 'elbow'])
        with self.assertRaisesRegex(RuntimeError, r'Servo 2 \(lift\).*Overload.*status=32'):
            session.read_positions(packet, cfg)
        self.assertEqual(packet.ReadPosSpeed.call_count, 2)

    def test_communication_failure_is_distinct_from_motor_fault(self):
        packet = MagicMock()
        packet.ReadPosSpeed.return_value = (0, 0, -6, 32)
        with self.assertRaisesRegex(RuntimeError, 'communication error -6'):
            session.read_positions(packet, dict(servo_ids=[3], joint_names=['elbow']))
        packet.getRxPacketError.assert_not_called()

    def run_session(self, fault=False, interrupt=None):
        cfg = dict(servo_ids=[1], joint_names=['pan'], camera='wrist',
                   overview_camera=dict(device='overview'), fps=10)
        packet, port, lock, camera = [MagicMock() for _ in range(4)]
        packet.read2ByteTxRx.side_effect = [(0, 0, 0), (4095, 0, 0)]
        camera.read.return_value = (True, np.zeros((8, 8, 3), dtype=np.uint8))
        feedback = ([2000], 0, 1)
        prediction = json.dumps(dict(joint_names=['pan'], actions=[[2001]]))+'\n'
        control = '{"return_to_start": true}\n'
        incoming = control if interrupt == 'waiting' else prediction + (control if interrupt else '')
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'rig.yaml'
            config.write_text(yaml.safe_dump(cfg))
            argv = ['policy_session.py', '--config', str(config), '--output', tmp, '--steps', '1']
            with patch.object(sys, 'argv', argv), \
                 patch.object(sys, 'stdin', io.StringIO(incoming)), \
                 patch.object(sys, 'stdout', io.StringIO()), \
                 patch.object(sys, 'stderr', io.StringIO()), \
                 patch.object(session, 'open_arm', return_value=(port, packet, 40, lock)), \
                 patch.object(session, 'return_to_pose') as return_pose, \
                 patch.object(session.select, 'select', return_value=([True], [], []) if interrupt == 'moving' else ([], [], [])), \
                 patch.object(session.Path, 'read_text', return_value='{"state_ticks": [2000]}'), \
                 patch.object(session.yaml, 'safe_load', return_value=cfg), \
                 patch.object(session.cv2, 'VideoCapture', return_value=camera), \
                 patch.object(session.time, 'sleep'), \
                 patch.object(session, 'read_positions', side_effect=[feedback, RuntimeError('servo overload')] if fault else None, return_value=feedback) as read, \
                 patch.object(session, 'write_targets') as write:
                finished = MagicMock(side_effect=lambda: self.assertNotIn('"finished": true', sys.stdout.getvalue()))
                if interrupt:
                    with self.assertRaises(SystemExit) as stopped:
                        session.main(finished)
                    self.assertEqual(stopped.exception.code, 130)
                    self.assertEqual(return_pose.call_count, 2)
                    self.assertEqual(return_pose.call_args.args[:4], (packet, 40, cfg, [2000]))
                    self.assertTrue(return_pose.call_args.kwargs['keep_powered'])
                    write.assert_not_called()
                    finished.assert_called_once_with()
                elif fault:
                    with self.assertRaisesRegex(RuntimeError, 'servo overload'):
                        session.main(finished)
                    finished.assert_not_called()
                    self.assertEqual(read.call_count, 2)
                    write.assert_not_called()
                else:
                    session.main(finished)
                    finished.assert_called_once_with()
                    self.assertEqual(write.call_count, 2)  # action, then measured hold
            report = json.loads((Path(tmp) / 'movement-report.json').read_text())
            port.closePort.assert_called_once()
            lock.close.assert_called_once()
            packet.write1ByteTxRx.assert_not_called()  # no cleanup torque enable
            return report

    def test_fault_cleanup_preserves_error_without_more_servo_io(self):
        report = self.run_session(fault=True)
        self.assertFalse(report['completed'])
        self.assertEqual(report['error'], 'servo overload')

    def test_success_still_sends_measured_hold(self):
        report = self.run_session()
        self.assertTrue(report['completed'])
        self.assertTrue(report['hold_command_sent'])

    def test_interrupt_while_waiting_returns_to_start(self):
        report = self.run_session(interrupt='waiting')
        self.assertTrue(report['returned_to_start'])
        self.assertTrue(report['interrupted'])
        self.assertFalse(report['completed'])

    def test_interrupt_during_chunk_returns_before_next_action(self):
        report = self.run_session(interrupt='moving')
        self.assertTrue(report['returned_to_start'])
        self.assertEqual(report['commands'], [])


if __name__ == '__main__':
    unittest.main()
