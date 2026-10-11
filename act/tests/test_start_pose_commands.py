import unittest
from unittest.mock import patch

from test_record import FakePacket, record


class StartPoseCommandTests(unittest.TestCase):
    def test_return_continues_after_commands_without_arrival_feedback(self):
        cfg = {'servo_ids': [1, 2, 3, 4, 5, 6]}
        packet = FakePacket([1000] * 6)
        packet.read2ByteTxRx = lambda servo_id, register: (
            0 if register == 9 else 4095, 0, 0)
        target = [1100] * 6
        with patch.object(record, 'read_positions',
                          side_effect=[([1000] * 6, 0, 0)]) as read, \
                patch.object(record.time, 'sleep'):
            result = record.return_to_pose(packet, 40, cfg, target,
                                           lambda: -1, keep_powered=True)
        self.assertEqual(result, -1)
        read.assert_called_once()
        self.assertEqual(packet.positions, target)
        self.assertEqual(list(packet.torque.values()), [1] * 6)
