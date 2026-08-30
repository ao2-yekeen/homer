import argparse
import unittest

from soarm_workspace_record import make_sample, parse_joint_ticks


class WorkspaceRecordTests(unittest.TestCase):
    def args(self, **overrides):
        values = {
            "sample_id": "front-centre-01",
            "x_m": 0.20,
            "y_m": 0.0,
            "z_m": 0.32,
            "result": "reachable",
            "reason": "Supported arm held point without contact.",
            "joint_ticks": ["1000", "1500", "1800", "2700", "2048", "3000"],
            "mast_clear": True,
            "base_clear": True,
            "camera_clear": True,
            "platform_clear": True,
            "cables_clear": True,
            "confirm_physical_observation": True,
        }
        values.update(overrides)
        return argparse.Namespace(**values)

    def test_valid_sample_has_frame_and_ticks(self):
        sample = make_sample(self.args())
        self.assertEqual(sample["frame_id"], "base_footprint")
        self.assertEqual(sample["joint_ticks"], [1000, 1500, 1800, 2700, 2048, 3000])

    def test_requires_operator_attestation(self):
        with self.assertRaisesRegex(ValueError, "confirm-physical-observation"):
            make_sample(self.args(confirm_physical_observation=False))

    def test_rejects_bad_joint_tick_count(self):
        with self.assertRaisesRegex(ValueError, "exactly six"):
            parse_joint_ticks(["1", "2"])

    def test_rejects_out_of_range_ticks(self):
        with self.assertRaisesRegex(ValueError, "0..4095"):
            parse_joint_ticks(["0", "0", "0", "0", "0", "4096"])
