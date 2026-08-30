import json
import tempfile
import unittest
from pathlib import Path

from soarm_reachability_map import (
    MARGINAL,
    REACHABLE,
    UNREACHABLE,
    classify,
    make_lookup,
    read_ascii_ply,
    validate_samples,
)


class ReachabilityMapTests(unittest.TestCase):
    def setUp(self):
        self.lookup = make_lookup([(0.10, 0.00, 0.30)], 0.01, 0.01)

    def test_classifies_sampled_neighbouring_and_distant_cells(self):
        self.assertEqual(classify(self.lookup, (0.10, 0.00, 0.30)), REACHABLE)
        self.assertEqual(classify(self.lookup, (0.11, 0.00, 0.30)), MARGINAL)
        self.assertEqual(classify(self.lookup, (0.20, 0.00, 0.30)), UNREACHABLE)

    def test_reads_ascii_ply(self):
        with tempfile.TemporaryDirectory() as directory:
            ply = Path(directory) / "points.ply"
            ply.write_text("ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n0.1 0.0 0.3\n")
            self.assertEqual(read_ascii_ply(ply), [(0.1, 0.0, 0.3)])

    def test_validation_keeps_marginal_results_out_of_match_count(self):
        with tempfile.TemporaryDirectory() as directory:
            samples = Path(directory) / "samples.jsonl"
            samples.write_text(json.dumps({"sample_id": "known", "frame_id": "base_footprint", "gripper_center_xyz_m": [0.1, 0, 0.3], "result": "reachable"}) + "\n" + json.dumps({"sample_id": "edge", "frame_id": "base_footprint", "gripper_center_xyz_m": [0.11, 0, 0.3], "result": "reachable"}) + "\n")
            result = validate_samples(self.lookup, samples)
        self.assertEqual(result["matches"], 1)
        self.assertEqual(result["requires_review"], 1)


if __name__ == "__main__":
    unittest.main()
