"""Regression tests for the conservative HOM-8 workspace filter."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import pybullet as pb

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pybullet_sim as sim  # noqa: E402


class WorkspaceSamplesTests(unittest.TestCase):
    def setUp(self) -> None:
        self.client = pb.connect(pb.DIRECT)
        self.robot = pb.loadURDF(
            str(sim.DEFAULT_URDF),
            useFixedBase=True,
            flags=(pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT),
        )

    def tearDown(self) -> None:
        pb.disconnect(self.client)

    def test_default_workspace_keeps_only_front_points_and_reports_rejections(self) -> None:
        points, summary = sim.workspace_samples(self.robot, 1_000, 0.01)

        self.assertTrue(points)
        self.assertTrue(all(x >= 0.0 for x, _, _ in points))
        self.assertGreater(summary["rejected_behind_robot"], 0)
        self.assertGreater(summary["rejected_model_collision"], 0)

    def test_configured_front_boundary_is_respected(self) -> None:
        points, summary = sim.workspace_samples(self.robot, 1_000, 0.01, front_min_x_m=0.10)

        self.assertTrue(points)
        self.assertTrue(all(x >= 0.10 for x, _, _ in points))
        self.assertEqual(summary["front_min_x_m"], 0.10)


if __name__ == "__main__":
    unittest.main()
