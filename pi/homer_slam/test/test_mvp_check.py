import math
from types import SimpleNamespace
import unittest

from homer_slam.mvp_check import displacement, pose_xy_yaw, tracking_result


class TrackingTests(unittest.TestCase):
    def test_wraparound_is_small_rotation(self):
        translation, yaw = displacement((0, 0, math.radians(179)), (0, 0, math.radians(-179)))
        self.assertEqual(translation, 0)
        self.assertAlmostEqual(yaw, 2)

    def test_stationary_rejects_excursion_even_after_return(self):
        result = tracking_result([(0, 0, 0), (.06, 0, 0), (0, 0, 0)], 'stationary', 60, True)
        self.assertFalse(result['gate'])
        self.assertEqual(result['endpoint_translation_m'], 0)

    def test_missing_operator_confirmation_is_not_a_pass(self):
        result = tracking_result([(0, 0, 0), (.001, 0, 0)], 'stationary', 60, False)
        self.assertTrue(result['numerical_pass'])
        self.assertIsNone(result['gate'])

    def test_short_stationary_trial_cannot_pass(self):
        self.assertFalse(tracking_result([(0, 0, 0), (0, 0, 0)], 'stationary', 59, True)['gate'])

    def test_return_requires_movement_and_small_final_error(self):
        self.assertFalse(tracking_result([(0, 0, 0), (.01, 0, 0)], 'return', 60, True)['gate'])
        self.assertTrue(tracking_result([(0, 0, 0), (1, 0, 0), (.1, 0, .05)], 'return', 60, True)['gate'])
        self.assertFalse(tracking_result([(0, 0, 0), (1, 0, 0), (.3, 0, 0)], 'return', 60, True)['gate'])

    def test_invalid_pose_is_rejected(self):
        pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0.), orientation=SimpleNamespace(x=0., y=0., z=0., w=0.))
        with self.assertRaisesRegex(ValueError, 'normalized'):
            pose_xy_yaw(pose)
        pose.orientation.w = 1.
        self.assertEqual(pose_xy_yaw(pose), (0., 0., 0.))
        pose.position.x = float('nan')
        with self.assertRaisesRegex(ValueError, 'Nonfinite'):
            pose_xy_yaw(pose)
