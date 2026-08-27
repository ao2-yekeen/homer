import unittest

from gripper_contact import ContactConfig, ContactState, GripperContactDetector


def enabled_config(**overrides):
    values = {
        "enabled": True,
        "closing_direction": 1,
        "minimum_load_raw": 100,
        "minimum_current_raw": 20,
        "minimum_position_error_ticks": 12,
        "empty_closed_position_ticks": 200,
        "minimum_object_gap_ticks": 10,
        "confirmation_samples": 3,
    }
    values.update(overrides)
    return ContactConfig.from_dict(values)


class ContactConfigTests(unittest.TestCase):
    def test_rejects_uncalibrated_direction(self):
        with self.assertRaisesRegex(ValueError, "closing_direction"):
            enabled_config(closing_direction=0)

    def test_rejects_zero_threshold(self):
        with self.assertRaisesRegex(ValueError, "thresholds"):
            enabled_config(minimum_load_raw=0)


class GripperContactDetectorTests(unittest.TestCase):
    def test_disabled_detector_never_claims_contact(self):
        config = ContactConfig.from_dict(
            {
                "enabled": False,
                "closing_direction": 1,
                "minimum_load_raw": 1,
                "minimum_current_raw": 1,
                "minimum_position_error_ticks": 1,
                "empty_closed_position_ticks": 0,
                "minimum_object_gap_ticks": 1,
                "confirmation_samples": 1,
            }
        )
        detector = GripperContactDetector(config)
        detector.command(10)
        state = detector.observe(
            position_ticks=100, target_ticks=200, load_raw=1000, current_raw=1000
        )
        self.assertEqual(ContactState.DISABLED, state)

    def test_requires_all_evidence_for_sustained_samples(self):
        detector = GripperContactDetector(enabled_config())
        self.assertEqual(ContactState.CLOSING, detector.command(4))
        weak_samples = (
            (90, 20, 15),
            (100, 19, 15),
            (100, 20, 11),
        )
        for load, current, error in weak_samples:
            state = detector.observe(
                position_ticks=100,
                target_ticks=100 + error,
                load_raw=load,
                current_raw=current,
            )
            self.assertEqual(ContactState.CLOSING, state)

        for expected in (ContactState.CLOSING, ContactState.CLOSING, ContactState.GRIPPED):
            state = detector.observe(
                position_ticks=100, target_ticks=115, load_raw=-100, current_raw=20
            )
            self.assertEqual(expected, state)

    def test_nonmatching_sample_breaks_confirmation_streak(self):
        detector = GripperContactDetector(enabled_config())
        detector.command(1)
        strong = dict(position_ticks=100, target_ticks=115, load_raw=100, current_raw=20)
        detector.observe(**strong)
        detector.observe(**strong)
        detector.observe(position_ticks=100, target_ticks=115, load_raw=99, current_raw=20)
        self.assertEqual(ContactState.CLOSING, detector.observe(**strong))

    def test_empty_mechanical_close_is_not_reported_as_object(self):
        detector = GripperContactDetector(enabled_config(confirmation_samples=1))
        detector.command(1)
        state = detector.observe(
            position_ticks=195, target_ticks=215, load_raw=200, current_raw=40
        )
        self.assertEqual(ContactState.CLOSING, state)

    def test_opening_releases_latched_contact(self):
        detector = GripperContactDetector(enabled_config(confirmation_samples=1))
        detector.command(1)
        detector.observe(position_ticks=100, target_ticks=115, load_raw=100, current_raw=20)
        self.assertEqual(ContactState.GRIPPED, detector.state)
        self.assertEqual(ContactState.UNKNOWN, detector.command(-1))

    def test_wrong_command_direction_does_not_arm_detector(self):
        detector = GripperContactDetector(enabled_config(closing_direction=-1))
        detector.command(5)
        state = detector.observe(
            position_ticks=100, target_ticks=115, load_raw=100, current_raw=20
        )
        self.assertEqual(ContactState.UNKNOWN, state)

    def test_negative_closing_direction_uses_gap_in_correct_direction(self):
        detector = GripperContactDetector(
            enabled_config(
                closing_direction=-1,
                empty_closed_position_ticks=100,
                confirmation_samples=1,
            )
        )
        detector.command(-5)
        state = detector.observe(
            position_ticks=130, target_ticks=115, load_raw=100, current_raw=20
        )
        self.assertEqual(ContactState.GRIPPED, state)


if __name__ == "__main__":
    unittest.main()
