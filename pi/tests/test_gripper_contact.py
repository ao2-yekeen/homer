import unittest

from gripper_contact import (
    AutoCloseAction,
    AutoCloseConfig,
    AutoCloseState,
    AutomaticCloseController,
    ContactConfig,
    ContactState,
    GripperContactDetector,
)


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
        "maximum_load_raw": 200,
        "maximum_current_raw": 40,
    }
    values.update(overrides)
    return ContactConfig.from_dict(values)


def auto_config(**overrides):
    values = {
        "automatic_close": {
            "enabled": True,
            "speed": 10,
            "acceleration": 3,
            "maximum_duration_s": 12.0,
            "endpoint_tolerance_ticks": 3,
            "relief_ticks": 0,
        }
    }
    values["automatic_close"].update(overrides)
    return AutoCloseConfig.from_dict(values)


def installed_calibration():
    return enabled_config(
        closing_direction=-1,
        minimum_load_raw=90,
        minimum_current_raw=4,
        minimum_position_error_ticks=20,
        empty_closed_position_ticks=2042,
        minimum_object_gap_ticks=100,
        confirmation_samples=2,
        maximum_load_raw=140,
        maximum_current_raw=10,
    )


class ContactConfigTests(unittest.TestCase):
    def test_rejects_uncalibrated_direction(self):
        with self.assertRaisesRegex(ValueError, "closing_direction"):
            enabled_config(closing_direction=0)

    def test_rejects_zero_threshold(self):
        with self.assertRaisesRegex(ValueError, "thresholds"):
            enabled_config(minimum_load_raw=0)


class AutoCloseConfigTests(unittest.TestCase):
    def test_rejects_unbounded_duration(self):
        with self.assertRaisesRegex(ValueError, "maximum_duration"):
            auto_config(maximum_duration_s=31)

    def test_rejects_large_unverified_relief(self):
        with self.assertRaisesRegex(ValueError, "relief_ticks"):
            auto_config(relief_ticks=11)


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
                "maximum_load_raw": 2,
                "maximum_current_raw": 2,
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

    def test_stop_disarms_closing_but_preserves_grip_latch(self):
        detector = GripperContactDetector(enabled_config(confirmation_samples=1))
        detector.command(1)
        self.assertEqual(ContactState.UNKNOWN, detector.stop())
        detector.command(1)
        detector.observe(position_ticks=100, target_ticks=115, load_raw=100, current_raw=20)
        self.assertEqual(ContactState.GRIPPED, detector.stop())

    def test_protective_ceiling_stops_on_one_sample_and_latches(self):
        detector = GripperContactDetector(enabled_config())
        detector.command(1)
        state = detector.observe(
            position_ticks=100,
            target_ticks=115,
            load_raw=200,
            current_raw=20,
        )
        self.assertEqual(ContactState.PROTECTIVE_STOP, state)
        self.assertEqual(ContactState.PROTECTIVE_STOP, detector.command(1))
        self.assertEqual(ContactState.UNKNOWN, detector.command(-1))

    def test_installed_calibration_separates_recorded_empty_and_contact(self):
        detector = GripperContactDetector(installed_calibration())
        detector.command(-1)
        for _ in range(2):
            state = detector.observe(
                position_ticks=2200,
                target_ticks=2042,
                load_raw=84,
                current_raw=4,
            )
            self.assertEqual(ContactState.CLOSING, state)
        for expected in (ContactState.CLOSING, ContactState.GRIPPED):
            state = detector.observe(
                position_ticks=2212,
                target_ticks=2042,
                load_raw=100,
                current_raw=4,
            )
            self.assertEqual(expected, state)

    def test_installed_calibration_does_not_call_empty_endpoint_an_object(self):
        detector = GripperContactDetector(installed_calibration())
        detector.command(-1)
        state = detector.observe(
            position_ticks=2042,
            target_ticks=2034,
            load_raw=200,
            current_raw=20,
        )
        self.assertEqual(ContactState.CLOSING, state)

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


class AutomaticCloseControllerTests(unittest.TestCase):
    def controller(self):
        return AutomaticCloseController(
            auto_config(),
            closing_direction=-1,
            empty_closed_position_ticks=2042,
        )

    def test_contact_immediately_requests_hold(self):
        controller = self.controller()
        self.assertEqual(AutoCloseState.CLOSING, controller.start(10.0))
        action = controller.observe(
            position_ticks=2190,
            contact_state=ContactState.GRIPPED,
            now_s=10.2,
        )
        self.assertEqual(AutoCloseAction.HOLD_GRIPPED, action)
        self.assertEqual(AutoCloseState.GRIPPED, controller.state)

    def test_empty_endpoint_stops_without_claiming_grip(self):
        controller = self.controller()
        controller.start(10.0)
        action = controller.observe(
            position_ticks=2044,
            contact_state=ContactState.CLOSING,
            now_s=11.0,
        )
        self.assertEqual(AutoCloseAction.HOLD_EMPTY, action)
        self.assertEqual(AutoCloseState.EMPTY, controller.state)

    def test_timeout_requests_hold(self):
        controller = self.controller()
        controller.start(10.0)
        action = controller.observe(
            position_ticks=2200,
            contact_state=ContactState.CLOSING,
            now_s=22.0,
        )
        self.assertEqual(AutoCloseAction.HOLD_TIMEOUT, action)
        self.assertEqual(AutoCloseState.TIMED_OUT, controller.state)

    def test_protective_stop_requests_immediate_hold(self):
        controller = self.controller()
        controller.start(10.0)
        action = controller.observe(
            position_ticks=2190,
            contact_state=ContactState.PROTECTIVE_STOP,
            now_s=10.1,
        )
        self.assertEqual(AutoCloseAction.HOLD_PROTECTIVE_STOP, action)
        self.assertEqual(AutoCloseState.PROTECTIVE_STOP, controller.state)

    def test_operator_stop_and_open_reset(self):
        controller = self.controller()
        controller.start(10.0)
        self.assertEqual(AutoCloseState.STOPPED, controller.stop())
        self.assertEqual(AutoCloseState.IDLE, controller.reset())
        self.assertEqual(
            AutoCloseAction.NONE,
            controller.observe(
                position_ticks=2200,
                contact_state=ContactState.GRIPPED,
                now_s=30.0,
            ),
        )


if __name__ == "__main__":
    unittest.main()
