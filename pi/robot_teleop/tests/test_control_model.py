import unittest

from robot_teleop.control_model import (
    ArmController,
    BaseController,
    GamepadLayout,
    GamepadSnapshot,
    NeckController,
    ModeSelector,
    RobotMode,
)


class ControlModelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = GamepadLayout()

    def test_base_is_stopped_until_teleop_and_deadman_are_active(self) -> None:
        controller = BaseController(self.layout)
        snapshot = GamepadSnapshot((1.0, 1.0), (0,) * 8)
        self.assertEqual(controller.command(snapshot, RobotMode.AUTONOMOUS).linear, 0.0)
        self.assertEqual(controller.command(snapshot, RobotMode.TELEOP).linear, 0.0)
        active = GamepadSnapshot((1.0, 1.0), (0, 0, 0, 0, 1))
        self.assertGreater(controller.command(active, RobotMode.TELEOP).linear, 0.0)

    def test_arm_is_stopped_without_teleop_or_arm_deadman(self) -> None:
        controller = ArmController(self.layout, ticks_per_second=10.0)
        snapshot = GamepadSnapshot((1.0,) * 8, (0,) * 8)
        self.assertEqual(controller.command(snapshot, RobotMode.TELEOP, 1.0), (0,) * 6)
        enabled = GamepadSnapshot((1.0,) * 8, (0, 0, 0, 0, 0, 1))
        self.assertEqual(controller.command(enabled, RobotMode.AUTONOMOUS, 1.0), (0,) * 6)

    def test_arm_rate_and_per_cycle_limit_are_enforced(self) -> None:
        controller = ArmController(self.layout, ticks_per_second=100.0, max_ticks_per_cycle=2)
        enabled = GamepadSnapshot((1.0,) * 8, (0, 0, 0, 0, 0, 1))
        self.assertEqual(controller.command(enabled, RobotMode.TELEOP, 1.0), (2,) * 6)

    def test_mode_requires_a_rising_edge(self) -> None:
        selector = ModeSelector(self.layout)
        neutral = GamepadSnapshot((), (0,) * 8)
        select = GamepadSnapshot((), (1, 0, 0, 0, 0, 0, 0, 0))  # A
        selector.update(neutral)
        self.assertEqual(selector.update(select), RobotMode.TELEOP)
        self.assertIsNone(selector.update(select))

        selector.update(neutral)
        autonomous = GamepadSnapshot((), (0, 1, 0, 0, 0, 0, 0, 0))  # B
        self.assertEqual(selector.update(autonomous), RobotMode.AUTONOMOUS)

    def test_neck_stays_in_tested_range(self) -> None:
        controller = NeckController(self.layout, angle=149.0, degrees_per_second=100.0)
        enabled = GamepadSnapshot((0.0, 0.0, 0.0, 0.0, 0.0, -1.0), (0, 0, 0, 0, 0, 1))
        self.assertEqual(controller.command(enabled, RobotMode.TELEOP, 1.0), 150)


if __name__ == "__main__":
    unittest.main()
