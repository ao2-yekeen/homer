"""Pure, hardware-independent gamepad control rules.

Keeping these rules free of ROS and serial I/O makes the safety behaviour
unit-testable. The ROS node adapts Joy messages to these objects.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import copysign, trunc
from typing import Sequence


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class RobotMode(Enum):
    TELEOP = "teleop"
    AUTONOMOUS = "autonomous"


@dataclass(frozen=True)
class AxisBinding:
    """Maps one gamepad axis to an output, including its direction."""

    axis: int
    direction: float = 1.0


@dataclass(frozen=True)
class GamepadLayout:
    """Controller layout for the intended Aurora/Xbox-style operator profile.

    Button and axis numbers are isolated here so receiver-specific verification
    can change the profile without touching any motion or safety behaviour.
    """

    base_linear: AxisBinding = AxisBinding(1)
    base_angular: AxisBinding = AxisBinding(0)
    arm: tuple[AxisBinding, ...] = (
        AxisBinding(0), AxisBinding(1), AxisBinding(3),
        AxisBinding(4), AxisBinding(6), AxisBinding(7),
    )
    # Both trigger axes rest at +1 and decrease when pressed.
    neck_up: AxisBinding = AxisBinding(5, -1.0)     # right trigger
    neck_down: AxisBinding = AxisBinding(2, -1.0)   # left trigger
    base_enable_button: int = 4   # L1, confirmed on the Aurora receiver
    arm_enable_button: int = 5    # R1; must be verified before enabling
    teleop_button: int = 0        # A; must be verified on the receiver
    autonomous_button: int = 1    # B; must be verified on the receiver


def axis_value(axes: Sequence[float], binding: AxisBinding, deadzone: float) -> float:
    """Return a rescaled axis value in [-1, 1], or zero inside the deadzone."""
    value = axes[binding.axis] if 0 <= binding.axis < len(axes) else 0.0
    value *= binding.direction
    if abs(value) <= deadzone:
        return 0.0
    return copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def button_pressed(buttons: Sequence[int], button: int) -> bool:
    return 0 <= button < len(buttons) and bool(buttons[button])


@dataclass
class GamepadSnapshot:
    axes: tuple[float, ...] = ()
    buttons: tuple[int, ...] = ()


@dataclass
class ModeSelector:
    """Edge-triggered mode switch that starts in the safest state."""

    layout: GamepadLayout
    mode: RobotMode = RobotMode.AUTONOMOUS
    _previous_buttons: tuple[int, ...] = ()

    def update(self, snapshot: GamepadSnapshot) -> RobotMode | None:
        previous = self._previous_buttons
        self._previous_buttons = snapshot.buttons
        if len(previous) != len(snapshot.buttons):
            return None
        for button, requested_mode in (
            (self.layout.teleop_button, RobotMode.TELEOP),
            (self.layout.autonomous_button, RobotMode.AUTONOMOUS),
        ):
            if button_pressed(snapshot.buttons, button) and not button_pressed(previous, button):
                self.mode = requested_mode
                return requested_mode
        return None


@dataclass(frozen=True)
class BaseCommand:
    linear: float = 0.0
    angular: float = 0.0


@dataclass
class BaseController:
    layout: GamepadLayout
    deadzone: float = 0.08
    max_linear: float = 0.7
    max_angular: float = 0.4

    def command(self, snapshot: GamepadSnapshot, mode: RobotMode) -> BaseCommand:
        if mode is not RobotMode.TELEOP or not button_pressed(
            snapshot.buttons, self.layout.base_enable_button
        ):
            return BaseCommand()
        return BaseCommand(
            linear=axis_value(snapshot.axes, self.layout.base_linear, self.deadzone) * self.max_linear,
            angular=axis_value(snapshot.axes, self.layout.base_angular, self.deadzone) * self.max_angular,
        )


@dataclass
class ArmController:
    """Converts held axes to slow relative SO-ARM joint movements."""

    layout: GamepadLayout
    deadzone: float = 0.12
    ticks_per_second: float = 50.0
    max_ticks_per_cycle: int = 3
    _residual_ticks: list[float] = field(default_factory=lambda: [0.0] * len(ARM_JOINTS))

    def command(
        self, snapshot: GamepadSnapshot, mode: RobotMode, elapsed_seconds: float
    ) -> tuple[int, ...]:
        if mode is not RobotMode.TELEOP or not button_pressed(
            snapshot.buttons, self.layout.arm_enable_button
        ):
            self._residual_ticks = [0.0] * len(ARM_JOINTS)
            return (0,) * len(ARM_JOINTS)

        deltas: list[int] = []
        for index, binding in enumerate(self.layout.arm):
            requested = axis_value(snapshot.axes, binding, self.deadzone)
            total = self._residual_ticks[index] + requested * self.ticks_per_second * elapsed_seconds
            whole_ticks = max(-self.max_ticks_per_cycle, min(self.max_ticks_per_cycle, trunc(total)))
            self._residual_ticks[index] = total - whole_ticks
            deltas.append(whole_ticks)
        return tuple(deltas)


@dataclass
class NeckController:
    """Maintains the neck command inside its mechanically tested limits."""

    layout: GamepadLayout
    deadzone: float = 0.12
    # Moderate slew rate limits current transients without making the neck
    # feel unresponsive under normal teleoperation.
    degrees_per_second: float = 10.0
    minimum_angle: float = 80.0
    maximum_angle: float = 150.0
    angle: float = 90.0

    def command(self, snapshot: GamepadSnapshot, mode: RobotMode, elapsed_seconds: float) -> int | None:
        if mode is not RobotMode.TELEOP or not button_pressed(
            snapshot.buttons, self.layout.arm_enable_button
        ):
            return None
        direction = axis_value(snapshot.axes, self.layout.neck_up, self.deadzone)
        direction -= axis_value(snapshot.axes, self.layout.neck_down, self.deadzone)
        if not direction:
            return None
        self.angle = min(
            self.maximum_angle,
            max(self.minimum_angle, self.angle + direction * self.degrees_per_second * elapsed_seconds),
        )
        return round(self.angle)
