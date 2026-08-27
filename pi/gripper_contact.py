"""Pure, hardware-independent gripper contact detection.

The detector deliberately reports contact rather than declaring an entire
grasp successful. A successful grasp still needs post-lift evidence, such as
vision confirming that the object moved with the gripper.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class ContactState(Enum):
    DISABLED = "DISABLED"
    UNKNOWN = "UNKNOWN"
    CLOSING = "CLOSING"
    GRIPPED = "GRIPPED"
    PROTECTIVE_STOP = "PROTECTIVE_STOP"


class AutoCloseState(Enum):
    IDLE = "IDLE"
    CLOSING = "CLOSING"
    GRIPPED = "GRIPPED"
    EMPTY = "EMPTY"
    STOPPED = "STOPPED"
    TIMED_OUT = "TIMED_OUT"
    PROTECTIVE_STOP = "PROTECTIVE_STOP"


class AutoCloseAction(Enum):
    NONE = "NONE"
    HOLD_GRIPPED = "HOLD_GRIPPED"
    HOLD_EMPTY = "HOLD_EMPTY"
    HOLD_TIMEOUT = "HOLD_TIMEOUT"
    HOLD_PROTECTIVE_STOP = "HOLD_PROTECTIVE_STOP"


@dataclass(frozen=True)
class ContactConfig:
    enabled: bool
    closing_direction: int
    minimum_load_raw: int
    minimum_current_raw: int
    minimum_position_error_ticks: int
    empty_closed_position_ticks: int
    minimum_object_gap_ticks: int
    confirmation_samples: int
    maximum_load_raw: int
    maximum_current_raw: int

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> "ContactConfig":
        config = cls(
            enabled=values.get("enabled") is True,
            closing_direction=int(values.get("closing_direction", 0)),
            minimum_load_raw=int(values.get("minimum_load_raw", 0)),
            minimum_current_raw=int(values.get("minimum_current_raw", 0)),
            minimum_position_error_ticks=int(
                values.get("minimum_position_error_ticks", 0)
            ),
            empty_closed_position_ticks=int(
                values.get("empty_closed_position_ticks", 0)
            ),
            minimum_object_gap_ticks=int(values.get("minimum_object_gap_ticks", 0)),
            confirmation_samples=int(values.get("confirmation_samples", 0)),
            maximum_load_raw=int(values.get("maximum_load_raw", 0)),
            maximum_current_raw=int(values.get("maximum_current_raw", 0)),
        )
        if config.closing_direction not in (-1, 1):
            raise ValueError("closing_direction must be -1 or 1")
        if min(
            config.minimum_load_raw,
            config.minimum_current_raw,
            config.minimum_position_error_ticks,
            config.minimum_object_gap_ticks,
            config.confirmation_samples,
        ) <= 0:
            raise ValueError("all contact thresholds must be positive")
        if not 0 <= config.empty_closed_position_ticks <= 4095:
            raise ValueError("empty_closed_position_ticks must be in 0..4095")
        if config.maximum_load_raw <= config.minimum_load_raw:
            raise ValueError("maximum_load_raw must exceed minimum_load_raw")
        if config.maximum_current_raw <= config.minimum_current_raw:
            raise ValueError("maximum_current_raw must exceed minimum_current_raw")
        return config


@dataclass(frozen=True)
class AutoCloseConfig:
    enabled: bool
    speed: int
    acceleration: int
    maximum_duration_s: float
    endpoint_tolerance_ticks: int
    relief_ticks: int

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> "AutoCloseConfig":
        auto = values.get("automatic_close", {})
        if not isinstance(auto, dict):
            raise ValueError("automatic_close must be an object")
        config = cls(
            enabled=auto.get("enabled") is True,
            speed=int(auto.get("speed", 0)),
            acceleration=int(auto.get("acceleration", 0)),
            maximum_duration_s=float(auto.get("maximum_duration_s", 0)),
            endpoint_tolerance_ticks=int(auto.get("endpoint_tolerance_ticks", 0)),
            relief_ticks=int(auto.get("relief_ticks", 0)),
        )
        if not 1 <= config.speed <= 1000:
            raise ValueError("automatic close speed must be in 1..1000")
        if not 0 <= config.acceleration <= 254:
            raise ValueError("automatic close acceleration must be in 0..254")
        if not 0 < config.maximum_duration_s <= 30:
            raise ValueError("automatic close maximum_duration_s must be in (0, 30]")
        if not 1 <= config.endpoint_tolerance_ticks <= 20:
            raise ValueError("automatic close endpoint_tolerance_ticks must be in 1..20")
        if not 0 <= config.relief_ticks <= 10:
            raise ValueError("automatic close relief_ticks must be in 0..10")
        return config


class GripperContactDetector:
    """Latch contact after multiple feedback signals agree while closing."""

    def __init__(self, config: ContactConfig) -> None:
        self.config = config
        self.state = ContactState.UNKNOWN if config.enabled else ContactState.DISABLED
        self._closing = False
        self._matching_samples = 0

    def command(self, delta_ticks: int) -> ContactState:
        """Track gripper intent without issuing any hardware command."""
        if not self.config.enabled or delta_ticks == 0:
            return self.state
        direction = 1 if delta_ticks > 0 else -1
        if direction == self.config.closing_direction:
            self._closing = True
            if self.state not in (ContactState.GRIPPED, ContactState.PROTECTIVE_STOP):
                self.state = ContactState.CLOSING
        else:
            # Opening is explicit evidence that a previous grip is released.
            self._closing = False
            self._matching_samples = 0
            self.state = ContactState.UNKNOWN
        return self.state

    def stop(self) -> ContactState:
        """Disarm closing evidence without clearing a confirmed grip latch."""
        self._closing = False
        self._matching_samples = 0
        if self.state is ContactState.CLOSING:
            self.state = ContactState.UNKNOWN
        return self.state

    def observe(
        self,
        *,
        position_ticks: int,
        target_ticks: int,
        load_raw: int,
        current_raw: int,
    ) -> ContactState:
        if not self.config.enabled or self.state in (
            ContactState.GRIPPED,
            ContactState.PROTECTIVE_STOP,
        ):
            return self.state
        if not self._closing:
            self._matching_samples = 0
            return self.state

        position_error = abs(target_ticks - position_ticks)
        object_gap = (
            self.config.empty_closed_position_ticks - position_ticks
        ) * self.config.closing_direction
        if (
            object_gap >= self.config.minimum_object_gap_ticks
            and position_error >= self.config.minimum_position_error_ticks
            and (
                abs(load_raw) >= self.config.maximum_load_raw
                or abs(current_raw) >= self.config.maximum_current_raw
            )
        ):
            self.state = ContactState.PROTECTIVE_STOP
            return self.state
        evidence_matches = (
            abs(load_raw) >= self.config.minimum_load_raw
            and abs(current_raw) >= self.config.minimum_current_raw
            and position_error >= self.config.minimum_position_error_ticks
            and object_gap >= self.config.minimum_object_gap_ticks
        )
        self._matching_samples = self._matching_samples + 1 if evidence_matches else 0
        if self._matching_samples >= self.config.confirmation_samples:
            self.state = ContactState.GRIPPED
        return self.state


class AutomaticCloseController:
    """Hardware-independent terminal-state logic for a guarded close."""

    def __init__(
        self,
        config: AutoCloseConfig,
        *,
        closing_direction: int,
        empty_closed_position_ticks: int,
    ) -> None:
        self.config = config
        self.closing_direction = closing_direction
        self.empty_closed_position_ticks = empty_closed_position_ticks
        self.state = AutoCloseState.IDLE
        self._deadline_s = 0.0

    @property
    def active(self) -> bool:
        return self.state is AutoCloseState.CLOSING

    def start(self, now_s: float) -> AutoCloseState:
        self.state = AutoCloseState.CLOSING
        self._deadline_s = now_s + self.config.maximum_duration_s
        return self.state

    def stop(self) -> AutoCloseState:
        if self.active:
            self.state = AutoCloseState.STOPPED
        return self.state

    def reset(self) -> AutoCloseState:
        self.state = AutoCloseState.IDLE
        self._deadline_s = 0.0
        return self.state

    def observe(
        self,
        *,
        position_ticks: int,
        contact_state: ContactState,
        now_s: float,
    ) -> AutoCloseAction:
        if not self.active:
            return AutoCloseAction.NONE
        if contact_state is ContactState.GRIPPED:
            self.state = AutoCloseState.GRIPPED
            return AutoCloseAction.HOLD_GRIPPED
        if contact_state is ContactState.PROTECTIVE_STOP:
            self.state = AutoCloseState.PROTECTIVE_STOP
            return AutoCloseAction.HOLD_PROTECTIVE_STOP
        remaining_ticks = (
            self.empty_closed_position_ticks - position_ticks
        ) * self.closing_direction
        if remaining_ticks <= self.config.endpoint_tolerance_ticks:
            self.state = AutoCloseState.EMPTY
            return AutoCloseAction.HOLD_EMPTY
        if now_s >= self._deadline_s:
            self.state = AutoCloseState.TIMED_OUT
            return AutoCloseAction.HOLD_TIMEOUT
        return AutoCloseAction.NONE
