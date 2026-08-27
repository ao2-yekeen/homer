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
            if self.state is not ContactState.GRIPPED:
                self.state = ContactState.CLOSING
        else:
            # Opening is explicit evidence that a previous grip is released.
            self._closing = False
            self._matching_samples = 0
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
        if not self.config.enabled or self.state is ContactState.GRIPPED:
            return self.state
        if not self._closing:
            self._matching_samples = 0
            return self.state

        position_error = abs(target_ticks - position_ticks)
        object_gap = (
            self.config.empty_closed_position_ticks - position_ticks
        ) * self.config.closing_direction
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
