from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from payload.domain.body_velocity import BodyVelocity


ManualControlCommandType = Literal["arm", "drive", "stop", "disarm", "status"]


@dataclass(frozen=True, slots=True)
class ManualControlCommand:
    session_id: str
    sequence: int
    command_type: ManualControlCommandType
    velocity: BodyVelocity | None = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.session_id, str)
            or not self.session_id
            or len(self.session_id) > 64
        ):
            raise ValueError(
                "manual-control session_id must be a non-empty string of at most 64 characters"
            )
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence < 0:
            raise ValueError("manual-control sequence must be a non-negative integer")
        if not isinstance(self.command_type, str) or self.command_type not in {
            "arm", "drive", "stop", "disarm", "status"
        }:
            raise ValueError("unsupported manual-control command type")
        if self.command_type == "drive":
            if not isinstance(self.velocity, BodyVelocity):
                raise ValueError("drive commands require a BodyVelocity")
        elif self.velocity is not None:
            raise ValueError(f"{self.command_type} commands must not include a velocity")
