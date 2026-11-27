from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal


ManualControlCommandType = Literal["arm", "drive", "stop", "disarm", "status"]


@dataclass(frozen=True, slots=True)
class ReferenceVelocity:
    """Translation in the static UWB-anchor reference frame, plus yaw rate."""

    vx_m_s: float
    vy_m_s: float
    yaw_rate_rad_s: float

    def __post_init__(self) -> None:
        for name in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")


@dataclass(frozen=True, slots=True)
class ManualControlCommand:
    session_id: str
    sequence: int
    command_type: ManualControlCommandType
    velocity: ReferenceVelocity | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.session_id, str) or not self.session_id or len(self.session_id) > 64:
            raise ValueError("manual-control session_id must be a non-empty string of at most 64 characters")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence < 0:
            raise ValueError("manual-control sequence must be a non-negative integer")
        if not isinstance(self.command_type, str) or self.command_type not in {
            "arm",
            "drive",
            "stop",
            "disarm",
            "status",
        }:
            raise ValueError("unsupported manual-control command type")
        if self.command_type == "drive":
            if not isinstance(self.velocity, ReferenceVelocity):
                raise ValueError("drive commands require a ReferenceVelocity")
        elif self.velocity is not None:
            raise ValueError(f"{self.command_type} commands must not include a velocity")
