from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BodyVelocity:
    """
    Desired planar velocity in the payload body frame.
    """

    vx_m_s: float
    vy_m_s: float
    yaw_rate_rad_s: float

    def __post_init__(self) -> None:
        for name in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name} must be a finite number")
