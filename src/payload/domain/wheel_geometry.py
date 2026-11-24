from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class WheelTransform2D:
    """
    Wheel centre and positive rolling direction in the payload frame.
    """

    x: float
    y: float
    yaw: float
    radius: float

    def __post_init__(self) -> None:
        for name in ("x", "y", "yaw", "radius"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"wheel {name} must be a finite number")
        if self.radius <= 0:
            raise ValueError("wheel radius must be positive")
