from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True, slots=True)
class WheelActuatorCapabilities:
    """Static or currently available wheel-actuator limits, in SI units."""

    max_surface_velocity: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.max_surface_velocity, bool)
            or not isinstance(self.max_surface_velocity, (int, float))
            or not math.isfinite(self.max_surface_velocity)
            or self.max_surface_velocity <= 0
        ):
            raise ValueError("max_surface_velocity must be finite and positive")

    @property
    def min_surface_velocity(self) -> float:
        return -self.max_surface_velocity


class WheelActuator(Protocol):
    """Common asynchronous interface for simulated and physical wheel motors."""

    @property
    def capabilities(self) -> WheelActuatorCapabilities:
        ...

    async def set_surface_velocity(self, velocity_m_s: float) -> None:
        ...

    async def stop(self) -> None:
        ...

    async def close(self) -> None:
        ...
