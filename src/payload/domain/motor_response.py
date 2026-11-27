from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol


def _is_number(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _validate_unit_interval(value: float, name: str) -> None:
    if not _is_number(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be a number in [0, 1]")


class MotorResponse(Protocol):
    """
    Linearizer of a motor: tell which motor command gives the wanted speed.

    A real motor does not respond linearly to its command (a PWM duty cycle): below
    some duty it does not turn at all (the dead zone), and above it the speed is not
    exactly proportional to the duty. A motor response is the inverse of that
    behaviour. It always implements a monotonic function from [0, 1] to [0, 1], so
    actuators can call it without knowing which model is behind it.
    """

    def inverse_response(self, speed_fraction: float) -> float:
        """
        Return the motor command in [0, 1] that reaches ``speed_fraction`` in [0, 1].

        ``speed_fraction`` is the wanted speed divided by the speed at full command
        (``max_surface_velocity``).
        """
        ...


@dataclass(frozen=True, slots=True)
class LinearMotorResponse:
    """The ideal motor: the command equals the wanted speed fraction."""

    def inverse_response(self, speed_fraction: float) -> float:
        _validate_unit_interval(speed_fraction, "speed fraction")
        return float(speed_fraction)


@dataclass(frozen=True, slots=True)
class SimpleDeadzoneMotorResponse:
    """
    A motor that does not turn up to ``dead_zone``, then speeds up in a straight line.

    ``dead_zone`` is the duty at which the wheel just starts to turn. Above it, the
    speed grows linearly up to full speed at full duty.
    """

    dead_zone: float

    def __post_init__(self) -> None:
        if not _is_number(self.dead_zone) or not 0.0 <= self.dead_zone < 1.0:
            raise ValueError("dead_zone must be a number in [0, 1)")

    def inverse_response(self, speed_fraction: float) -> float:
        _validate_unit_interval(speed_fraction, "speed fraction")
        if speed_fraction == 0.0:
            return 0.0
        return self.dead_zone + speed_fraction * (1.0 - self.dead_zone)


@dataclass(frozen=True, slots=True)
class MonotonousAffineSegmentsMotorResponse:
    """
    A motor described by measured ``(duty, speed_fraction)`` points joined by straight lines.

    There is no separate dead zone: it is part of the points (for example
    ``(0.2, 0.0)`` says that the wheel only starts to turn at 20 % duty).

    Duties strictly increase and speed fractions never decrease. The points
    ``(0, 0)`` and ``(1, 1)`` are added when missing, so a motor at rest stays at
    rest and full duty gives full speed.
    """

    points: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        if not isinstance(self.points, tuple):
            raise ValueError("points must be a tuple of (duty, speed_fraction) pairs")

        previous_duty, previous_fraction = 0.0, 0.0
        for index, point in enumerate(self.points):
            if not isinstance(point, tuple) or len(point) != 2 or not all(_is_number(value) for value in point):
                raise ValueError("points must be (duty, speed_fraction) pairs of numbers")
            duty, fraction = point
            if not 0.0 <= duty <= 1.0 or not 0.0 <= fraction <= 1.0:
                raise ValueError("point values must be in [0, 1]")
            if duty == 0.0 and fraction != 0.0:
                raise ValueError("the point at duty 0 must have speed fraction 0")
            if duty == 1.0 and fraction != 1.0:
                raise ValueError("the point at duty 1 must have speed fraction 1")
            if index > 0 and duty <= previous_duty:
                raise ValueError("point duties must strictly increase")
            if fraction < previous_fraction:
                raise ValueError("point speed fractions must not decrease")
            previous_duty, previous_fraction = duty, fraction

    def _knots(self) -> list[tuple[float, float]]:
        knots = list(self.points)
        if not knots or knots[0][0] > 0.0:
            knots.insert(0, (0.0, 0.0))
        if knots[-1][0] < 1.0:
            knots.append((1.0, 1.0))
        return knots

    def inverse_response(self, speed_fraction: float) -> float:
        _validate_unit_interval(speed_fraction, "speed fraction")
        if speed_fraction == 0.0:
            return 0.0

        knots = self._knots()
        for (duty_low, fraction_low), (duty_high, fraction_high) in zip(knots, knots[1:]):
            if speed_fraction <= fraction_high:
                if fraction_high == fraction_low:
                    return duty_high
                ratio = (speed_fraction - fraction_low) / (fraction_high - fraction_low)
                return min(1.0, duty_low + ratio * (duty_high - duty_low))
        return 1.0
