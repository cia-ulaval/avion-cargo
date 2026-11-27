from __future__ import annotations

import math
from dataclasses import dataclass


def _is_number(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


@dataclass(frozen=True, slots=True)
class MotorResponse:
    """
    Map a wanted speed to the PWM duty cycle that really produces it.

    A DC motor does not respond linearly to PWM: below some duty cycle (the dead
    zone) it does not turn at all, and above it the speed is not exactly
    proportional to the duty. This class inverts that behaviour so that callers can
    ask for a speed and get a speed.

    ``dead_zone`` is the duty at which the wheel just starts to turn.

    ``calibration`` holds measured ``(duty, speed_fraction)`` points above the dead
    zone, where ``speed_fraction`` is the measured speed divided by the speed at
    100 % duty (``max_surface_velocity``). Both values must strictly increase.
    Without points, the response is a straight line from the dead zone to full duty.

    With the defaults (no dead zone, no points), duty equals speed fraction.
    """

    dead_zone: float = 0.0
    calibration: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        if not _is_number(self.dead_zone) or not 0.0 <= self.dead_zone < 1.0:
            raise ValueError("dead_zone must be a number in [0, 1)")

        previous_duty = self.dead_zone
        previous_fraction = 0.0
        for point in self.calibration:
            if len(point) != 2 or not all(_is_number(value) for value in point):
                raise ValueError("calibration points must be (duty, speed_fraction) pairs of numbers")
            duty, fraction = point
            if not previous_duty < duty <= 1.0:
                raise ValueError("calibration duties must strictly increase, above dead_zone and up to 1")
            if not previous_fraction < fraction <= 1.0:
                raise ValueError("calibration speed fractions must strictly increase, above 0 and up to 1")
            if duty == 1.0 and fraction != 1.0:
                raise ValueError("a calibration point at duty 1 must have speed fraction 1")
            previous_duty, previous_fraction = duty, fraction

    def _knots(self) -> list[tuple[float, float]]:
        knots = [(self.dead_zone, 0.0), *self.calibration]
        if knots[-1][0] < 1.0:
            knots.append((1.0, 1.0))
        return knots

    def speed_fraction_for(self, duty: float) -> float:
        """
        Return the speed fraction in [0, 1] that this motor reaches at a duty in [0, 1].

        This is the motor itself (the inverse of ``duty_for``): it is what a simulated
        motor does, while ``duty_for`` is what the controller does to compensate it.
        """
        if not _is_number(duty) or not 0.0 <= duty <= 1.0:
            raise ValueError("duty must be a number in [0, 1]")
        if duty <= self.dead_zone:
            return 0.0

        knots = self._knots()
        for (duty_low, fraction_low), (duty_high, fraction_high) in zip(knots, knots[1:]):
            if duty <= duty_high:
                ratio = (duty - duty_low) / (duty_high - duty_low)
                return min(1.0, fraction_low + ratio * (fraction_high - fraction_low))
        return 1.0

    def duty_for(self, speed_fraction: float) -> float:
        """Return the duty in [0, 1] for a speed fraction in [0, 1]."""
        if not _is_number(speed_fraction) or not 0.0 <= speed_fraction <= 1.0:
            raise ValueError("speed fraction must be a number in [0, 1]")
        if speed_fraction == 0.0:
            return 0.0

        knots = self._knots()

        for (duty_low, fraction_low), (duty_high, fraction_high) in zip(knots, knots[1:]):
            if speed_fraction <= fraction_high:
                ratio = (speed_fraction - fraction_low) / (fraction_high - fraction_low)
                return min(1.0, duty_low + ratio * (duty_high - duty_low))
        return 1.0
