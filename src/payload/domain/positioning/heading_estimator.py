from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from payload.domain.body_velocity import BodyVelocity
from shared.contracts.geometry import Position


@dataclass(frozen=True, slots=True)
class HeadingEstimate:
    angle_rad: float
    confidence: float
    observations: int


class TranslationHeadingEstimator:
    """Infer payload yaw from body-frame commands and observed tag displacement.

    The caller supplies every fresh position estimate as it is produced by an
    individual range event. No synchronized anchor sweep is required. Heading
    candidates are accepted only after stable translation over a sufficient
    time/displacement window, then circularly filtered to reject position noise.
    """

    def __init__(
        self,
        *,
        minimum_displacement_m: float = 0.08,
        maximum_yaw_rate_rad_s: float = 0.45,
        smoothing: float = 0.25,
        minimum_interval_s: float = 0.20,
        maximum_interval_s: float = 1.0,
        consistency_threshold: float = 0.70,
        candidate_window: int = 5,
    ) -> None:
        if (
            minimum_displacement_m <= 0
            or maximum_yaw_rate_rad_s <= 0
            or not 0 < smoothing <= 1
            or minimum_interval_s <= 0
            or maximum_interval_s <= minimum_interval_s
            or not 0 < consistency_threshold <= 1
            or candidate_window < 3
        ):
            raise ValueError("invalid heading-estimator limits")
        self.minimum_displacement_m = minimum_displacement_m
        self.maximum_yaw_rate_rad_s = maximum_yaw_rate_rad_s
        self.smoothing = smoothing
        self.minimum_interval_s = minimum_interval_s
        self.maximum_interval_s = maximum_interval_s
        self.consistency_threshold = consistency_threshold
        self._estimate = HeadingEstimate(0.0, 0.0, 0)
        self._previous_position: Position | None = None
        self._previous_time_us: int | None = None
        self._previous_velocity: BodyVelocity | None = None
        self._candidate_history: deque[float] = deque(maxlen=candidate_window)

    @property
    def estimate(self) -> HeadingEstimate:
        return self._estimate

    def reset_baseline(self, *, clear_candidates: bool = True) -> None:
        """Forget motion history; preserve the last heading estimate."""
        self._previous_position = None
        self._previous_time_us = None
        self._previous_velocity = None
        if clear_candidates:
            self._candidate_history.clear()

    def observe(
        self,
        position: Position,
        body_velocity: BodyVelocity,
        device_monotonic_us: int | None,
    ) -> HeadingEstimate:
        if device_monotonic_us is None:
            self.reset_baseline()
            return self._estimate

        previous = self._previous_position
        previous_time = self._previous_time_us
        if previous is None or previous_time is None:
            self._set_baseline(position, body_velocity, device_monotonic_us)
            return self._estimate

        if device_monotonic_us <= previous_time:
            # Multiple independent anchor measurements can share the same
            # initiator timestamp. Refresh the baseline without discarding the
            # candidate history accumulated from earlier time intervals.
            self._set_baseline(position, body_velocity, device_monotonic_us)
            return self._estimate

        dt = (device_monotonic_us - previous_time) / 1_000_000.0
        if dt > self.maximum_interval_s:
            self.reset_baseline()
            self._set_baseline(position, body_velocity, device_monotonic_us)
            return self._estimate

        previous_velocity = self._previous_velocity
        body_speed = math.hypot(body_velocity.vx_m_s, body_velocity.vy_m_s)
        previous_speed = (
            math.hypot(previous_velocity.vx_m_s, previous_velocity.vy_m_s) if previous_velocity is not None else 0.0
        )
        if (
            previous_velocity is None
            or body_speed < 0.02
            or previous_speed < 0.02
            or abs(body_velocity.yaw_rate_rad_s) > self.maximum_yaw_rate_rad_s
            or abs(previous_velocity.yaw_rate_rad_s) > self.maximum_yaw_rate_rad_s
        ):
            self.reset_baseline()
            self._set_baseline(position, body_velocity, device_monotonic_us)
            return self._estimate

        speed_change = abs(body_speed - previous_speed)
        direction_change = abs(
            _wrap(
                math.atan2(body_velocity.vy_m_s, body_velocity.vx_m_s)
                - math.atan2(previous_velocity.vy_m_s, previous_velocity.vx_m_s)
            )
        )
        if speed_change > max(0.03, 0.25 * previous_speed) or direction_change > 0.20:
            # A materially changed setpoint cannot be paired with the previous
            # displacement interval. Start a new translation observation.
            self.reset_baseline()
            self._set_baseline(position, body_velocity, device_monotonic_us)
            return self._estimate

        if dt < self.minimum_interval_s:
            return self._estimate

        dx = position.x - previous.x
        dy = position.y - previous.y
        displacement = math.hypot(dx, dy)
        if displacement < self.minimum_displacement_m:
            # Keep the old baseline so valid small movements accumulate.
            return self._estimate

        observed_speed = displacement / dt
        if not 0.4 * body_speed <= observed_speed <= 2.5 * body_speed:
            self.reset_baseline()
            self._set_baseline(position, body_velocity, device_monotonic_us)
            return self._estimate

        self._set_baseline(position, body_velocity, device_monotonic_us)
        observed_angle = math.atan2(dy, dx)
        commanded_body_angle = math.atan2(body_velocity.vy_m_s, body_velocity.vx_m_s)
        # The body-frame command is already the command actually sent to the
        # wheel controller. Rotating it into the observed reference-frame
        # displacement yields the absolute payload heading.
        candidate = _wrap(observed_angle - commanded_body_angle)
        self._candidate_history.append(candidate)

        mean_angle, consistency = _circular_mean(self._candidate_history)
        if consistency < self.consistency_threshold:
            return self._estimate

        if self._estimate.observations == 0:
            # Do not publish a heading based on one noisy displacement. Wait
            # for at least three mutually consistent translation observations.
            if len(self._candidate_history) < 3:
                return self._estimate
            observations = len(self._candidate_history)
            confidence = min(1.0, observations / 5.0) * consistency
            self._estimate = HeadingEstimate(mean_angle, confidence, observations)
            return self._estimate

        delta = _wrap(mean_angle - self._estimate.angle_rad)
        # Limit a single noisy observation's influence without preventing a
        # later translation from correcting heading drift after in-place yaw.
        delta = max(-0.35, min(0.35, delta))
        angle = _wrap(self._estimate.angle_rad + self.smoothing * delta)
        observations = self._estimate.observations + 1
        confidence = min(1.0, observations / 5.0) * consistency
        self._estimate = HeadingEstimate(angle, confidence, observations)
        return self._estimate

    def _set_baseline(
        self,
        position: Position,
        velocity: BodyVelocity,
        device_monotonic_us: int,
    ) -> None:
        self._previous_position = position
        self._previous_time_us = device_monotonic_us
        self._previous_velocity = velocity


def _circular_mean(angles: deque[float]) -> tuple[float, float]:
    if not angles:
        return 0.0, 0.0
    mean_sin = sum(math.sin(angle) for angle in angles) / len(angles)
    mean_cos = sum(math.cos(angle) for angle in angles) / len(angles)
    return math.atan2(mean_sin, mean_cos), min(1.0, math.hypot(mean_sin, mean_cos))


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
