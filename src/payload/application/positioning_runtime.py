from __future__ import annotations

import math
from dataclasses import dataclass
from typing import AsyncIterator

from payload.domain.body_velocity import BodyVelocity
from payload.domain.positioning.events import DistanceEvent, DistanceFailure
from payload.domain.positioning.heading_estimator import HeadingEstimate, TranslationHeadingEstimator
from payload.domain.positioning.position_estimator import PositionEstimator
from shared.contracts.geometry import Position, PositionEstimate


@dataclass(frozen=True, slots=True)
class PayloadPositioningSnapshot:
    tag_estimate: PositionEstimate | None
    payload_position: Position | None
    heading: HeadingEstimate
    status: str
    last_event_sequence: int | None
    anchors_xy_m: tuple[tuple[str, float, float], ...]
    payload_to_tag_xy_m: tuple[float, float]
    reference_origin_world_m: tuple[float, float, float]
    reference_x_axis_world: tuple[float, float, float]
    reference_y_axis_world: tuple[float, float, float]
    range_count: int
    rejected_range_count: int


class PayloadPositioningRuntime:
    """Consume independent range events and maintain position/heading state."""

    def __init__(
        self,
        estimator: PositionEstimator,
        heading_estimator: TranslationHeadingEstimator,
        *,
        payload_to_tag_xy_m: tuple[float, float] = (0.0, 0.0),
        anchors_xy_m: tuple[tuple[str, float, float], ...] = (),
        reference_origin_world_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
        reference_x_axis_world: tuple[float, float, float] = (1.0, 0.0, 0.0),
        reference_y_axis_world: tuple[float, float, float] = (0.0, 1.0, 0.0),
        maximum_heading_position_sigma_m: float = 0.25,
    ) -> None:
        self._estimator = estimator
        self._heading = heading_estimator
        self._offset = payload_to_tag_xy_m
        if len(payload_to_tag_xy_m) != 2 or not all(math.isfinite(v) for v in payload_to_tag_xy_m):
            raise ValueError("payload_to_tag_xy_m must contain two finite values")
        self._anchors_xy_m = tuple(anchors_xy_m)
        self._reference_origin_world_m = _finite_vector(reference_origin_world_m, "reference_origin_world_m")
        self._reference_x_axis_world = _finite_vector(reference_x_axis_world, "reference_x_axis_world")
        self._reference_y_axis_world = _finite_vector(reference_y_axis_world, "reference_y_axis_world")
        if not math.isfinite(maximum_heading_position_sigma_m) or maximum_heading_position_sigma_m <= 0:
            raise ValueError("maximum_heading_position_sigma_m must be finite and positive")
        self._maximum_heading_position_sigma_m = maximum_heading_position_sigma_m
        self._latest: PositionEstimate | None = None
        self._last_sequence: int | None = None
        self._last_body_velocity = BodyVelocity(0.0, 0.0, 0.0)
        self._status = "waiting_for_three_anchors"

    @property
    def snapshot(self) -> PayloadPositioningSnapshot:
        payload_position = None
        if self._latest is not None:
            c = math.cos(self._heading.estimate.angle_rad)
            s = math.sin(self._heading.estimate.angle_rad)
            tx, ty = self._offset
            payload_position = Position(
                self._latest.position.x - (c * tx - s * ty),
                self._latest.position.y - (s * tx + c * ty),
                self._latest.position.z,
            )
        return PayloadPositioningSnapshot(
            self._latest,
            payload_position,
            self._heading.estimate,
            self._status,
            self._last_sequence,
            self._anchors_xy_m,
            self._offset,
            self._reference_origin_world_m,
            self._reference_x_axis_world,
            self._reference_y_axis_world,
            int(getattr(self._estimator, "range_count", 0)),
            int(getattr(self._estimator, "rejected_outliers", 0)),
        )

    def set_body_velocity(self, velocity: BodyVelocity) -> None:
        self._last_body_velocity = velocity
        if (
            math.hypot(velocity.vx_m_s, velocity.vy_m_s) < 0.02
            or abs(velocity.yaw_rate_rad_s) > self._heading.maximum_yaw_rate_rad_s
        ):
            # Stops and in-place rotations cannot provide translation-derived
            # heading observations. Preserve heading but discard motion history.
            self._heading.reset_baseline()

    def process(self, event: DistanceEvent) -> PayloadPositioningSnapshot:
        estimate = self._estimator.update(event)
        if estimate is None:
            estimator_latest = getattr(self._estimator, "latest_estimate", None)
            if isinstance(event, DistanceFailure) or estimator_latest is None:
                self._latest = None
                self._last_sequence = None
                self._status = "waiting_for_fresh_ranges"
                self._heading.reset_baseline()
            return self.snapshot

        estimate_is_new = self._latest is None or estimate.sequence > self._latest.sequence
        if estimate_is_new:
            self._latest = estimate
            self._last_sequence = estimate.sequence
            self._status = "tracking"
            if (
                estimate.residual_m is not None
                and estimate.residual_m <= 0.08
                and self._position_sigma(estimate) <= self._maximum_heading_position_sigma_m
            ):
                # This runs once per new position estimate. The estimator emits
                # one after each accepted individual range event; no three-range
                # sweep or timestamp correlation is performed here.
                self._heading.observe(
                    estimate.position,
                    self._last_body_velocity,
                    estimate.device_monotonic_us,
                )
        return self.snapshot

    async def consume(self, events: AsyncIterator[DistanceEvent]) -> None:
        async for event in events:
            self.process(event)

    @staticmethod
    def _position_sigma(estimate: PositionEstimate) -> float:
        covariance = estimate.covariance_xy_m2
        if covariance is None:
            return math.inf
        xx, xy, yy = covariance
        largest = 0.5 * (xx + yy + math.sqrt(max(0.0, (xx - yy) ** 2 + 4.0 * xy**2)))
        return math.sqrt(max(0.0, largest))


def _finite_vector(value: tuple[float, float, float], name: str) -> tuple[float, float, float]:
    if len(value) != 3 or not all(math.isfinite(component) for component in value):
        raise ValueError(f"{name} must contain three finite values")
    return tuple(float(component) for component in value)
