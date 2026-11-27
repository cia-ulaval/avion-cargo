from __future__ import annotations

import math
from collections import deque
from typing import Mapping

import numpy as np

from payload.domain.positioning.events import DistanceEvent, DistanceFailure, DistanceSample
from payload.domain.positioning.position_estimator import PositionEstimator
from shared.contracts.geometry import Position, PositionEstimate


class NonlinearLeastSquaresPositionEstimator(PositionEstimator):
    """Estimate tag X/Y on a fixed horizontal plane from full 3-D UWB ranges.

    The ground is world Z=0 and ``tag_height_m`` is the known tag height above
    that ground. Anchor X/Y coordinates define a stable horizontal reference
    frame; their individual Z coordinates are retained in the range model.

    Measurements arrive independently. The estimator keeps the most recent
    filtered range from each anchor and re-solves immediately whenever one new
    valid range arrives. It does not wait for or construct synchronized
    multi-anchor measurement batches.
    """

    def __init__(
        self,
        anchors: Mapping[str, Position],
        *,
        tag_height_m: float,
        max_residual_m: float = 0.25,
        range_std_m: float = 0.015,
        max_range_age_us: int = 250_000,
        range_filter_window: int = 3,
        outlier_gate_sigma: float = 5.0,
        max_iterations: int = 30,
        convergence_m: float = 1e-7,
    ) -> None:
        if len(anchors) < 3:
            raise ValueError("at least three anchors are required")
        if not math.isfinite(tag_height_m):
            raise ValueError("tag_height_m must be finite")
        if not math.isfinite(max_residual_m) or max_residual_m <= 0:
            raise ValueError("max_residual_m must be finite and positive")
        if not math.isfinite(range_std_m) or range_std_m <= 0:
            raise ValueError("range_std_m must be finite and positive")
        if (
            isinstance(max_range_age_us, bool)
            or not isinstance(max_range_age_us, int)
            or max_range_age_us <= 0
        ):
            raise ValueError("max_range_age_us must be a positive integer")
        if (
            isinstance(range_filter_window, bool)
            or not isinstance(range_filter_window, int)
            or range_filter_window < 1
        ):
            raise ValueError("range_filter_window must be a positive integer")
        if not math.isfinite(outlier_gate_sigma) or outlier_gate_sigma <= 0:
            raise ValueError("outlier_gate_sigma must be finite and positive")
        if max_iterations < 1:
            raise ValueError("max_iterations must be positive")

        self._anchors = dict(anchors)
        if any(not isinstance(key, str) or not key for key in self._anchors):
            raise ValueError("anchor IDs must be non-empty strings")
        if len(set(self._anchors)) != len(self._anchors):
            raise ValueError("anchor IDs must be unique")
        if any(not isinstance(position, Position) for position in self._anchors.values()):
            raise TypeError("anchor positions must be Position instances")

        anchor_world = np.asarray(
            [[position.x, position.y, position.z] for position in self._anchors.values()],
            dtype=float,
        )
        if not np.all(np.isfinite(anchor_world)):
            raise ValueError("anchor coordinates must be finite")

        # The reference frame is horizontal (world XY), not the slightly tilted
        # plane formed by anchors at different mounting heights. X follows the
        # first-to-second anchor baseline; Y is oriented toward the third anchor.
        anchor_xy_world = anchor_world[:, :2]
        axis_x_xy = anchor_xy_world[1] - anchor_xy_world[0]
        axis_x_norm = float(np.linalg.norm(axis_x_xy))
        if axis_x_norm < 1e-8:
            raise ValueError("the first two anchors must have distinct XY positions")
        axis_x_xy = axis_x_xy / axis_x_norm
        axis_y_xy = np.array([-axis_x_xy[1], axis_x_xy[0]], dtype=float)
        third_side = float(np.dot(anchor_xy_world[2] - anchor_xy_world[0], axis_y_xy))
        if abs(third_side) < 1e-8:
            raise ValueError("the first three anchors must not be collinear in XY")
        if third_side < 0:
            axis_y_xy *= -1.0

        origin_xy = np.mean(anchor_xy_world, axis=0)
        self._origin = np.array([origin_xy[0], origin_xy[1], float(tag_height_m)])
        self._axis_x = np.array([axis_x_xy[0], axis_x_xy[1], 0.0])
        self._axis_y = np.array([axis_y_xy[0], axis_y_xy[1], 0.0])
        self._anchor_xy = np.column_stack(
            (
                (anchor_xy_world - origin_xy) @ axis_x_xy,
                (anchor_xy_world - origin_xy) @ axis_y_xy,
            )
        )
        self._anchor_world = anchor_world
        self._anchor_ids = tuple(self._anchors)
        self._anchor_index_by_id = {
            anchor_id: index for index, anchor_id in enumerate(self._anchor_ids)
        }
        self._anchor_vertical = float(tag_height_m) - anchor_world[:, 2]

        self._tag_height_m = float(tag_height_m)
        self._max_residual_m = float(max_residual_m)
        self._range_std_m = float(range_std_m)
        self._max_range_age_us = int(max_range_age_us)
        self._range_filter_window = int(range_filter_window)
        self._outlier_gate_sigma = float(outlier_gate_sigma)
        self._max_iterations = int(max_iterations)
        self._convergence_m = float(convergence_m)

        # Each anchor owns an independent filter/history. A new measurement
        # changes only that anchor's range before triggering a fresh solve.
        self._range_history: dict[str, deque[float]] = {
            anchor_id: deque(maxlen=self._range_filter_window)
            for anchor_id in self._anchor_ids
        }
        self._ranges: dict[str, tuple[float, int, int | None, int | None, float | None]] = {}
        self._estimate: PositionEstimate | None = None
        self._sequence = 0
        self.rejected_outliers = 0

    @property
    def latest_estimate(self) -> PositionEstimate | None:
        return self._estimate

    @property
    def range_count(self) -> int:
        return len(self._ranges)

    @property
    def anchor_ids(self) -> tuple[str, ...]:
        return self._anchor_ids

    @property
    def reference_origin_world_m(self) -> tuple[float, float, float]:
        """World position at local reference coordinate (0, 0)."""
        return tuple(float(value) for value in self._origin)

    @property
    def reference_x_axis_world(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self._axis_x)

    @property
    def reference_y_axis_world(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self._axis_y)

    @property
    def anchors_xy_m(self) -> tuple[tuple[str, float, float], ...]:
        """Anchor XY positions in the horizontal local reference frame."""
        return tuple(
            (anchor_id, float(x), float(y))
            for anchor_id, (x, y) in zip(self._anchor_ids, self._anchor_xy)
        )

    def world_to_reference_xy(self, position: Position) -> tuple[float, float]:
        relative_xy = np.array([position.x, position.y], dtype=float) - self._origin[:2]
        return (
            float(relative_xy @ self._axis_x[:2]),
            float(relative_xy @ self._axis_y[:2]),
        )

    def _tag_world_position(self, reference_xy: np.ndarray) -> np.ndarray:
        return (
            self._origin
            + reference_xy[0] * self._axis_x
            + reference_xy[1] * self._axis_y
        )

    def update(self, event: DistanceEvent) -> PositionEstimate | None:
        if isinstance(event, DistanceFailure):
            self._expire_old_ranges(event)
            if len(self._ranges) < 3:
                self._estimate = None
                return None
            return self._estimate
        if not isinstance(event, DistanceSample) or event.anchor_id not in self._anchors:
            return self._estimate
        if not math.isfinite(event.distance_m) or event.distance_m <= 0:
            return self._estimate

        previous = self._ranges.get(event.anchor_id)
        if previous is not None and event.sequence <= previous[1]:
            return self._estimate

        self._expire_old_ranges(event)
        history = self._range_history[event.anchor_id]
        if self._is_range_outlier(float(event.distance_m), history):
            self.rejected_outliers += 1
            return self._estimate

        history.append(float(event.distance_m))
        filtered_distance = float(np.median(tuple(history)))
        self._ranges[event.anchor_id] = (
            filtered_distance,
            int(event.sequence),
            event.device_monotonic_us,
            event.epoch_us,
            event.received_at,
        )

        if len(self._ranges) < 3:
            self._estimate = None
            return None

        solution = self._solve()
        if solution is None:
            self._estimate = None
            return None
        x, y, residual, covariance = solution
        if residual > self._max_residual_m:
            # Keep the previous valid estimate rather than publishing a jump
            # produced by an inconsistent set of the latest independent ranges.
            return self._estimate

        self._sequence += 1
        self._estimate = PositionEstimate(
            position=Position(x, y, self._tag_height_m),
            residual_m=residual,
            anchor_ids=tuple(sorted(self._ranges)),
            sequence=self._sequence,
            device_monotonic_us=event.device_monotonic_us,
            epoch_us=event.epoch_us,
            received_at=event.received_at,
            covariance_xy_m2=covariance,
            quality="good" if residual < 0.04 else "degraded",
        )
        return self._estimate

    def _expire_old_ranges(self, event: DistanceSample | DistanceFailure) -> None:
        """Expire each independently stale anchor sample; do not form sweeps."""
        stale: list[str] = []
        for anchor_id, record in self._ranges.items():
            _distance, _sequence, device_us, _epoch_us, received_at = record
            age_us: float | None = None
            if event.device_monotonic_us is not None and device_us is not None:
                age_us = event.device_monotonic_us - device_us
            elif event.received_at is not None and received_at is not None:
                age_us = (event.received_at - received_at) * 1_000_000.0
            if age_us is not None and (age_us < -1_000 or age_us > self._max_range_age_us):
                stale.append(anchor_id)
        for anchor_id in stale:
            self._ranges.pop(anchor_id, None)
            self._range_history[anchor_id].clear()
        if stale:
            self._estimate = None

    def _is_range_outlier(self, distance_m: float, history: deque[float]) -> bool:
        if len(history) < 3:
            return False
        values = np.asarray(tuple(history), dtype=float)
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        robust_sigma = 1.4826 * mad
        gate = max(
            self._outlier_gate_sigma * self._range_std_m,
            self._outlier_gate_sigma * robust_sigma,
        )
        return abs(distance_m - median) > gate

    def _solve(self) -> tuple[float, float, float, tuple[float, float, float]] | None:
        ids = sorted(self._ranges)
        anchor_indices = [self._anchor_index_by_id[anchor_id] for anchor_id in ids]
        anchor_xy = self._anchor_xy[anchor_indices]
        vertical = self._anchor_vertical[anchor_indices]
        measured = np.asarray([self._ranges[anchor_id][0] for anchor_id in ids], dtype=float)

        horizontal_squared = measured**2 - vertical**2
        tolerance = (5.0 * self._range_std_m) ** 2
        if np.any(horizontal_squared < -tolerance):
            return None
        horizontal_squared = np.maximum(horizontal_squared, 0.0)

        # Closed-form initialization from differences of sphere equations.
        # This provides a good starting point even when the payload is outside
        # the anchor triangle, where centroid-only Gauss-Newton starts can fail.
        base = anchor_xy[0]
        base_constant = float(base @ base - horizontal_squared[0])
        matrix = 2.0 * (anchor_xy[1:] - base)
        vector = (
            np.sum(anchor_xy[1:] ** 2, axis=1)
            - horizontal_squared[1:]
            - base_constant
        )
        try:
            linear_start, *_ = np.linalg.lstsq(matrix, vector, rcond=None)
        except np.linalg.LinAlgError:
            return None
        if np.linalg.matrix_rank(matrix) < 2 or not np.all(np.isfinite(linear_start)):
            return None

        measured_starts: list[np.ndarray] = [linear_start]
        if self._estimate is not None:
            measured_starts.append(np.array([self._estimate.position.x, self._estimate.position.y]))
        measured_starts.append(np.mean(anchor_xy, axis=0))

        def evaluate(point: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            delta = point[None, :] - anchor_xy
            predicted = np.sqrt(np.sum(delta**2, axis=1) + vertical**2)
            predicted = np.maximum(predicted, 1e-12)
            residuals = predicted - measured
            jacobian = delta / predicted[:, None]
            return predicted, residuals, jacobian

        candidates: list[tuple[float, np.ndarray]] = []
        for initial in measured_starts:
            point = np.asarray(initial, dtype=float).copy()
            for _ in range(self._max_iterations):
                _predicted, residuals, jacobian = evaluate(point)
                normalized = np.abs(residuals) / self._range_std_m
                weights = np.ones_like(normalized)
                large = normalized > 2.5
                weights[large] = 2.5 / normalized[large]
                sqrt_weights = np.sqrt(weights)
                try:
                    step, *_ = np.linalg.lstsq(
                        jacobian * sqrt_weights[:, None],
                        residuals * sqrt_weights,
                        rcond=None,
                    )
                except np.linalg.LinAlgError:
                    break
                if not np.all(np.isfinite(step)):
                    break

                current_cost = self._huber_cost(residuals, self._range_std_m)
                accepted = False
                scale = 1.0
                for _line_search in range(8):
                    candidate = point - scale * step
                    _p, candidate_residuals, _j = evaluate(candidate)
                    if self._huber_cost(candidate_residuals, self._range_std_m) <= current_cost:
                        point = candidate
                        accepted = True
                        break
                    scale *= 0.5
                if not accepted or float(np.linalg.norm(scale * step)) < self._convergence_m:
                    break

            _predicted, residuals, _jacobian = evaluate(point)
            cost = self._huber_cost(residuals, self._range_std_m)
            if math.isfinite(cost) and np.all(np.isfinite(point)):
                candidates.append((cost, point))

        if not candidates:
            return None
        _cost, point = min(candidates, key=lambda candidate: candidate[0])
        _predicted, residuals, jacobian = evaluate(point)
        rms = float(np.sqrt(np.mean(residuals**2)))
        covariance_matrix = np.linalg.pinv(jacobian.T @ jacobian) * self._range_std_m**2
        covariance = (
            float(covariance_matrix[0, 0]),
            float(covariance_matrix[0, 1]),
            float(covariance_matrix[1, 1]),
        )
        return float(point[0]), float(point[1]), rms, covariance

    @staticmethod
    def _huber_cost(residuals: np.ndarray, scale: float) -> float:
        normalized = np.abs(residuals) / scale
        threshold = 2.5
        quadratic = normalized <= threshold
        cost = np.empty_like(normalized)
        cost[quadratic] = 0.5 * normalized[quadratic] ** 2
        cost[~quadratic] = threshold * (normalized[~quadratic] - 0.5 * threshold)
        return float(np.sum(cost))
