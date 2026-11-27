import math
import random
import statistics

import pytest

from payload.application.positioning_runtime import PayloadPositioningRuntime
from payload.domain.body_velocity import BodyVelocity
from payload.domain.positioning.events import DistanceFailure, DistanceSample
from payload.domain.positioning.heading_estimator import TranslationHeadingEstimator
from payload.domain.positioning.nonlinear_least_squares import NonlinearLeastSquaresPositionEstimator
from shared.contracts.geometry import Position, PositionEstimate
from shared.contracts.messages import FailureReason


ANCHORS = {
    "anchor-3": Position(-1.0, -0.8, 0.05),
    "anchor-4": Position(1.0, -0.8, 0.05),
    "anchor-5": Position(0.0, 1.0, 0.08),
}


def _ranges_for(position: Position) -> dict[str, float]:
    return {
        anchor_id: math.dist(
            (position.x, position.y, position.z),
            (anchor.x, anchor.y, anchor.z),
        )
        for anchor_id, anchor in ANCHORS.items()
    }


def _feed_initial(estimator, truth: Position, sequence: int = 1):
    result = None
    distances = _ranges_for(truth)
    for offset, anchor_id in enumerate(ANCHORS):
        result = estimator.update(
            DistanceSample(
                anchor_id,
                distances[anchor_id],
                sequence + offset,
                device_monotonic_us=1_000_000,
            )
        )
    return result


def test_constrained_sphere_plane_solver_uses_world_ground_height_and_3d_ranges():
    truth = Position(0.0, 0.0, 0.081)
    estimator = NonlinearLeastSquaresPositionEstimator(ANCHORS, tag_height_m=0.081)
    distances = _ranges_for(truth)

    for sequence, anchor_id in enumerate(ANCHORS, start=1):
        estimate = estimator.update(
            DistanceSample(
                anchor_id,
                distances[anchor_id],
                sequence,
                device_monotonic_us=2_152_000,
            )
        )
        if sequence < 3:
            assert estimate is None

    assert estimate is not None
    expected_xy = estimator.world_to_reference_xy(truth)
    assert estimate.position.x == pytest.approx(expected_xy[0], abs=1e-6)
    assert estimate.position.y == pytest.approx(expected_xy[1], abs=1e-6)
    assert estimate.position.z == pytest.approx(0.081)
    assert estimate.residual_m == pytest.approx(0.0, abs=1e-7)

    # The tag-motion plane is horizontal at world Z=0.081. The configured
    # anchor mounting heights remain independent Z values in the 3-D range model.
    assert estimator.reference_origin_world_m == pytest.approx((0.0, -0.2, 0.081))
    assert estimator.reference_x_axis_world == pytest.approx((1.0, 0.0, 0.0))
    assert estimator.reference_y_axis_world == pytest.approx((0.0, 1.0, 0.0))
    anchors_xy = estimator.anchors_xy_m
    assert tuple(row[0] for row in anchors_xy) == ("anchor-3", "anchor-4", "anchor-5")
    expected_anchors_xy = ((-1.0, -0.6), (1.0, -0.6), (0.0, 1.2))
    for actual, expected in zip(anchors_xy, expected_anchors_xy):
        assert actual[1:] == pytest.approx(expected)


def test_solver_converges_outside_anchor_triangle():
    truth = Position(2.0, 0.6, 0.081)
    estimator = NonlinearLeastSquaresPositionEstimator(ANCHORS, tag_height_m=truth.z)
    estimate = _feed_initial(estimator, truth)
    assert estimate is not None
    expected = estimator.world_to_reference_xy(truth)
    assert estimate.position.x == pytest.approx(expected[0], abs=1e-5)
    assert estimate.position.y == pytest.approx(expected[1], abs=1e-5)
    assert estimate.residual_m == pytest.approx(0.0, abs=1e-7)


def test_every_individual_range_event_updates_position_without_a_triplet_batch():
    estimator = NonlinearLeastSquaresPositionEstimator(ANCHORS, tag_height_m=0.081)
    initial = _feed_initial(estimator, Position(0.0, 0.0, 0.081))
    assert initial is not None

    # Repeated independent updates from a single anchor each trigger a solve
    # against that anchor's newest range and the latest retained ranges from
    # the other anchors. No synchronized three-measurement batch is required.
    result = initial
    for sequence, x in enumerate((0.04, 0.08, 0.12), start=10):
        truth = Position(x, 0.0, 0.081)
        result = estimator.update(
            DistanceSample(
                "anchor-3",
                _ranges_for(truth)["anchor-3"],
                sequence,
                device_monotonic_us=1_000_000 + sequence * 20_000,
            )
        )
        assert result is not None
        assert result.sequence > initial.sequence
    assert result.sequence == initial.sequence + 3


def test_independent_per_anchor_median_filter_rejects_a_range_spike():
    truth = Position(0.2, 0.1, 0.081)
    estimator = NonlinearLeastSquaresPositionEstimator(ANCHORS, tag_height_m=truth.z)
    distances = _ranges_for(truth)
    sequence = 1
    estimate = None
    for _ in range(4):
        for anchor_id in ANCHORS:
            estimate = estimator.update(
                DistanceSample(
                    anchor_id,
                    distances[anchor_id],
                    sequence,
                    device_monotonic_us=1_000_000 + sequence * 20_000,
                )
            )
            sequence += 1
    assert estimate is not None
    stable_sequence = estimate.sequence

    rejected = estimator.update(
        DistanceSample(
            "anchor-3",
            distances["anchor-3"] + 0.5,
            sequence,
            device_monotonic_us=1_000_000 + sequence * 20_000,
        )
    )
    assert rejected is not None
    assert rejected.sequence == stable_sequence
    assert estimator.rejected_outliers == 1

    recovered = estimator.update(
        DistanceSample(
            "anchor-3",
            distances["anchor-3"],
            sequence + 1,
            device_monotonic_us=1_000_000 + (sequence + 1) * 20_000,
        )
    )
    assert recovered is not None
    expected = estimator.world_to_reference_xy(truth)
    assert recovered.position.x == pytest.approx(expected[0], abs=0.02)
    assert recovered.position.y == pytest.approx(expected[1], abs=0.02)



def test_independent_range_filter_reduces_noise_while_tracking_outside_triangle():
    truth = Position(1.4, 0.25, 0.081)  # outside the anchor triangle
    estimator = NonlinearLeastSquaresPositionEstimator(ANCHORS, tag_height_m=truth.z)
    noise = random.Random(23)
    distances = _ranges_for(truth)
    estimates = []
    sequence = 1
    for cycle in range(120):
        timestamp = 1_000_000 + cycle * 20_000
        for anchor_id in ANCHORS:
            measured = distances[anchor_id] + noise.gauss(0.0, 0.015)
            estimate = estimator.update(
                DistanceSample(
                    anchor_id,
                    measured,
                    sequence,
                    device_monotonic_us=timestamp,
                )
            )
            sequence += 1
            if estimate is not None:
                estimates.append(estimate)

    assert len(estimates) > 300
    expected_xy = estimator.world_to_reference_xy(truth)
    final = estimates[-1]
    assert final.position.x == pytest.approx(expected_xy[0], abs=0.025)
    assert final.position.y == pytest.approx(expected_xy[1], abs=0.025)
    # The independent per-anchor median filter should keep stationary XY
    # jitter below the raw 15 mm range-noise standard deviation.
    x_jitter = statistics.pstdev(item.position.x for item in estimates[-180:])
    y_jitter = statistics.pstdev(item.position.y for item in estimates[-180:])
    assert x_jitter < 0.015
    assert y_jitter < 0.015

def test_old_cached_anchor_range_is_expired_without_waiting_for_a_sweep():
    estimator = NonlinearLeastSquaresPositionEstimator(
        ANCHORS, tag_height_m=0.081, max_range_age_us=100_000
    )
    initial = _feed_initial(estimator, Position(0.0, 0.0, 0.081))
    assert initial is not None

    result = estimator.update(
        DistanceSample(
            "anchor-3",
            _ranges_for(Position(0.1, 0.0, 0.081))["anchor-3"],
            20,
            device_monotonic_us=1_200_001,
        )
    )
    assert result is None
    assert estimator.latest_estimate is None
    assert estimator.range_count == 1



def test_single_range_failure_keeps_last_good_ranges_until_they_expire():
    estimator = NonlinearLeastSquaresPositionEstimator(
        ANCHORS, tag_height_m=0.081, max_range_age_us=100_000
    )
    initial = _feed_initial(estimator, Position(0.0, 0.0, 0.081))
    assert initial is not None

    # One failed attempt must not invalidate the other fresh range cache or
    # force three anchors to initialize again.
    retained = estimator.update(
        DistanceFailure(
            "anchor-3",
            FailureReason(code="range_timeout", message="one missed range"),
            device_monotonic_us=1_020_000,
        )
    )
    assert retained is not None
    assert retained.sequence == initial.sequence
    assert estimator.range_count == 3

    # Failure events also advance the device clock. If ranging stops entirely,
    # the independent freshness deadlines expire without waiting for a sample.
    expired = estimator.update(
        DistanceFailure(
            "anchor-4",
            FailureReason(code="range_timeout", message="ranging outage"),
            device_monotonic_us=1_120_001,
        )
    )
    assert expired is None
    assert estimator.latest_estimate is None
    assert estimator.range_count == 0

def test_heading_estimator_requires_consistent_translation_and_ignores_pure_yaw():
    estimator = TranslationHeadingEstimator(
        minimum_displacement_m=0.01,
        minimum_interval_s=0.12,
    )
    command = BodyVelocity(0.1, 0.0, 0.0)
    true_heading = 1.1
    direction = (math.cos(true_heading), math.sin(true_heading))
    estimate = estimator.observe(Position(0.0, 0.0, 0.081), command, 1_000_000)
    assert estimate.observations == 0

    for index in range(1, 4):
        estimate = estimator.observe(
            Position(0.025 * index * direction[0], 0.025 * index * direction[1], 0.081),
            command,
            1_000_000 + index * 250_000,
        )
    assert estimate.observations >= 3
    assert estimate.angle_rad == pytest.approx(true_heading, abs=1e-6)

    before = estimator.estimate
    estimator.observe(
        Position(0.075 * direction[0], 0.075 * direction[1], 0.081),
        BodyVelocity(0.0, 0.0, 1.0),
        2_000_000,
    )
    estimator.observe(
        Position(0.075 * direction[0], 0.075 * direction[1], 0.081),
        BodyVelocity(0.0, 0.0, 1.0),
        2_250_000,
    )
    assert estimator.estimate == before


class _EventfulMovingPositionEstimator:
    def __init__(self) -> None:
        self.sequence = 0
        self.position = Position(0.0, 0.0, 0.081)
        self.latest_estimate = None

    def update(self, event):
        if not isinstance(event, DistanceSample):
            self.latest_estimate = None
            return None
        self.sequence += 1
        direction = math.pi / 4
        self.position = Position(
            self.position.x + 0.025 * math.cos(direction),
            self.position.y + 0.025 * math.sin(direction),
            0.081,
        )
        self.latest_estimate = PositionEstimate(
            position=self.position,
            residual_m=0.01,
            anchor_ids=("anchor-3",),
            sequence=self.sequence,
            device_monotonic_us=event.device_monotonic_us,
            covariance_xy_m2=(0.001, 0.0, 0.001),
            quality="good",
        )
        return self.latest_estimate


def test_runtime_updates_heading_from_individual_range_events_not_anchor_sweeps():
    estimator = _EventfulMovingPositionEstimator()
    heading = TranslationHeadingEstimator(
        minimum_displacement_m=0.01,
        minimum_interval_s=0.12,
    )
    runtime = PayloadPositioningRuntime(
        estimator,
        heading,
        payload_to_tag_xy_m=(0.012, -0.006),
    )
    runtime.set_body_velocity(BodyVelocity(0.1, 0.0, 0.0))

    # All observations arrive from the same anchor ID. Position updates and
    # heading inference remain event-driven; no three-anchor sweep is assembled.
    for sequence in range(1, 5):
        snapshot = runtime.process(
            DistanceSample(
                "anchor-3",
                1.0,
                sequence,
                device_monotonic_us=1_000_000 + sequence * 250_000,
            )
        )
    assert snapshot.tag_estimate is not None
    assert snapshot.heading.observations >= 3
    assert snapshot.heading.angle_rad == pytest.approx(math.pi / 4, abs=1e-6)

    previous_heading = snapshot.heading
    # No range events means no heading updates, regardless of commanded motion.
    runtime.set_body_velocity(BodyVelocity(0.0, 0.0, 1.0))
    assert runtime.snapshot.heading == previous_heading
    runtime.process(
        DistanceFailure(
            "anchor-3",
            FailureReason(code="range_failed", message="range failed"),
            device_monotonic_us=2_500_000,
        )
    )
    assert runtime.snapshot.heading == previous_heading
    assert runtime.snapshot.tag_estimate is None


def test_payload_position_runtime_applies_configured_tag_lever_arm():
    truth = Position(0.2, 0.1, 0.081)
    estimator = NonlinearLeastSquaresPositionEstimator(ANCHORS, tag_height_m=truth.z)
    expected_tag_xy = estimator.world_to_reference_xy(truth)
    runtime = PayloadPositioningRuntime(
        estimator,
        TranslationHeadingEstimator(),
        payload_to_tag_xy_m=(0.03, -0.01),
        anchors_xy_m=estimator.anchors_xy_m,
        reference_origin_world_m=estimator.reference_origin_world_m,
        reference_x_axis_world=estimator.reference_x_axis_world,
        reference_y_axis_world=estimator.reference_y_axis_world,
    )
    distances = _ranges_for(truth)
    snapshot = None
    for sequence, anchor_id in enumerate(ANCHORS, start=1):
        snapshot = runtime.process(
            DistanceSample(
                anchor_id,
                distances[anchor_id],
                sequence,
                device_monotonic_us=2_152_000,
            )
        )
    assert snapshot is not None and snapshot.tag_estimate is not None
    assert snapshot.payload_position is not None
    assert snapshot.payload_position.x == pytest.approx(expected_tag_xy[0] - 0.03, abs=1e-5)
    assert snapshot.payload_position.y == pytest.approx(expected_tag_xy[1] + 0.01, abs=1e-5)
