from payload.domain.docking.plane_peer import PlanePeer
from payload.domain.positioning.distance_sensor import DistanceSensor
from payload.domain.positioning.events import DistanceFailure, DistanceSample
from payload.domain.positioning.position_estimator import PositionEstimator
from shared.contracts.geometry import Position
from shared.contracts.messages import FailureReason


def test_distance_event_models_include_samples_and_failures() -> None:
    sample = DistanceSample(anchor_id="anchor-a", distance_m=1.25, sequence=3)
    failure = DistanceFailure(anchor_id="anchor-b", reason=FailureReason("timeout", "No response"))

    assert sample.distance_m == 1.25
    assert failure.reason.code == "timeout"


def test_positioning_and_plane_peer_protocols_are_importable() -> None:
    assert DistanceSensor.__name__ == "DistanceSensor"
    assert PositionEstimator.__name__ == "PositionEstimator"
    assert PlanePeer.__name__ == "PlanePeer"
    assert Position(0.0, 0.0, 0.0).z == 0.0
