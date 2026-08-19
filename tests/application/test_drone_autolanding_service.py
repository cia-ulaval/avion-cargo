from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pytest

from application import drone_autolanding_service as autolanding_module
from domain.models import Pose3D, TargetedMarker


@dataclass
class FakeContentStreamer:
    _frame_buffer: Any
    _config: Any
    sent_payloads: list[dict[str, Any]] = field(default_factory=list)

    def send_data(self, data: dict[str, Any]) -> None:
        self.sent_payloads.append(data)

    def stream_video(self) -> None:
        return None


@dataclass
class FakeTrackerCamera:
    fps: int = 30

    def open(self) -> None:
        return None

    def get_fps(self) -> int:
        return self.fps


@dataclass
class FakeTracker:
    target: TargetedMarker
    camera: FakeTrackerCamera = field(default_factory=FakeTrackerCamera)

    def get_target(self) -> TargetedMarker:
        return self.target


@dataclass
class FakeDrone:
    status: Any
    service: autolanding_module.DroneAutolandingService | None = None
    land_mode_calls: int = 0
    land_calls: list[tuple[Pose3D, tuple[float, float]]] = field(default_factory=list)

    def activate_land_mode(self) -> None:
        self.land_mode_calls += 1

    def get_status(self) -> Any:
        assert self.service is not None
        self.service._tracking_started = False
        return self.status

    def land_on_target(self, position: Pose3D, target_size: tuple[float, float]) -> None:
        self.land_calls.append((position, target_size))


def build_service(monkeypatch: pytest.MonkeyPatch, drone: FakeDrone) -> autolanding_module.DroneAutolandingService:
    monkeypatch.setattr(autolanding_module, "WebRTCContentStreamer", FakeContentStreamer)
    service = autolanding_module.DroneAutolandingService(
        drone=drone,
        tracker=FakeTracker(TargetedMarker(id=29, length=0.896, dictionary=0)),
        content_streamer_config=object(),
    )
    drone.service = service
    return service


@pytest.mark.parametrize(
    ("reported_relative_altitude", "tracked_pose_z", "expected_z"),
    [
        (6.2, 1.4, 6.2),
        (4.5, 1.4, 1.4),
    ],
)
def test_precision_landing_sends_uav_pose_with_altitude_selected_from_drone_status_when_reliable(
    monkeypatch: pytest.MonkeyPatch,
    reported_relative_altitude: float,
    tracked_pose_z: float,
    expected_z: float,
    sample_drone_status,
) -> None:
    drone = FakeDrone(sample_drone_status(relative_altitude=reported_relative_altitude))
    service = build_service(monkeypatch, drone)
    service.pose_buffer.set_uav_pose_value(Pose3D(x=0.2, y=-0.3, z=tracked_pose_z))
    service._tracking_started = True

    service._landing_target_loop()

    assert drone.land_mode_calls == 1
    assert drone.land_calls == [(Pose3D(x=0.2, y=-0.3, z=expected_z), (0.896, 0.896))]


def test_telemetry_payload_merges_latest_tracking_metadata_with_current_drone_status(
    monkeypatch: pytest.MonkeyPatch,
    sample_drone_status,
) -> None:
    drone_status = sample_drone_status(relative_altitude=3.2)
    service = build_service(monkeypatch, FakeDrone(drone_status))
    service.frame_buffer.set_value(
        frame=np.zeros((2, 2, 3), dtype=np.uint8),
        metadata={
            "status": 1,
            "marker_id": 29,
            "poses": {
                "estimated_pose_from_camera": {"x": 0.1, "y": 0.2, "z": 1.4},
                "estimated_pose_to_uav": {"x": -0.2, "y": 0.1, "z": 1.4},
            },
        },
    )
    service.drone_status_buffer.set_value(drone_status)

    payload = service._build_telemetry_payload()

    assert payload["status"] == 1
    assert payload["marker_id"] == 29
    assert payload["poses"]["estimated_pose_to_uav"] == {"x": -0.2, "y": 0.1, "z": 1.4}
    assert payload["drone"]["relative_altitude"] == pytest.approx(3.2)
    assert payload["drone"]["mode"] == drone_status.mode
