from dataclasses import dataclass, field
from threading import Event, Thread
from typing import Any
from unittest.mock import Mock

import numpy as np
import pytest

from application import drone_autolanding_service as autolanding_module
from domain.buffer import DroneStatusBufferPort, FrameBufferPort, PoseBufferPort
from domain.content_streamer import ContentStreamer
from domain.models import Pose3D, TargetedMarker


@dataclass
class FakeTrackerCamera:
    fps: int = 30

    def open(self) -> None:
        return None

    def get_fps(self) -> int:
        return self.fps

    def close(self) -> None:
        return None


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

    def close(self) -> None:
        return None


def build_service(
    drone: FakeDrone, streamer: ContentStreamer | None = None, *, telemetry_dps: float = 5
) -> autolanding_module.DroneAutolandingService:
    frame_buffer = Mock(spec=FrameBufferPort)
    frame_buffer.get_value.return_value = (None, None)
    pose_buffer = Mock(spec=PoseBufferPort)
    pose_buffer.get_uav_pose_value.return_value = None
    status_buffer = Mock(spec=DroneStatusBufferPort)
    status_buffer.get_value.return_value = None
    service = autolanding_module.DroneAutolandingService(
        drone=drone,
        tracker=FakeTracker(TargetedMarker(id=29, length=0.896, dictionary=0)),
        content_streamer=streamer if streamer is not None else Mock(spec=ContentStreamer),
        frame_buffer=frame_buffer,
        pose_buffer=pose_buffer,
        drone_status_buffer=status_buffer,
        telemetry_dps=telemetry_dps,
    )
    drone.service = service
    return service


@pytest.mark.parametrize(
    ("reported_relative_altitude", "tracked_pose_z", "expected_z"),
    [
        (6.2, 1.4, 1.4),
        (4.5, 1.4, 1.4),
    ],
)
def test_precision_landing_preserves_target_distance_regardless_of_relative_altitude(
    reported_relative_altitude: float,
    tracked_pose_z: float,
    expected_z: float,
    sample_drone_status,
) -> None:
    drone = FakeDrone(sample_drone_status(relative_altitude=reported_relative_altitude))
    service = build_service(drone)
    service.pose_buffer.get_uav_pose_value.return_value = Pose3D(x=0.2, y=-0.3, z=tracked_pose_z)
    service._tracking_started = True

    service._landing_target_loop()

    assert drone.land_mode_calls == 1
    assert drone.land_calls == [(Pose3D(x=0.2, y=-0.3, z=expected_z), (0.896, 0.896))]


def test_telemetry_payload_merges_latest_tracking_metadata_with_current_drone_status(sample_drone_status) -> None:
    drone_status = sample_drone_status(relative_altitude=3.2)
    service = build_service(FakeDrone(drone_status))
    service.frame_buffer.get_value.return_value = (
        np.zeros((2, 2, 3), dtype=np.uint8),
        {
            "status": 1,
            "marker_id": 29,
            "poses": {
                "estimated_pose_from_camera": {"x": 0.1, "y": 0.2, "z": 1.4},
                "estimated_pose_to_uav": {"x": -0.2, "y": 0.1, "z": 1.4},
            },
        },
    )
    service.drone_status_buffer.get_value.return_value = drone_status

    payload = service._build_telemetry_payload()

    assert payload["status"] == 1
    assert payload["marker_id"] == 29
    assert payload["poses"]["estimated_pose_to_uav"] == {"x": -0.2, "y": 0.1, "z": 1.4}
    assert payload["drone"]["relative_altitude"] == pytest.approx(3.2)
    assert payload["drone"]["mode"] == drone_status.mode


def test_telemetry_is_sent_through_the_injected_streamer(sample_drone_status) -> None:
    streamer = Mock(spec=ContentStreamer)
    service = build_service(FakeDrone(sample_drone_status()), streamer)
    service.frame_buffer.get_value.return_value = (None, {"marker_id": 29})
    streamer.send_data.side_effect = lambda _payload: setattr(service, "_tracking_started", False)
    service._tracking_started = True

    service._telemetry_loop()

    streamer.send_data.assert_called_once_with({"marker_id": 29})


def test_telemetry_cadence_uses_configured_dps_instead_of_camera_fps(monkeypatch, sample_drone_status) -> None:
    service = build_service(FakeDrone(sample_drone_status()), telemetry_dps=4)
    monkeypatch.setattr(autolanding_module.time, "monotonic", lambda: 0)
    service._stop_event = Mock()
    service._stop_event.wait.side_effect = lambda _delay: setattr(service, "_tracking_started", False)
    service._tracking_started = True

    service._telemetry_loop()

    service._stop_event.wait.assert_called_once_with(0.25)


def test_landing_target_transmission_is_paced_without_incoming_messages(monkeypatch, sample_drone_status) -> None:
    drone = FakeDrone(sample_drone_status())
    service = build_service(drone)
    monkeypatch.setattr(autolanding_module.time, "monotonic", lambda: 0)
    drone.get_status = Mock(return_value=drone.status)
    service.pose_buffer.get_uav_pose_value.return_value = Pose3D(x=0, y=0, z=2)
    service._stop_event = Mock()

    def wait_one_cycle(_delay):
        if len(drone.land_calls) == 3:
            service._tracking_started = False

    service._stop_event.wait.side_effect = wait_one_cycle
    service._tracking_started = True

    service._landing_target_loop()

    assert len(drone.land_calls) == 3
    assert service._stop_event.wait.call_count == 3
    service._stop_event.wait.assert_called_with(pytest.approx(1 / 30))


@pytest.mark.parametrize("dps", [0, -1, float("nan"), float("inf")])
def test_invalid_telemetry_cadence_is_rejected(sample_drone_status, dps) -> None:
    with pytest.raises(ValueError, match="telemetry_dps"):
        build_service(FakeDrone(sample_drone_status()), telemetry_dps=dps)


@pytest.mark.parametrize("worker", ["tracking", "telemetry", "streaming"])
def test_worker_failure_reaches_main_loop_and_requests_shutdown(sample_drone_status, worker) -> None:
    service = build_service(FakeDrone(sample_drone_status()))
    original_error = OSError(f"{worker} failed")
    service._tracking_started = True
    service._start_worker(worker, Mock(side_effect=original_error))
    assert service._stop_event.wait(2)

    with pytest.raises(RuntimeError, match=worker) as failure:
        service.perform_precision_landing()

    assert failure.value.__cause__ is original_error
    assert not service._tracking_started
    service.stop()


def test_failed_land_mode_request_prevents_target_emissions(sample_drone_status) -> None:
    drone = FakeDrone(sample_drone_status())
    drone.activate_land_mode = Mock(side_effect=TimeoutError("mode rejected"))
    service = build_service(drone)
    service._tracking_started = True

    with pytest.raises(TimeoutError, match="mode rejected"):
        service.perform_precision_landing()

    assert not drone.land_calls


def test_shutdown_closes_every_resource_even_if_one_fails(sample_drone_status) -> None:
    service = build_service(FakeDrone(sample_drone_status()))
    service.aruco_tracker.camera.close = Mock(side_effect=OSError("camera close failed"))
    service.drone.close = Mock()

    with pytest.raises(RuntimeError, match="resource cleanup failed"):
        service.stop()

    service.content_streamer.stop.assert_called_once()
    service.drone.close.assert_called_once()
    service.stop()
    service.drone.close.assert_called_once()


def test_shutdown_is_bounded_when_a_driver_cannot_be_interrupted(sample_drone_status) -> None:
    service = build_service(FakeDrone(sample_drone_status()))
    blocked = Event()
    service.shutdown_timeout_s = 0.05
    thread = Thread(target=blocked.wait, name="stuck-camera", daemon=True)
    service._threads["tracking"] = thread
    thread.start()
    try:
        with pytest.raises(TimeoutError, match="tracking"):
            service.stop()
    finally:
        blocked.set()
        thread.join(timeout=1)


def test_expired_heartbeat_prevents_landing_target_emission(sample_drone_status):
    drone = FakeDrone(sample_drone_status(last_heartbeat_s=1))
    service = build_service(drone)
    service.pose_buffer.get_uav_pose_value.return_value = Pose3D(0, 0, 2)
    service._tracking_started = True
    service._landing_target_loop()
    assert not drone.land_calls
