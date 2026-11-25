import time
from dataclasses import dataclass, field
from threading import Event, Thread
from typing import Any
from unittest.mock import Mock

import numpy as np
import pytest

from vtol.application.autolanding import drone_autolanding_service as autolanding_module
from vtol.domain.buffer import DroneStatusBufferPort, FrameBufferPort, PoseBufferPort
from vtol.domain.content_streamer import ContentStreamer
from vtol.domain.models import Pose3D, TargetedMarker
from vtol.domain.rigid_transform import RigidTransform
from vtol.domain.tracking import TrackingResult


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
    target_messages: list[tuple[Pose3D, tuple[float, float]]] = field(default_factory=list)
    measurements: list[dict] = field(default_factory=list)

    def get_status(self) -> Any:
        assert self.service is not None
        self.service._tracking_started = False
        return self.status

    def send_landing_target(self, position: Pose3D, target_size: tuple[float, float], **measurement) -> None:
        self.target_messages.append((position, target_size))
        self.measurements.append(measurement)

    def close(self) -> None:
        return None


def detection(pose, captured_at_s=None, transform=None):
    return TrackingResult.detected(
        pose,
        29,
        uav_pose=pose,
        captured_at_s=time.monotonic() if captured_at_s is None else captured_at_s,
        body_from_marker=transform,
    )


def build_service(
    drone: FakeDrone, streamer: ContentStreamer | None = None, *, telemetry_dps: float = 5
) -> autolanding_module.DroneAutolandingService:
    frame_buffer = Mock(spec=FrameBufferPort)
    frame_buffer.get_value.return_value = (None, None)
    pose_buffer = Mock(spec=PoseBufferPort)
    pose_buffer.get_tracking_result.return_value = None
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
def test_target_publication_preserves_target_distance_regardless_of_relative_altitude(
    reported_relative_altitude: float,
    tracked_pose_z: float,
    expected_z: float,
    sample_drone_status,
) -> None:
    drone = FakeDrone(sample_drone_status(relative_altitude=reported_relative_altitude))
    service = build_service(drone)
    service.pose_buffer.get_tracking_result.return_value = detection(Pose3D(x=0.2, y=-0.3, z=tracked_pose_z))
    service._tracking_started = True

    service._target_publication_loop()

    assert drone.target_messages == [(Pose3D(x=0.2, y=-0.3, z=expected_z), (0.896, 0.896))]


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
    assert payload["drone"]["altitude_msl_m"] == drone_status.relative_altitude_ms
    assert payload["drone"]["vertical_speed_mps"] == drone_status.speed


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


def test_landing_target_transmission_is_paced_without_incoming_messages(monkeypatch, sample_drone_status):
    drone = FakeDrone(sample_drone_status())
    service = build_service(drone)
    clock = [100.0]
    monkeypatch.setattr(autolanding_module.time, "monotonic", lambda: clock[0])
    drone.get_status = Mock(return_value=drone.status)
    service.pose_buffer.get_tracking_result.side_effect = lambda: detection(Pose3D(0, 0, 2))
    service._stop_event = Mock()

    def wait_one_cycle(delay):
        clock[0] += delay
        if len(drone.target_messages) == 3:
            service.request_stop()

    service._stop_event.wait.side_effect = wait_one_cycle
    service._tracking_started = True
    service._target_publication_loop()
    assert len(drone.target_messages) == 3
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
        service.publish_target_positions()

    assert failure.value.__cause__ is original_error
    assert not service._tracking_started
    service.stop()


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
    service.pose_buffer.get_tracking_result.return_value = detection(Pose3D(0, 0, 2))
    service._tracking_started = True
    service._target_publication_loop()
    assert not drone.target_messages


@pytest.mark.parametrize("visible_cycles", [set(), {5}, {0, 1, 10, 11}])
def test_publication_continues_for_minutes_without_target_or_after_target_loss(
    monkeypatch, sample_drone_status, visible_cycles
):
    drone = FakeDrone(sample_drone_status())
    service = build_service(drone)
    drone.get_status = Mock(return_value=drone.status)
    pose = Pose3D(0.2, -0.3, 2)
    service.pose_buffer.get_tracking_result.side_effect = [
        detection(pose, captured_at_s=cycle * 30) if cycle in visible_cycles else None for cycle in range(12)
    ]
    elapsed = [0.0]
    monkeypatch.setattr(autolanding_module.time, "monotonic", lambda: elapsed[0])
    service._stop_event = Mock()

    def advance_clock(_delay):
        elapsed[0] += 30
        if elapsed[0] >= 360:
            service.request_stop()

    service._stop_event.wait.side_effect = advance_clock
    service._tracking_started = True

    service.publish_target_positions()

    assert elapsed[0] == 360
    assert drone.get_status.call_count == 12
    assert service.drone_status_buffer.set_value.call_count == 12
    assert drone.target_messages == [(pose, (0.896, 0.896))] * len(visible_cycles)


def test_transient_camera_timeout_clears_target_and_recovers(sample_drone_status):
    service = build_service(FakeDrone(sample_drone_status()))
    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    pose = Pose3D(0.1, 0.2, 1.0)
    recovered_result = type(
        "TrackingResultStub",
        (),
        {
            "status": type("Status", (), {"name": "DETECTED"})(),
            "captured_at_s": 12.0,
            "pose": pose,
            "uav_pose": pose,
            "to_dict": lambda self: {
                "status": 1,
                "marker_id": 29,
                "poses": {
                    "estimated_pose_from_camera": {"x": 0.1, "y": 0.2, "z": 1.0},
                    "estimated_pose_to_uav": {"x": 0.1, "y": 0.2, "z": 1.0},
                },
            },
        },
    )()
    service.aruco_tracker.track_target = Mock(side_effect=[TimeoutError("no fresh frame"), (frame, recovered_result)])
    service._tracking_started = True
    waits = 0

    def wait_for_next_frame(_delay):
        nonlocal waits
        waits += 1
        if waits == 2:
            service._tracking_started = False

    service._stop_event.wait = Mock(side_effect=wait_for_next_frame)

    service._tracking_target_loop()

    assert service.aruco_tracker.track_target.call_count == 2
    first_metadata = service.frame_buffer.set_value.call_args_list[0].args[1]
    assert first_metadata["camera_state"] == "STALE"
    assert first_metadata["poses"]["estimated_pose_to_uav"] is None
    service.pose_buffer.set_value.assert_any_call(None)
    service.pose_buffer.set_uav_pose_value.assert_any_call(None)
    recovered_metadata = service.frame_buffer.set_value.call_args_list[1].args[1]
    assert recovered_metadata["camera_state"] == "OK"
    service.pose_buffer.set_uav_pose_value.assert_called_with(pose, timestamp=12.0)
    service.pose_buffer.set_tracking_result.assert_any_call(None)
    service.pose_buffer.set_tracking_result.assert_called_with(recovered_result)


def test_camera_publishes_each_detection_and_loss(monkeypatch, sample_drone_status):
    clock = [100.0]
    monkeypatch.setattr(autolanding_module.time, "monotonic", lambda: clock[0])
    service = build_service(FakeDrone(sample_drone_status()))
    transform = RigidTransform.from_vectors([0, 0, 0.5], [0, 0, 5])
    frames = [
        TrackingResult.detected(transform.position, 29, captured_at_s=100, body_from_marker=transform),
        TrackingResult.not_found(captured_at_s=100.04),
        TrackingResult.detected(transform.position, 29, captured_at_s=100.08, body_from_marker=transform),
    ]
    service.aruco_tracker.track_target = Mock(side_effect=[(np.zeros((2, 2, 3)), result) for result in frames])

    def next_frame(_delay):
        clock[0] += 0.04
        if service.aruco_tracker.track_target.call_count == len(frames):
            service.request_stop()

    service._stop_event.wait = Mock(side_effect=next_frame)
    service._tracking_started = True
    service._tracking_target_loop()
    assert [call.args[0] for call in service.pose_buffer.set_tracking_result.call_args_list] == frames


@pytest.mark.parametrize("age", [-0.1, 0.41, 2])
def test_stale_or_future_detection_is_not_published(monkeypatch, sample_drone_status, age):
    monkeypatch.setattr(autolanding_module.time, "monotonic", lambda: 100)
    drone = FakeDrone(sample_drone_status())
    service = build_service(drone)
    service.pose_buffer.get_tracking_result.return_value = detection(Pose3D(0, 0, 5), 100 - age)
    service._tracking_started = True
    service._target_publication_loop()
    assert not drone.target_messages


def test_each_capture_is_published_once_with_its_orientation(monkeypatch, sample_drone_status):
    monkeypatch.setattr(autolanding_module.time, "monotonic", lambda: 100)
    drone = FakeDrone(sample_drone_status())
    service = build_service(drone)
    transform = RigidTransform.from_vectors([0, 0, 0.5], [0, 0, 5])
    service.pose_buffer.get_tracking_result.return_value = detection(transform.position, 100, transform)
    drone.get_status = Mock(return_value=drone.status)
    service._stop_event = Mock()
    cycles = []

    def wait(_delay):
        cycles.append(1)
        if len(cycles) == 3:
            service.request_stop()

    service._stop_event.wait.side_effect = wait
    service._tracking_started = True
    service._target_publication_loop()
    assert len(drone.target_messages) == 1
    assert drone.measurements == [{"body_from_marker": transform, "captured_at_s": 100}]


def test_tracking_starts_only_measurement_and_telemetry_workers(sample_drone_status):
    from vtol.domain.drone import Drone

    service = build_service(FakeDrone(sample_drone_status()))
    service.__init__(
        drone=Mock(spec=Drone),
        tracker=service.aruco_tracker,
        content_streamer=service.content_streamer,
        frame_buffer=service.frame_buffer,
        pose_buffer=service.pose_buffer,
        drone_status_buffer=service.drone_status_buffer,
    )
    service._start_worker = Mock()
    service.track_target()
    assert [call.args[0] for call in service._start_worker.call_args_list] == ["tracking", "telemetry"]
    service.stop()
    service.drone.close.assert_called_once_with()
