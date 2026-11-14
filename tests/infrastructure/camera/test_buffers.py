import time

import numpy as np

from domain.models import Pose3D
from domain.rigid_transform import RigidTransform
from domain.tracking import TrackingResult
from infrastructure.camera.frame_buffer import FrameBuffer
from infrastructure.vision.pose_buffer import PoseBuffer


def test_expired_capture_never_becomes_a_fresh_pose_or_image():
    frame_buffer, pose_buffer = FrameBuffer(), PoseBuffer()
    timestamp = time.monotonic() - 2
    frame_buffer.set_value(np.ones((2, 2, 3)), {"status": 1}, timestamp=timestamp)
    pose_buffer.set_value(Pose3D(1, 2, 3), timestamp=timestamp)
    pose_buffer.set_uav_pose_value(Pose3D(-2, 1, 3), timestamp=timestamp)
    assert pose_buffer.get_value() is None
    assert pose_buffer.get_uav_pose_value() is None
    image, metadata = frame_buffer.get_value()
    assert image is None
    assert metadata["status"] == 2
    assert metadata["poses"]["estimated_pose_to_uav"] is None
    assert metadata["stale"] is True


def test_frame_buffer_owns_its_data_and_returns_independent_snapshots():
    buffer = FrameBuffer()
    image, metadata = np.zeros((2, 2, 3)), {"poses": {"x": 1}}
    buffer.set_value(image, metadata)
    image[:] = 1
    metadata["poses"]["x"] = 2
    first, first_metadata = buffer.get_value()
    assert not first.any()
    assert first_metadata["poses"]["x"] == 1
    first[:] = 3
    first_metadata["poses"]["x"] = 4
    second, second_metadata = buffer.get_value()
    assert not second.any()
    assert second_metadata["poses"]["x"] == 1


def test_full_transform_snapshot_keeps_capture_age_and_is_cleared_on_target_loss(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("time.monotonic", lambda: clock[0])
    buffer = PoseBuffer(max_age_s=0.5)
    transform = RigidTransform.from_vectors([0, 0, 0.5], [0, 0, 5])
    result = TrackingResult.detected(transform.position, 29, captured_at_s=100, body_from_marker=transform)
    buffer.set_tracking_result(result)
    assert buffer.get_tracking_result() is result
    clock[0] = 100.6
    assert buffer.get_tracking_result() is None
    buffer.set_tracking_result(result)  # Republishing an old observation does not refresh it.
    assert buffer.get_tracking_result() is None
    buffer.set_tracking_result(None)
    assert buffer.get_tracking_result() is None
