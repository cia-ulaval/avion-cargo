import time

import numpy as np

from domain.models import Pose3D
from infrastructure.camera.frame_buffer import FrameBuffer
from infrastructure.vision.pose_buffer import PoseBuffer


def test_expired_capture_never_becomes_a_fresh_pose_or_image():
    frame_buffer, pose_buffer = FrameBuffer(), PoseBuffer()
    timestamp = time.monotonic() - 2
    frame_buffer.set_value(np.ones((2, 2, 3)), {'status': 1}, timestamp=timestamp)
    pose_buffer.set_value(Pose3D(1, 2, 3), timestamp=timestamp)
    pose_buffer.set_uav_pose_value(Pose3D(-2, 1, 3), timestamp=timestamp)
    assert pose_buffer.get_value() is None
    assert pose_buffer.get_uav_pose_value() is None
    image, metadata = frame_buffer.get_value()
    assert image is None
    assert metadata['status'] == 2
    assert metadata['poses']['estimated_pose_to_uav'] is None
    assert metadata['stale'] is True


def test_frame_buffer_owns_its_data_and_returns_independent_snapshots():
    buffer = FrameBuffer()
    image, metadata = np.zeros((2, 2, 3)), {'poses': {'x': 1}}
    buffer.set_value(image, metadata)
    image[:] = 1
    metadata['poses']['x'] = 2
    first, first_metadata = buffer.get_value()
    assert not first.any()
    assert first_metadata['poses']['x'] == 1
    first[:] = 3
    first_metadata['poses']['x'] = 4
    second, second_metadata = buffer.get_value()
    assert not second.any()
    assert second_metadata['poses']['x'] == 1
