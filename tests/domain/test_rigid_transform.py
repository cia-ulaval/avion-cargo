import math
from dataclasses import replace

import cv2
import numpy as np
import pytest

from domain.camera_mount import CameraMount
from domain.errors import InvalidPoseError
from domain.rigid_transform import RigidTransform
from domain.yaw_alignment import YawAlignmentConfig, marker_heading_error_rad


def test_composition_moves_an_offset_marker_point_and_preserves_its_orientation():
    # Camera origin is one meter forward; marker +X points down in camera axes.
    mount = CameraMount(translation_m=(1, 0, 0))
    camera_from_marker = RigidTransform.from_vectors([0, 0, math.pi / 2], [2, 3, 4])
    body_from_marker = mount.transform_to_body(camera_from_marker)
    np.testing.assert_allclose(body_from_marker.as_matrix() @ [0, 0, 0, 1], [-2, 2, 4, 1])
    np.testing.assert_allclose(body_from_marker.as_matrix() @ [0.3, 0, 0, 1], [-2.3, 2, 4, 1])
    matrix = body_from_marker.as_matrix()
    matrix[0, 3] = 999
    assert body_from_marker.position.x == -2


@pytest.mark.parametrize("rvec", [[0, 0, 0], [1e-10, -2e-10, 3e-10], [2.8, 0.3, -0.1], [0, math.pi, 0]])
def test_axis_angle_conversion_matches_opencv(rvec):
    transform = RigidTransform.from_vectors(rvec, [0.1, 0.2, 3])
    np.testing.assert_allclose(transform.as_matrix()[:3, :3], cv2.Rodrigues(np.float64(rvec))[0], atol=1e-9)


@pytest.mark.parametrize("matrix", [np.eye(3), np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1]), np.ones((4, 4))])
def test_invalid_rigid_transforms_are_rejected(matrix):
    with pytest.raises(InvalidPoseError):
        RigidTransform(matrix)


def test_heading_error_compensates_body_tilt_and_wraps_across_north(sample_drone_status):
    roll, pitch, yaw = np.radians([20, -15, 170])
    rx = cv2.Rodrigues(np.float64([roll, 0, 0]))[0]
    ry = cv2.Rodrigues(np.float64([0, pitch, 0]))[0]
    rz = cv2.Rodrigues(np.float64([0, 0, yaw]))[0]
    marker_in_ned = cv2.Rodrigues(np.float64([0, 0, math.radians(-170)]))[0]
    matrix = np.eye(4)
    matrix[:3, :3] = (rz @ ry @ rx).T @ marker_in_ned
    status = replace(sample_drone_status(), roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw)
    assert math.degrees(marker_heading_error_rad(RigidTransform(matrix), status)) == pytest.approx(20)


def test_vertical_marker_axis_has_no_heading(sample_drone_status):
    transform = RigidTransform.from_vectors([0, math.pi / 2, 0], [0, 0, 3])
    status = replace(sample_drone_status(), roll_rad=0, pitch_rad=0, yaw_rad=0)
    with pytest.raises(ValueError, match="horizontal"):
        marker_heading_error_rad(transform, status)


@pytest.mark.parametrize(
    "values",
    [
        {"trigger_deg": 2},
        {"gain_per_rad": float("nan")},
        {"max_age_s": 0},
        {"target_loss_grace_s": 0},
        {"target_loss_grace_s": float("inf")},
        {"enabled": "true"},
        {"max_yaw_input": 2},
        {"detection_stable_time_s": 0},
        {"detection_loss_timeout_s": float("inf")},
        {"attempt_timeout_s": -1},
        {"max_attempts": 0},
        {"max_attempts": -1},
        {"max_attempts": 1.5},
        {"max_attempts": True},
        {"max_attempts": "3"},
        {"retry_descent_m": -1},
        {"retry_descent_m": float("nan")},
        {"retry_descent_m": True},
    ],
)
def test_invalid_alignment_settings_are_rejected(values):
    with pytest.raises(ValueError):
        YawAlignmentConfig(**values)
