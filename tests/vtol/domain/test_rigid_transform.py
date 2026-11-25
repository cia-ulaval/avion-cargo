import math

import cv2
import numpy as np
import pytest

from vtol.domain.camera_mount import CameraMount
from vtol.domain.errors import InvalidPoseError
from vtol.domain.rigid_transform import RigidTransform


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


@pytest.mark.parametrize("axis", [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, -2, 3]])
@pytest.mark.parametrize("angle", [0, 1e-10, math.pi - 1e-9, math.pi, math.pi + 1e-9])
def test_quaternion_rotates_points_like_opencv_including_half_turns(axis, angle):
    vector = np.array(axis, dtype=float)
    rvec = vector / np.linalg.norm(vector) * angle
    transform = RigidTransform.from_vectors(rvec, [0, 0, 5])
    w, *xyz = transform.quaternion_wxyz()
    xyz = np.array(xyz)
    point = np.array([0.3, -0.7, 1.2])
    rotated = point + 2 * np.cross(xyz, np.cross(xyz, point) + w * point)
    np.testing.assert_allclose(rotated, cv2.Rodrigues(rvec)[0] @ point, atol=1e-9)
    assert w * w + xyz @ xyz == pytest.approx(1)


@pytest.mark.parametrize("matrix", [np.eye(3), np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1]), np.ones((4, 4))])
def test_invalid_rigid_transforms_are_rejected(matrix):
    with pytest.raises(InvalidPoseError):
        RigidTransform(matrix)
