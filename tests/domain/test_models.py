import math

import numpy as np
import pytest

from domain.errors import InvalidCalibrationError, InvalidMarkerLengthError
from domain.models import CalibrationData, Pose3D, TargetedMarker


def test_targeted_marker_accepts_documented_aruco_dictionary_boundaries() -> None:
    first_supported = TargetedMarker(id=29, length=0.896, dictionary=0)
    last_supported = TargetedMarker(id=None, length=0.896, dictionary=16)

    assert first_supported.dictionary == 0
    assert last_supported.id is None


@pytest.mark.parametrize("dictionary_id", [-1, 17])
def test_targeted_marker_rejects_aruco_dictionary_outside_supported_opencv_range(dictionary_id: int) -> None:
    with pytest.raises(ValueError, match="dictionary"):
        TargetedMarker(id=29, length=0.896, dictionary=dictionary_id)


@pytest.mark.parametrize("length", [0.0, -0.1, math.inf, math.nan])
def test_targeted_marker_rejects_non_physical_marker_lengths(length: float) -> None:
    with pytest.raises(InvalidMarkerLengthError):
        TargetedMarker(id=29, length=length, dictionary=0)


def test_pose_angles_follow_camera_axis_convention() -> None:
    pose = Pose3D(x=1.0, y=-2.0, z=4.0)

    angle_x, angle_y = pose.to_angle()

    assert angle_x == pytest.approx(math.atan2(-2.0, 4.0))
    assert angle_y == pytest.approx(math.atan2(1.0, 4.0))


def test_calibration_data_rejects_non_finite_intrinsics_before_pose_estimation() -> None:
    camera_matrix = np.array(
        [
            [600.0, 0.0, 320.0],
            [0.0, math.inf, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )

    with pytest.raises(InvalidCalibrationError, match="non-finite"):
        CalibrationData(camera_matrix=camera_matrix, dist_coeffs=np.zeros(5))
