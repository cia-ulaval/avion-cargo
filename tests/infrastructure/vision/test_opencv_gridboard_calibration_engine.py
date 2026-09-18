import cv2
import numpy as np
import pytest

from domain.models import TargetedMarker
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetector, OpenCVArucoDetectorConfig
from infrastructure.vision.opencv_gridboard_calibration_engine import (
    GridBoardSpec,
    OpenCVGridBoardCameraCalibrationEngine,
)


def make_board_frames(dictionary_id: int) -> list[np.ndarray]:
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.GridBoard((4, 5), 0.03, 0.01, dictionary)
    board_image = board.generateImage((1500, 1900))
    image_corners = np.float32([[0, 0], [1499, 0], [1499, 1899], [0, 1899]])
    board_corners = np.float32([[0, 0, 0], [0.15, 0, 0], [0.15, 0.19, 0], [0, 0.19, 0]])
    camera_matrix = np.float64([[1000, 0, 480], [0, 1000, 360], [0, 0, 1]])
    poses = [
        ((0.1, 0.2, 0.0), (-0.07, -0.09, 0.60)),
        ((-0.3, 0.1, 0.1), (-0.10, -0.06, 0.65)),
        ((0.2, -0.3, -0.1), (-0.04, -0.12, 0.55)),
        ((-0.2, -0.2, 0.2), (-0.08, -0.10, 0.70)),
        ((0.4, 0.1, -0.2), (-0.03, -0.08, 0.60)),
        ((0.1, -0.4, 0.1), (-0.12, -0.09, 0.65)),
    ]
    frames = []
    for rotation, translation in poses:
        projected, _ = cv2.projectPoints(
            board_corners, np.float64(rotation), np.float64(translation), camera_matrix, np.zeros(5)
        )
        transform = cv2.getPerspectiveTransform(image_corners, projected.reshape(4, 2))
        frame = cv2.warpPerspective(board_image, transform, (960, 720), borderValue=255)
        frames.append(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
    return frames


@pytest.mark.parametrize("dictionary_id", range(17))
def test_collector_and_calibration_engine_use_the_same_opencv_dictionary(dictionary_id: int) -> None:
    frames = make_board_frames(dictionary_id)
    detector = OpenCVArucoDetector(OpenCVArucoDetectorConfig(dictionary_id=dictionary_id))
    target = TargetedMarker(id=None, length=0.03, dictionary=dictionary_id)
    for frame in frames:
        assert {marker_id for marker_id, _ in detector.detect(frame, target)} == set(range(20))

    engine = OpenCVGridBoardCameraCalibrationEngine(GridBoardSpec(4, 5, 0.03, 0.01, dictionary_id))
    report = engine.calibrate_from_frames(frames)

    assert (report.image_width, report.image_height) == (960, 720)
    assert np.isfinite(report.camera_matrix).all()
    assert np.isfinite(report.camera_distortion_matrix).all()
    assert report.avg_reprojection_error < 1.0
    np.testing.assert_allclose([report.camera_matrix[0, 0], report.camera_matrix[1, 1]], [1000, 1000], rtol=0.05)


@pytest.mark.parametrize("dictionary_id", [-1, 17])
def test_calibration_engine_rejects_dictionary_outside_supported_range(dictionary_id: int) -> None:
    with pytest.raises(ValueError, match="dictionary_id must be in 0..16"):
        OpenCVGridBoardCameraCalibrationEngine(GridBoardSpec(4, 5, 0.03, 0.01, dictionary_id))


def test_too_few_distinct_calibration_views_are_rejected():
    from infrastructure.vision.opencv_gridboard_calibration_engine import NotEnoughFramesError

    frames = make_board_frames(0)
    engine = OpenCVGridBoardCameraCalibrationEngine(GridBoardSpec(4, 5, 0.03, 0.01, 0))
    with pytest.raises(NotEnoughFramesError):
        engine.calibrate_from_frames(frames[:2])
    with pytest.raises(NotEnoughFramesError):
        engine.calibrate_from_frames([frames[0]] * 10)


def test_calibration_refuses_mixed_resolutions():
    from domain.errors import InvalidCalibrationError

    frames = make_board_frames(0)
    engine = OpenCVGridBoardCameraCalibrationEngine(GridBoardSpec(4, 5, 0.03, 0.01, 0))
    with pytest.raises(InvalidCalibrationError, match="same resolution"):
        engine.calibrate_from_frames([frames[0], frames[1][:300]])
