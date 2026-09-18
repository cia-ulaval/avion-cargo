from dataclasses import dataclass, field
from typing import Any
from unittest.mock import Mock

import numpy as np
import pytest

from application.tracking_service import TrackingService
from domain.frame_annotator import FrameAnnotator
from domain.models import CalibrationData, Pose3D, TargetedMarker
from domain.tracking import TrackingStatus


@dataclass
class FakeCamera:
    frame: np.ndarray
    fps: int = 30

    def get_frame(self) -> np.ndarray:
        return self.frame

    def get_fps(self) -> int:
        return self.fps


@dataclass
class FakeDetector:
    detections: list[tuple[int, np.ndarray]]
    calls: list[tuple[np.ndarray, TargetedMarker]] = field(default_factory=list)

    def detect(self, frame: np.ndarray, target: TargetedMarker) -> list[tuple[int, np.ndarray]]:
        self.calls.append((frame, target))
        return self.detections


@dataclass
class FakePoseEstimator:
    pose: Pose3D
    rotation_vectors: np.ndarray
    translation_vectors: np.ndarray
    calls: list[dict[str, Any]] = field(default_factory=list)

    def estimate_pose(
        self,
        *,
        corners: np.ndarray,
        marker_length_m: float,
        calib: CalibrationData,
        center: bool,
    ) -> tuple[Pose3D, np.ndarray, np.ndarray]:
        self.calls.append(
            {
                "corners": corners,
                "marker_length_m": marker_length_m,
                "calib": calib,
                "center": center,
            }
        )
        return self.pose, self.rotation_vectors, self.translation_vectors


def calibration_data(size=8) -> CalibrationData:
    return CalibrationData(
        camera_matrix=np.array([[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
        camera_width=size,
        camera_height=size,
    )


def test_tracking_returns_not_found_without_estimating_pose_or_drawing_when_marker_is_absent() -> None:
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    target = TargetedMarker(id=29, length=0.896, dictionary=0)
    detector = FakeDetector(detections=[])
    pose_estimator = FakePoseEstimator(
        pose=Pose3D(1.0, 2.0, 3.0),
        rotation_vectors=np.zeros((1, 1, 3)),
        translation_vectors=np.zeros((1, 1, 3)),
    )
    annotator = Mock(spec=FrameAnnotator)

    service = TrackingService(
        camera=FakeCamera(frame),
        detector=detector,
        pose_estimator=pose_estimator,
        target=target,
        calibration=calibration_data(),
        annotator=annotator,
    )

    returned_frame, result = service.track_target()

    assert returned_frame is frame
    assert result.status is TrackingStatus.NOT_FOUND
    assert result.pose is None
    assert result.uav_pose is None
    assert pose_estimator.calls == []
    assert detector.calls == [(frame, target)]
    annotator.annotate.assert_not_called()


def test_tracking_estimates_first_detection_draws_overlays_and_converts_pose_to_uav_axes() -> None:
    frame = np.zeros((12, 12, 3), dtype=np.uint8)
    first_corners = np.array([[[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]])
    ignored_corners = np.array([[[5.0, 5.0], [7.0, 5.0], [7.0, 7.0], [5.0, 7.0]]])
    rotation_vectors = np.array([[[0.1, 0.2, 0.3]]])
    translation_vectors = np.array([[[3.0, -2.0, 10.0]]])
    calibration = calibration_data(12)
    target = TargetedMarker(id=None, length=0.896, dictionary=0)
    detector = FakeDetector(detections=[(41, first_corners), (99, ignored_corners)])
    pose_estimator = FakePoseEstimator(
        pose=Pose3D(x=3.0, y=-2.0, z=10.0),
        rotation_vectors=rotation_vectors,
        translation_vectors=translation_vectors,
    )
    annotated_frame = frame.copy()
    annotator = Mock(spec=FrameAnnotator)
    annotator.annotate.return_value = annotated_frame

    service = TrackingService(
        camera=FakeCamera(frame),
        detector=detector,
        pose_estimator=pose_estimator,
        target=target,
        calibration=calibration,
        annotator=annotator,
    )

    returned_frame, result = service.track_target()

    assert returned_frame is annotated_frame
    assert result.status is TrackingStatus.DETECTED
    assert result.marker_id == 41
    assert result.pose == Pose3D(x=3.0, y=-2.0, z=10.0)
    assert result.uav_pose == Pose3D(x=2.0, y=3.0, z=10.0)

    assert len(pose_estimator.calls) == 1
    assert pose_estimator.calls[0]["marker_length_m"] == pytest.approx(0.896)
    assert pose_estimator.calls[0]["calib"] is calibration
    assert pose_estimator.calls[0]["center"] is True
    np.testing.assert_array_equal(pose_estimator.calls[0]["corners"], first_corners)

    annotator.annotate.assert_called_once()
    assert annotator.annotate.call_args.args[0] is frame
    annotation = annotator.annotate.call_args.kwargs
    assert annotation["marker_id"] == 41
    np.testing.assert_array_equal(annotation["corners"], first_corners)
    assert annotation["calibration"] is calibration
    assert annotation["rotation_vectors"] is rotation_vectors
    assert annotation["translation_vectors"] is translation_vectors


def test_wrong_capture_resolution_is_rejected_before_detection():
    detector = Mock()
    service = TrackingService(
        FakeCamera(np.zeros((10, 10, 3))), detector, Mock(), TargetedMarker(0, 0.1, 0), calibration_data(8), Mock()
    )
    with pytest.raises(ValueError, match="does not match calibration"):
        service.track_target()
    detector.detect.assert_not_called()
