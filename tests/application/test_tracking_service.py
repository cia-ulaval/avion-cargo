from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pytest

from application.tracking_service import TrackingService
from domain.models import CalibrationData, Pose3D, TargetedMarker
from domain.tracking import TrackingStatus
from infrastructure.vision.opencv_frame_manipution_tool import FrameManipulationTool


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


def calibration_data() -> CalibrationData:
    return CalibrationData(
        camera_matrix=np.array([[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
    )


def test_tracking_returns_not_found_without_estimating_pose_or_drawing_when_marker_is_absent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    target = TargetedMarker(id=29, length=0.896, dictionary=0)
    detector = FakeDetector(detections=[])
    pose_estimator = FakePoseEstimator(
        pose=Pose3D(1.0, 2.0, 3.0),
        rotation_vectors=np.zeros((1, 1, 3)),
        translation_vectors=np.zeros((1, 1, 3)),
    )
    monkeypatch.setattr(FrameManipulationTool, "draw_detected_markers", pytest.fail)
    monkeypatch.setattr(FrameManipulationTool, "draw_axes_for_poses", pytest.fail)

    service = TrackingService(
        camera=FakeCamera(frame),
        detector=detector,
        pose_estimator=pose_estimator,
        target=target,
        calibration=calibration_data(),
    )

    returned_frame, result = service.track_target()

    assert returned_frame is frame
    assert result.status is TrackingStatus.NOT_FOUND
    assert result.pose is None
    assert result.uav_pose is None
    assert pose_estimator.calls == []
    assert detector.calls == [(frame, target)]


def test_tracking_estimates_first_detection_draws_overlays_and_converts_pose_to_uav_axes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    frame = np.zeros((12, 12, 3), dtype=np.uint8)
    first_corners = np.array([[[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]])
    ignored_corners = np.array([[[5.0, 5.0], [7.0, 5.0], [7.0, 7.0], [5.0, 7.0]]])
    rotation_vectors = np.array([[[0.1, 0.2, 0.3]]])
    translation_vectors = np.array([[[3.0, -2.0, 10.0]]])
    calibration = calibration_data()
    target = TargetedMarker(id=None, length=0.896, dictionary=0)
    detector = FakeDetector(detections=[(41, first_corners), (99, ignored_corners)])
    pose_estimator = FakePoseEstimator(
        pose=Pose3D(x=3.0, y=-2.0, z=10.0),
        rotation_vectors=rotation_vectors,
        translation_vectors=translation_vectors,
    )
    draw_calls: list[tuple[str, tuple[Any, ...]]] = []

    def draw_detected_markers(*args: Any, **_kwargs: Any) -> np.ndarray:
        draw_calls.append(("markers", args))
        return args[0]

    def draw_axes_for_poses(*args: Any, **_kwargs: Any) -> np.ndarray:
        draw_calls.append(("axes", args))
        return args[0]

    monkeypatch.setattr(FrameManipulationTool, "draw_detected_markers", draw_detected_markers)
    monkeypatch.setattr(FrameManipulationTool, "draw_axes_for_poses", draw_axes_for_poses)

    service = TrackingService(
        camera=FakeCamera(frame),
        detector=detector,
        pose_estimator=pose_estimator,
        target=target,
        calibration=calibration,
    )

    returned_frame, result = service.track_target()

    assert returned_frame is frame
    assert result.status is TrackingStatus.DETECTED
    assert result.marker_id == 41
    assert result.pose == Pose3D(x=3.0, y=-2.0, z=10.0)
    assert result.uav_pose == Pose3D(x=2.0, y=3.0, z=10.0)

    assert len(pose_estimator.calls) == 1
    assert pose_estimator.calls[0]["marker_length_m"] == pytest.approx(0.896)
    assert pose_estimator.calls[0]["calib"] is calibration
    assert pose_estimator.calls[0]["center"] is True
    np.testing.assert_array_equal(pose_estimator.calls[0]["corners"], first_corners)

    assert [name for name, _args in draw_calls] == ["markers", "axes"]
    np.testing.assert_array_equal(draw_calls[0][1][1][0], first_corners)
    np.testing.assert_array_equal(draw_calls[0][1][2], np.array([[41]], dtype=np.int32))
    assert draw_calls[1][1][1] is calibration.camera_matrix
    assert draw_calls[1][1][2] is calibration.dist_coeffs
    assert draw_calls[1][1][3] is rotation_vectors
    assert draw_calls[1][1][4] is translation_vectors
