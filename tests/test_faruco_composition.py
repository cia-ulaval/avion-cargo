import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import composition
from domain.models import CalibrationData, TargetedMarker
from domain.tracking import TrackingStatus


def test_tracking_builder_selects_faruco_adapter_for_faruco_targets(monkeypatch) -> None:
    target = TargetedMarker(
        id=None,
        length=0.896,
        marker_type="faruco",
        fractal_type="FRACTAL_3L_6",
    )
    calibration = CalibrationData(
        camera_matrix=np.array([[500.0, 0.0, 200.0], [0.0, 500.0, 200.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
        camera_width=400,
        camera_height=400,
    )
    detector = Mock(name="faruco-detector")
    faruco_factory = Mock(return_value=detector)
    monkeypatch.setattr(composition, "FArUcoDetector", faruco_factory)

    service = composition.build_tracking_service(
        camera=Mock(),
        target=target,
        detector_config=None,
        calibration_data=calibration,
    )

    assert service.detector is detector
    assert service.annotator is detector
    faruco_factory.assert_called_once_with(target, calibration)


@pytest.mark.parametrize("has_pose", [False, True])
def test_tracking_uses_the_current_bridge_annotation_with_or_without_pose(monkeypatch, has_pose) -> None:
    frame = np.zeros((400, 400, 3), dtype=np.uint8)
    annotated_frame = np.full_like(frame, 255)
    marker = SimpleNamespace(
        id=0,
        corners=np.float32([[1, 1], [5, 1], [5, 5], [1, 5]]),
        rvec=np.float64([0.1, 0.2, 0.3]) if has_pose else None,
        tvec=np.float64([1, 2, 3]) if has_pose else None,
        reprojection_error_px=9.0 if has_pose else None,
    )
    bridge_result = Mock(markers=[marker])
    bridge_result.annotate.return_value = annotated_frame
    empty_result = Mock(markers=[])
    bridge = Mock()
    bridge.detect.side_effect = [bridge_result, empty_result]
    monkeypatch.setitem(sys.modules, "faruco_bridge", SimpleNamespace(Detector=Mock(return_value=bridge)))
    camera = Mock()
    camera.get_frame.return_value = frame
    target = TargetedMarker(None, 0.896, marker_type="faruco", fractal_type="FRACTAL_3L_6")
    calibration = CalibrationData(
        camera_matrix=np.float64([[500, 0, 200], [0, 500, 200], [0, 0, 1]]),
        dist_coeffs=np.zeros(5),
        camera_width=400,
        camera_height=400,
    )
    service = composition.build_tracking_service(camera, target, None, calibration)
    service.pose_estimator = Mock()

    returned_frame, result = service.track_target()

    assert returned_frame is annotated_frame
    bridge_result.annotate.assert_called_once_with(frame)
    service.pose_estimator.estimate_pose.assert_not_called()
    if has_pose:
        assert result.status is TrackingStatus.DETECTED
        assert (result.pose.x, result.pose.y, result.pose.z) == (1, 2, 3)
    else:
        assert result.status is TrackingStatus.NOT_FOUND
        assert result.pose is None

    returned_frame, result = service.track_target()

    assert returned_frame is frame
    assert result.status is TrackingStatus.NOT_FOUND
    bridge_result.annotate.assert_called_once()
