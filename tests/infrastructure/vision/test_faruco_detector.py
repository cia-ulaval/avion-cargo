from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from domain.models import CalibrationData, TargetedMarker
from infrastructure.vision.faruco_detector import FArUcoDetector


def target() -> TargetedMarker:
    return TargetedMarker(
        id=7,
        length=0.896,
        marker_type="faruco",
        fractal_type="FRACTAL_3L_6",
    )


def calibration() -> CalibrationData:
    return CalibrationData(
        camera_matrix=np.array([[500.0, 0.0, 200.0], [0.0, 500.0, 200.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
        camera_width=400,
        camera_height=400,
    )


def bridge_marker(marker_id: int, *, with_pose: bool = True) -> SimpleNamespace:
    return SimpleNamespace(
        id=marker_id,
        corners=np.array([[10, 10], [110, 10], [110, 110], [10, 110]], dtype=np.float32),
        rvec=np.array([[0.1], [0.2], [0.3]]) if with_pose else None,
        tvec=np.array([[1.0], [2.0], [3.0]]) if with_pose else None,
        reprojection_error_px=0.25 if with_pose else None,
    )


def test_adapter_filters_by_fractal_id_and_preserves_bridge_pose() -> None:
    bridge = Mock()
    bridge.detect.return_value = SimpleNamespace(markers=[bridge_marker(8), bridge_marker(7)])
    detector = FArUcoDetector(target(), calibration(), bridge_detector=bridge)

    frame = np.zeros((400, 400, 3), dtype=np.uint8)
    detections = detector.detect(frame, target())

    assert len(detections) == 1
    detection = detections[0]
    assert detection.marker_id == 7
    assert detection.corners.shape == (1, 4, 2)
    assert detection.rotation_vectors.shape == (1, 1, 3)
    assert detection.translation_vectors.shape == (1, 1, 3)
    assert detection.reprojection_error_px == 0.25
    np.testing.assert_allclose(detection.translation_vectors.reshape(-1), [1.0, 2.0, 3.0])
    bridge.detect.assert_called_once_with(frame, include_points=False)


def test_adapter_keeps_detection_when_bridge_cannot_estimate_pose() -> None:
    bridge = Mock()
    bridge.detect.return_value = SimpleNamespace(markers=[bridge_marker(7, with_pose=False)])
    detector = FArUcoDetector(target(), calibration(), bridge_detector=bridge)

    detections = detector.detect(np.zeros((400, 400, 3), dtype=np.uint8), target())

    assert len(detections) == 1
    assert detections[0].rotation_vectors is None
    assert detections[0].translation_vectors is None


def test_close_is_forwarded_to_bridge_worker() -> None:
    bridge = Mock()
    detector = FArUcoDetector(target(), calibration(), bridge_detector=bridge)

    detector.close()

    bridge.close.assert_called_once_with()
