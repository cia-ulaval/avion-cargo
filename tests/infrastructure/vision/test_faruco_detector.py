import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from domain.models import CalibrationData, TargetedMarker
from infrastructure.vision.faruco_detector import FArUcoDetector


def target(preset="FRACTAL_5L_6"):
    return TargetedMarker(None, 1.792, marker_type="faruco", fractal_type=preset)


def calibration():
    return CalibrationData(
        camera_matrix=np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
        camera_width=640,
        camera_height=480,
    )


FRAME = np.zeros((480, 640, 3), dtype=np.uint8)


def marker(level=0, *, translation=(0.08, -0.04, 3.0), pose=True):
    return SimpleNamespace(
        id=level,
        corners=np.array([[1, 1], [11, 1], [11, 11], [1, 11]], dtype=np.float32),
        rvec=np.array([[3.1], [0.08], [0.02]]) if pose else None,
        tvec=np.asarray(translation).reshape(3, 1) if pose else None,
        reprojection_error_px=12.0 if pose else None,
    )


def detector_for(markers):
    bridge = Mock()
    bridge.detect.return_value = SimpleNamespace(markers=markers)
    return FArUcoDetector(target(), calibration(), bridge_detector=bridge), bridge


@pytest.mark.parametrize("level", range(5))
def test_bridge_pose_is_used_unchanged_on_the_first_frame_even_for_small_markers(level):
    observed = marker(level)
    detector, bridge = detector_for([observed])

    (detection,) = detector.detect(FRAME, target())

    assert detection.marker_id == level
    assert detection.corners.shape == (1, 4, 2)
    assert detection.rotation_vectors.shape == detection.translation_vectors.shape == (1, 1, 3)
    np.testing.assert_array_equal(detection.corners.reshape(4, 2), observed.corners)
    np.testing.assert_array_equal(detection.rotation_vectors.reshape(3), observed.rvec.reshape(3))
    np.testing.assert_array_equal(detection.translation_vectors.reshape(3), observed.tvec.reshape(3))
    assert detection.reprojection_error_px == observed.reprojection_error_px
    bridge.detect.assert_called_once_with(FRAME)


def test_all_bridge_detections_are_returned_in_their_original_order():
    detector, _ = detector_for([marker(3), marker(0), marker(1)])
    assert [detection.marker_id for detection in detector.detect(FRAME, target())] == [3, 0, 1]


def test_level_changes_position_jumps_and_reacquisition_are_immediate():
    detector, bridge = detector_for([])
    for observed in [marker(), marker(3, translation=(0.4, -0.2, 0.5)), None, marker(1)]:
        bridge.detect.return_value = SimpleNamespace(markers=[] if observed is None else [observed])
        detections = detector.detect(FRAME, target())
        if observed is None:
            assert detections == []
        else:
            (detection,) = detections
            assert detection.marker_id == observed.id
            np.testing.assert_array_equal(detection.translation_vectors.reshape(3), observed.tvec.reshape(3))


def test_bridge_detection_without_pose_keeps_its_corners_and_has_no_fallback_pose():
    observed = marker(pose=False)
    detector, _ = detector_for([observed])

    (detection,) = detector.detect(FRAME, target())

    np.testing.assert_array_equal(detection.corners.reshape(4, 2), observed.corners)
    assert not detection.has_pose
    assert detection.rotation_vectors is None
    assert detection.translation_vectors is None
    assert detection.reprojection_error_px is None


def test_bridge_is_constructed_with_the_same_parameters_as_the_webcam_example(monkeypatch):
    factory = Mock()
    monkeypatch.setitem(sys.modules, "faruco_bridge", SimpleNamespace(Detector=factory))
    calib = calibration()

    FArUcoDetector(target(), calib)

    factory.assert_called_once_with(
        fractal_type="FRACTAL_5L_6",
        marker_size_m=1.792,
        camera_matrix=calib.camera_matrix,
        dist_coeffs=calib.dist_coeffs,
    )


def test_missing_optional_dependency_has_an_installation_message(monkeypatch):
    monkeypatch.setitem(sys.modules, "faruco_bridge", None)
    with pytest.raises(RuntimeError, match="poetry install --with faruco"):
        FArUcoDetector(target(), calibration())


def test_close_is_forwarded_to_bridge_worker():
    detector, bridge = detector_for([])
    detector.close()
    bridge.close.assert_called_once_with()


def test_unknown_root_id_is_rejected():
    with pytest.raises(ValueError, match="automatic tracking"):
        FArUcoDetector(TargetedMarker(7, 1.792, marker_type="faruco", fractal_type="FRACTAL_5L_6"), calibration())


def test_changing_target_requires_a_new_detector():
    detector, bridge = detector_for([])
    with pytest.raises(ValueError, match="changing the target"):
        detector.detect(FRAME, target("FRACTAL_3L_6"))
    bridge.detect.assert_not_called()
