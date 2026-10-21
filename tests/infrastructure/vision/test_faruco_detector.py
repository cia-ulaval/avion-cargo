from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from domain.models import CalibrationData, TargetedMarker
from infrastructure.vision.faruco_detector import FArUcoDetector, FArUcoDetectorConfig


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
ROTATION = np.array([np.pi, 0.08, 0.02])


def square(side):
    h = side / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])


def project(points, translation):
    return cv2.projectPoints(
        np.asarray(points, dtype=float),
        ROTATION,
        np.asarray(translation, dtype=float),
        calibration().camera_matrix,
        calibration().dist_coeffs,
    )[0].reshape(-1, 2)


def marker(level=0, ratio=1.0, translation=(0.08, -0.04, 3.0), *, old_fallback=False, points=True):
    corners = project(square(1.792 * ratio), translation)
    return SimpleNamespace(
        id=level,
        type="FRACTAL_5L_6",
        corners=corners,
        object_points=(square(1.792 if old_fallback else 1.792 * ratio) if points else None),
        image_points=(corners.copy() if points else None),
        rvec=ROTATION.copy(),
        tvec=np.asarray(translation) / ratio,
        reprojection_error_px=0.0,
    )


def detector_for(markers, *, frames=1, configured_target=None):
    bridge = Mock()
    bridge.detect.return_value = SimpleNamespace(markers=markers)
    detector = FArUcoDetector(
        configured_target or target(),
        calibration(),
        bridge_detector=bridge,
        config=FArUcoDetectorConfig(transition_frames=frames),
    )
    return detector, bridge


LAYOUTS = [
    ("FRACTAL_2L_6", (1, 1 / 3)),
    ("FRACTAL_3L_6", (1, 3 / 7, 1 / 7)),
    ("FRACTAL_4L_6", (1, 7 / 15, 1 / 5, 1 / 15)),
    ("FRACTAL_5L_6", (1, 5 / 13, 7 / 39, 1 / 13, 1 / 39)),
]


@pytest.mark.parametrize("preset,level,ratio", [(p, i, r) for p, ratios in LAYOUTS for i, r in enumerate(ratios)])
@pytest.mark.parametrize("old_fallback", [False, True])
def test_all_levels_preserve_the_global_metric_centre(preset, level, ratio, old_fallback):
    translation = np.array([0.04, -0.02, 4.0]) * (1.792 * ratio)
    observed = marker(level, ratio, translation, old_fallback=old_fallback)
    observed.type = preset
    detector, bridge = detector_for([observed], configured_target=target(preset))

    (detection,) = detector.detect(FRAME, target(preset))

    assert detection.marker_id == level
    assert detection.corners.shape == (1, 4, 2)
    assert detection.rotation_vectors.shape == detection.translation_vectors.shape == (1, 1, 3)
    np.testing.assert_allclose(detection.translation_vectors.reshape(3), translation, atol=1e-6)
    assert detection.reprojection_error_px < 1e-5
    bridge.detect.assert_called_once_with(FRAME, include_points=True)


def test_missing_correspondences_use_the_visible_levels_size_instead_of_the_bridge_pose():
    detector, _ = detector_for([marker(1, 5 / 13, points=False)])
    (detection,) = detector.detect(FRAME, target())
    np.testing.assert_allclose(detection.translation_vectors.reshape(3), [0.08, -0.04, 3.0], atol=1e-6)


@pytest.mark.parametrize("outlier", [False, True])
def test_multipoint_pose_preserves_global_coordinates_and_rejects_outliers(outlier):
    observed = marker(1, 5 / 13)
    observed.object_points = np.vstack([square(0.8), square(0.5), square(0.3)])
    observed.image_points = project(observed.object_points, [0.08, -0.04, 3.0])
    if outlier:
        observed.image_points[-1] += [40, -30]
    detector, _ = detector_for([observed])
    (detection,) = detector.detect(FRAME, target())
    np.testing.assert_allclose(detection.translation_vectors.reshape(3), [0.08, -0.04, 3.0], atol=1e-5)


def test_unknown_level_and_other_preset_are_ignored():
    other = marker()
    other.type = "FRACTAL_3L_6"
    detector, _ = detector_for([marker(9), other])
    assert detector.detect(FRAME, target()) == []


@pytest.mark.parametrize("case", ["tiny", "clipped", "nan", "collinear", "missing_points", "bad_points", "residual"])
def test_unusable_geometry_is_not_reported_as_a_detection(case):
    observed = marker()
    if case == "tiny":
        observed = marker(4, 1 / 39)
    elif case == "clipped":
        observed = marker(translation=(2.0, 0.0, 3.0))
    elif case == "nan":
        observed.corners[0, 0] = np.nan
    elif case == "collinear":
        observed.object_points[:, 1:] = 0
    elif case == "missing_points":
        observed.image_points = None
    elif case == "bad_points":
        observed.object_points[0, 0] = np.inf
    elif case == "residual":
        observed.image_points[0] += [15, -20]
    detector, _ = detector_for([observed])
    assert detector.detect(FRAME, target()) == []


def test_solver_failure_does_not_expose_an_unsafe_outer_size_fallback(monkeypatch):
    detector, _ = detector_for([marker(1, 5 / 13)])
    monkeypatch.setattr(cv2, "solvePnP", lambda *args, **kwargs: (False, None, None))
    assert detector.detect(FRAME, target()) == []


def test_acquisition_and_level_changes_require_three_frames_and_can_skip_levels():
    detector, bridge = detector_for([marker()], frames=3)
    assert detector.detect(FRAME, target()) == []
    assert detector.detect(FRAME, target()) == []
    (outer,) = detector.detect(FRAME, target())
    bridge.detect.return_value.markers = [marker(2, 7 / 39)]
    assert detector.detect(FRAME, target()) == []
    assert detector.detect(FRAME, target()) == []
    (inner,) = detector.detect(FRAME, target())
    assert outer.marker_id == 0 and inner.marker_id == 2
    np.testing.assert_allclose(inner.translation_vectors, outer.translation_vectors, atol=1e-6)


def test_interrupted_switch_resets_confirmation_and_never_replays_old_poses():
    detector, bridge = detector_for([marker()], frames=2)
    detector.detect(FRAME, target())
    assert detector.detect(FRAME, target())
    bridge.detect.return_value.markers = [marker(1, 5 / 13)]
    assert detector.detect(FRAME, target()) == []
    bridge.detect.return_value.markers = []
    assert detector.detect(FRAME, target()) == []
    bridge.detect.return_value.markers = [marker(1, 5 / 13)]
    assert detector.detect(FRAME, target()) == []
    assert detector.detect(FRAME, target())[0].marker_id == 1


def test_current_observation_continues_while_a_better_level_is_confirmed():
    detector, bridge = detector_for([marker(1, 5 / 13)], frames=2)
    detector.detect(FRAME, target())
    assert detector.detect(FRAME, target())[0].marker_id == 1
    bridge.detect.return_value.markers = [marker(1, 5 / 13), marker()]
    assert detector.detect(FRAME, target())[0].marker_id == 1
    assert detector.detect(FRAME, target())[0].marker_id == 0


def test_position_jump_is_rejected_then_reacquisition_is_possible_after_loss():
    detector, bridge = detector_for([marker()], frames=2)
    detector.detect(FRAME, target())
    assert detector.detect(FRAME, target())
    bridge.detect.return_value.markers = [marker(1, 5 / 13, translation=(0.08, -0.04, 1.5))]
    assert detector.detect(FRAME, target()) == []
    bridge.detect.return_value.markers = [marker()]
    assert detector.detect(FRAME, target())
    bridge.detect.return_value.markers = []
    for _ in range(5):
        assert detector.detect(FRAME, target()) == []
    bridge.detect.return_value.markers = [marker(1, 5 / 13, translation=(0.08, -0.04, 1.5))]
    assert detector.detect(FRAME, target()) == []
    (detection,) = detector.detect(FRAME, target())
    np.testing.assert_allclose(detection.translation_vectors.reshape(3), [0.08, -0.04, 1.5], atol=1e-6)


def test_close_is_forwarded_to_bridge_worker():
    detector, bridge = detector_for([])
    detector.close()
    bridge.close.assert_called_once_with()


def test_unknown_root_id_is_rejected():
    with pytest.raises(ValueError, match="automatic tracking"):
        FArUcoDetector(TargetedMarker(7, 1.792, marker_type="faruco", fractal_type="FRACTAL_5L_6"), calibration())
