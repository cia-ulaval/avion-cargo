import cv2
import numpy as np
import pytest

from domain.models import CalibrationData, TargetedMarker
from infrastructure.vision.faruco_detector import FArUcoDetector, FArUcoDetectorConfig


@pytest.fixture(scope="module")
def native_texture(tmp_path_factory):
    bridge = pytest.importorskip("faruco_bridge")
    path = tmp_path_factory.mktemp("native-faruco") / "marker.png"
    bridge.generate_marker(fractal_type="FRACTAL_5L_6", output=path, bit_size=4)
    texture = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert texture is not None
    return texture


def view(texture, rendered_size):
    _, x = np.nonzero(texture < 128)
    scale = rendered_size / texture.shape[1]
    expected_z = 1.792 * 600 / ((x.max() - x.min() + 1) * scale)
    transform = np.array(
        [
            [scale, 0, 320 + 600 * 0.02 / expected_z - rendered_size / 2],
            [0, scale, 240 - 600 * 0.01 / expected_z - rendered_size / 2],
        ]
    )
    frame = cv2.warpAffine(texture, transform, (640, 480), flags=cv2.INTER_NEAREST, borderValue=255)
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR), expected_z


def make_detector(config=None):
    calibration = CalibrationData(
        camera_matrix=np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
        camera_width=640,
        camera_height=480,
    )
    target = TargetedMarker(None, 1.792, marker_type="faruco", fractal_type="FRACTAL_5L_6")
    return FArUcoDetector(target, calibration, config=config), target


@pytest.mark.parametrize("rendered_size,level", [(360, 0), (720, 1), (1440, 2), (2880, 3), (11520, 4)])
def test_native_inner_only_views_preserve_distance_and_lateral_offset(native_texture, rendered_size, level):
    frame, expected_z = view(native_texture, rendered_size)
    detector, target = make_detector(FArUcoDetectorConfig(transition_frames=1))
    try:
        (detection,) = detector.detect(frame, target)
        assert detection.marker_id == level
        np.testing.assert_allclose(detection.translation_vectors.reshape(3), [0.02, -0.01, expected_z], atol=0.004)
        assert detection.reprojection_error_px < 3
    finally:
        detector.close()


def test_native_approach_and_retreat_keep_a_common_pose_through_all_five_levels(native_texture):
    detector, target = make_detector()
    sizes = np.r_[np.geomspace(360, 11520, 80), np.repeat(11520, 3), np.geomspace(11520, 360, 80), np.repeat(360, 3)]
    levels = set()
    try:
        for size in sizes:
            frame, expected_z = view(native_texture, size)
            detections = detector.detect(frame, target)
            if not detections:
                continue
            (detection,) = detections
            levels.add(detection.marker_id)
            x, y, z = detection.translation_vectors.reshape(3)
            assert abs(x - 0.02) < 0.006 and abs(y + 0.01) < 0.006
            assert z == pytest.approx(expected_z, rel=0.03)
        assert levels == {0, 1, 2, 3, 4}
        assert detections[0].marker_id == 0
    finally:
        detector.close()
