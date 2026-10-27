import cv2
import numpy as np
import pytest

from domain.models import CalibrationData, TargetedMarker
from infrastructure.vision.faruco_detector import FArUcoDetector


@pytest.fixture(scope="module")
def native_texture(tmp_path_factory):
    bridge = pytest.importorskip("faruco_bridge")
    path = tmp_path_factory.mktemp("native-faruco") / "marker.png"
    bridge.generate_marker(fractal_type="FRACTAL_5L_6", output=path, bit_size=4)
    texture = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert texture is not None
    return texture


def view(texture, rendered_size):
    scale = rendered_size / texture.shape[1]
    transform = np.array([[scale, 0, 320 - rendered_size / 2], [0, scale, 240 - rendered_size / 2]])
    frame = cv2.warpAffine(texture, transform, (640, 480), flags=cv2.INTER_NEAREST, borderValue=255)
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def calibration():
    return CalibrationData(
        camera_matrix=np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
        camera_width=640,
        camera_height=480,
    )


@pytest.mark.parametrize("rendered_size,level", [(360, 0), (720, 1), (1440, 2), (4320, 3), (11520, 4)])
def test_native_detection_and_annotation_match_the_bridge_at_every_level(native_texture, rendered_size, level):
    from faruco_bridge import Detector

    frame = view(native_texture, rendered_size)
    original = frame.copy()
    calib = calibration()
    target = TargetedMarker(None, 1.792, marker_type="faruco", fractal_type="FRACTAL_5L_6")
    detector = FArUcoDetector(target, calib)
    try:
        with Detector(
            fractal_type=target.fractal_type.value,
            marker_size_m=target.length,
            camera_matrix=calib.camera_matrix,
            dist_coeffs=calib.dist_coeffs,
        ) as reference:
            expected = reference.detect(frame)
        (observed,) = expected.markers
        (detection,) = detector.detect(frame, target)

        assert detection.marker_id == observed.id == level
        np.testing.assert_array_equal(detection.corners.reshape(4, 2), observed.corners)
        np.testing.assert_allclose(detection.rotation_vectors.reshape(3), observed.rvec.reshape(3))
        np.testing.assert_allclose(detection.translation_vectors.reshape(3), observed.tvec.reshape(3))
        assert detection.reprojection_error_px == pytest.approx(observed.reprojection_error_px)

        annotated = detector.annotate(
            frame,
            marker_id=detection.marker_id,
            corners=detection.corners,
            calibration=calib,
            rotation_vectors=detection.rotation_vectors,
            translation_vectors=detection.translation_vectors,
        )
        np.testing.assert_array_equal(annotated, expected.annotate(frame))
        np.testing.assert_array_equal(frame, original)
        assert np.any(annotated != original)
    finally:
        detector.close()
