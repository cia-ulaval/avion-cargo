import math
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from pymavlink.quaternion import Quaternion

from application.tracking_service import TrackingService
from domain.models import CalibrationData, TargetedMarker
from infrastructure.communication.mavlink import DroneMavlinkUDPConnector, MavlinkConnectionParams
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


@pytest.mark.parametrize("rendered_size,level", [(360, 0), (720, 1), (1440, 2), (4320, 3), (11520, 4)])
@pytest.mark.parametrize("angle,expected_error", [(30, 60), (150, -60)])
def test_repository_faruco_texture_publishes_oriented_landing_target(rendered_size, level, angle, expected_error):
    pytest.importorskip("faruco_bridge")
    texture_path = Path(__file__).parents[3] / "simulation/worlds/markers/faruco.png"
    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    frame = view(texture, rendered_size)
    frame = cv2.warpAffine(
        frame, cv2.getRotationMatrix2D((320, 240), angle, 1), (640, 480), borderValue=(255, 255, 255)
    )
    calib = calibration()
    # Vary the virtual focal length to see each fractal level from the same nominal height.
    calib.camera_matrix[0, 0] = calib.camera_matrix[1, 1] = rendered_size * 5 / (2.4384 * 15 / 13)
    target = TargetedMarker(None, 2.4384, marker_type="faruco", fractal_type="FRACTAL_5L_6")
    detector = FArUcoDetector(target, calib)
    camera = Mock()
    camera.get_frame.return_value = frame
    camera.last_capture_time_s = 100.0
    tracker = TrackingService(camera, detector, Mock(), target, calib, detector)
    drone = DroneMavlinkUDPConnector(MavlinkConnectionParams("127.0.0.1", 14550))
    drone.connection = mavutil.mavfile(None, "memory", source_component=191, input=False)
    drone.connection.write = Mock()
    drone.connection.close = Mock()
    drone._configure_mavlink2()
    try:
        _, tracking = tracker.track_target()
        assert tracking.marker_id == level
        assert tracking.target_identity == "faruco:FRACTAL_5L_6"
        drone.send_landing_target(
            tracking.uav_pose,
            (target.length, target.length),
            body_from_marker=tracking.body_from_marker,
            captured_at_s=tracking.captured_at_s,
        )
        drone.connection.write.assert_called_once()
        message = mavlink2.MAVLink(None).parse_char(drone.connection.write.call_args.args[0])
        assert message.get_type() == "LANDING_TARGET"
        assert message.frame == mavlink2.MAV_FRAME_BODY_FRD
        assert message.position_valid == 1
        assert message.target_num == 0
        assert message.time_usec == 100_000_000
        assert (message.x, message.y, message.z) == pytest.approx(
            (tracking.uav_pose.x, tracking.uav_pose.y, tracking.uav_pose.z)
        )
        rotation = Quaternion(message.q).dcm
        assert math.degrees(math.atan2(rotation.b.x, rotation.a.x)) == pytest.approx(expected_error, abs=1)
    finally:
        detector.close()
        drone.close()
