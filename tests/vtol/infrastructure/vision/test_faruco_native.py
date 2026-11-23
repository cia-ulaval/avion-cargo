from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from vtol.application.autolanding.tracking_service import TrackingService
from vtol.application.autolanding.yaw_alignment_supervisor import AlignmentState, YawAlignmentSupervisor
from vtol.domain.drone import Drone, DroneMode
from vtol.domain.models import CalibrationData, TargetedMarker
from vtol.domain.yaw_alignment import YawAlignmentConfig
from vtol.infrastructure.vision.faruco_detector import FArUcoDetector


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
def test_repository_faruco_texture_supplies_heading_through_tracking_to_yaw_control(
    rendered_size, level, angle, expected_error, monkeypatch, sample_drone_status
):
    pytest.importorskip("faruco_bridge")
    texture_path = Path(__file__).parents[3] / "simulation/worlds/markers/faruco.png"
    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    frame = view(texture, rendered_size)
    frame = cv2.warpAffine(
        frame, cv2.getRotationMatrix2D((320, 240), angle, 1), (640, 480), borderValue=(255, 255, 255)
    )
    calib = calibration()
    # Vary the virtual focal length to see each fractal level from the same
    # nominal height, above the supervisor's minimum height for rotation.
    calib.camera_matrix[0, 0] = calib.camera_matrix[1, 1] = rendered_size * 5 / (2.4384 * 15 / 13)
    target = TargetedMarker(None, 2.4384, marker_type="faruco", fractal_type="FRACTAL_5L_6")
    detector = FArUcoDetector(target, calib)
    camera = Mock()
    camera.get_frame.return_value = frame
    tracker = TrackingService(camera, detector, Mock(), target, calib, detector)
    drone = Mock(spec=Drone)
    drone.prepare_yaw_control.return_value = True
    drone.request_position_hold.return_value = True
    drone.position_hold_accepted.return_value = True
    supervisor = YawAlignmentSupervisor(drone, YawAlignmentConfig(enabled=True))
    clock = [100.0]
    monkeypatch.setattr("time.monotonic", lambda: clock[0])
    try:
        for index in range(55):
            now = clock[0] = 100 + index * 0.04
            camera.last_capture_time_s = now
            _, tracking = tracker.track_target()
            assert tracking.marker_id == level
            assert tracking.target_identity == "faruco:FRACTAL_5L_6"
            status = replace(
                sample_drone_status(),
                mode=DroneMode.QLAND if drone.request_mode.call_count == 0 else DroneMode.GUIDED,
                armed=True,
                speed=0,
                groundspeed_mps=0,
                roll_rad=0,
                pitch_rad=0,
                yaw_rad=0,
                attitude_timestamp_s=now,
                position_timestamp_s=now,
                last_heartbeat_monotonic_s=now,
            )
            supervisor.step(status, tracking, now)
            if supervisor.state is AlignmentState.ALIGNING:
                break
        assert supervisor.state is AlignmentState.ALIGNING
        assert supervisor.error_deg == pytest.approx(expected_error, abs=1)
        assert drone.send_yaw_input.call_args.args[0] * expected_error > 0
        drone.request_mode.assert_called_once_with(DroneMode.GUIDED, expected_modes=(DroneMode.QLAND,))
    finally:
        detector.close()
