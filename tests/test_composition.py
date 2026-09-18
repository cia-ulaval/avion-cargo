from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import composition
from domain.camera import Camera
from domain.drone import Drone
from domain.models import CalibrationData, CalibrationReport, TargetedMarker
from domain.tracking import TrackingStatus
from infrastructure.camera.opencv_capture_adapter import OpenCVCamera
from infrastructure.communication.mavlink import DroneMavlinkSerialConnector, DroneMavlinkUDPConnector
from infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader
from infrastructure.persistence.calibration_repository import CalibrationRepository
from infrastructure.persistence.configuration_models import CameraConfiguration, DroneConnectionConfiguration
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetectorConfig
from infrastructure.vision.opencv_gridboard_calibration_engine import GridBoardCalibrationConfig, GridBoardSpec
from tests.conftest import write_config
from tests.infrastructure.vision.test_opencv_gridboard_calibration_engine import make_board_frames


@pytest.mark.parametrize("dictionary_id", [0, 16])
def test_calibration_assembly_detects_calibrates_and_saves(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, dictionary_id: int
) -> None:
    frames = make_board_frames(dictionary_id)
    camera = Mock(spec=Camera)
    parameters = composition.CameraCalibrationParameters(
        board_specifications=GridBoardSpec(4, 5, 0.03, 0.01, dictionary_id),
        board_calibration_config=GridBoardCalibrationConfig(),
        target=TargetedMarker(id=None, length=0.03, dictionary=dictionary_id),
        dictionary_id=dictionary_id,
    )
    service = composition.build_camera_calibration_service(camera, parameters)
    for frame in frames:
        detected = service.frame_collector.detector.detect(frame, service.frame_collector.target)
        assert {marker_id for marker_id, _ in detected} == set(range(20))
    monkeypatch.setattr(service.frame_collector, "collect", lambda: frames)
    service.calibration_repository.default_calibration_filedir = tmp_path

    report, path = service.calibrate()

    assert path.parent == tmp_path
    loaded = CalibrationRepository().set_calibration_filepath(path).load_calibration_data()
    np.testing.assert_array_equal(loaded.camera_matrix, report.camera_matrix)
    assert (loaded.camera_width, loaded.camera_height) == (960, 720)
    assert report.avg_reprojection_error < 1
    camera.open.assert_not_called()


def test_tracking_assembly_detects_estimates_and_annotates_a_real_marker() -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(16)
    frame = np.full((400, 400, 3), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(dictionary, 29, 100)
    frame[150:250, 150:250] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    original = frame.copy()
    camera = Mock(spec=Camera)
    camera.get_frame.return_value = frame
    calibration = CalibrationData(
        camera_matrix=np.float64([[500, 0, 200], [0, 500, 200], [0, 0, 1]]),
        dist_coeffs=np.zeros(5),
    )
    service = composition.build_tracking_service(
        camera, TargetedMarker(29, 0.1, 16), OpenCVArucoDetectorConfig(dictionary_id=16), calibration
    )

    annotated, result = service.track_target()

    assert result.status == TrackingStatus.DETECTED
    assert result.marker_id == 29
    assert result.pose.z == pytest.approx(0.5, abs=0.01)
    assert np.any(annotated != original)
    camera.get_frame.assert_called_once_with()


def test_landing_assembly_shares_frame_buffer_and_preserves_configuration(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, valid_config_data, sample_calibration_report: CalibrationReport
) -> None:
    repository = CalibrationRepository()
    repository.default_calibration_filedir = tmp_path
    path = repository.save_report(sample_calibration_report)
    valid_config_data["camera"]["calibration_filepath"] = path.name
    config = AutolanderConfigurationReader(write_config(tmp_path, valid_config_data)).read()
    drone = Mock(spec=Drone)
    monkeypatch.setattr(composition, "build_drone", Mock(return_value=drone))

    service = composition.build_landing_service(config)

    camera = service.aruco_tracker.camera
    assert isinstance(camera, OpenCVCamera)
    assert (camera.width, camera.height) == (1280, 720)
    assert camera.fps == 25
    assert camera._cap is None
    assert service.content_streamer.buffer is service.frame_buffer
    assert service.content_streamer.configuration.port == 8085
    assert service.content_streamer.configuration.stream_fps == 15
    assert service._threads == {}
    drone.connect.assert_not_called()


@pytest.mark.parametrize("serial", [False, True])
def test_drone_assembly_selects_transport_without_connecting(serial: bool) -> None:
    config = DroneConnectionConfiguration(
        serial, "/dev/ttyACM0" if serial else "127.0.0.1", None if serial else 14550, 57600
    )

    drone = composition.build_drone(config)

    assert isinstance(drone, DroneMavlinkSerialConnector if serial else DroneMavlinkUDPConnector)
    assert drone.connection is None
    assert drone.parameters.address == config.address
    assert drone.parameters.port == config.port
    assert drone.parameters.baud_rate == 57600


@pytest.mark.parametrize("simulation", [False, True])
def test_camera_assembly_selects_picamera_or_gazebo_without_importing_hardware(
    monkeypatch: pytest.MonkeyPatch, simulation: bool
) -> None:
    import sys
    from types import ModuleType

    picamera_module = ModuleType("infrastructure.camera.picamera_adapter")
    picamera_module.PiCameraAdapter = Mock()
    gazebo_module = ModuleType("simulation.gazebo_camera")
    gazebo_module.GazeboCamera = Mock()
    monkeypatch.setitem(sys.modules, picamera_module.__name__, picamera_module)
    monkeypatch.setitem(sys.modules, gazebo_module.__name__, gazebo_module)
    config = CameraConfiguration(0, True, 25, width=800, height=600, simulation_topic_name="/camera/image")

    camera = composition.build_camera(config, use_simulated_cam=simulation)

    if simulation:
        gazebo_module.GazeboCamera.assert_called_once_with("/camera/image")
        picamera_module.PiCameraAdapter.assert_not_called()
        assert camera is gazebo_module.GazeboCamera.return_value
    else:
        picamera_module.PiCameraAdapter.assert_called_once_with(width=800, height=600, fps=25)
        gazebo_module.GazeboCamera.assert_not_called()
        assert camera is picamera_module.PiCameraAdapter.return_value
