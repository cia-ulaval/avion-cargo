"""Assemble application services with their concrete adapters."""

from dataclasses import dataclass
import os
from typing import Optional

from application.camera_calibration_service import CameraCalibrationService
from application.drone_autolanding_service import DroneAutolandingService
from application.tracking_service import TrackingService
from domain.camera import Camera
from domain.camera_mount import CameraMount
from domain.drone import Drone
from domain.models import CalibrationData, TargetedMarker
from infrastructure.camera.frame_buffer import FrameBuffer
from infrastructure.camera.opencv_capture_adapter import OpenCVCamera
from infrastructure.communication.drone_status_buffer import DroneStatusBuffer
from infrastructure.persistence.calibration_repository import CalibrationRepository
from infrastructure.persistence.configuration_models import (
    AutolanderConfiguration,
    CameraConfiguration,
    DroneConnectionConfiguration,
)
from infrastructure.vision.live_frame_collector import LiveFrameCollector, LiveFrameCollectorConfig
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetector, OpenCVArucoDetectorConfig
from infrastructure.vision.opencv_frame_annotator import OpenCVFrameAnnotator
from infrastructure.vision.opencv_gridboard_calibration_engine import (
    GridBoardCalibrationConfig,
    GridBoardSpec,
    OpenCVGridBoardCameraCalibrationEngine,
)
from infrastructure.vision.opencv_pose_estimator import OpenCVPoseEstimator
from infrastructure.vision.pose_buffer import PoseBuffer


@dataclass(frozen=True)
class CameraCalibrationParameters:
    board_specifications: GridBoardSpec
    board_calibration_config: GridBoardCalibrationConfig
    target: TargetedMarker
    dictionary_id: int


def build_camera(
    camera_config: CameraConfiguration,
    calibration_data: Optional[CalibrationData] = None,
    use_simulated_cam: Optional[bool] = False,
) -> Camera:
    if use_simulated_cam:
        if not camera_config.simulation_topic_name:
            raise ValueError("The simulation topic name must be provided")

        from simulation.gazebo_camera import GazeboCamera

        return GazeboCamera(camera_config.simulation_topic_name, fps=camera_config.fps)

    camera_height, camera_width = camera_config.height, camera_config.width
    if calibration_data:
        camera_height, camera_width = calibration_data.camera_height, calibration_data.camera_width

    if camera_config.use_picamera:
        from infrastructure.camera.picamera_adapter import PiCameraAdapter

        return PiCameraAdapter(width=camera_width, height=camera_height, fps=camera_config.fps)

    return OpenCVCamera(
        source=camera_config.id,
        width=camera_width,
        height=camera_height,
        fps=camera_config.fps,
    )


def build_drone(drone_connection_config: DroneConnectionConfiguration) -> Drone:
    from infrastructure.communication.mavlink import (
        DroneMavlinkSerialConnector,
        DroneMavlinkUDPConnector,
        MavlinkConnectionParams,
    )

    mavlink_params = MavlinkConnectionParams(
        address=drone_connection_config.address,
        port=drone_connection_config.port,
        baud_rate=drone_connection_config.baud_rate,
    )
    if drone_connection_config.use_serial:
        return DroneMavlinkSerialConnector(mavlink_params)

    else:
        return DroneMavlinkUDPConnector(mavlink_params)


def build_camera_calibration_service(
    camera: Camera, calibration_params: CameraCalibrationParameters
) -> CameraCalibrationService:
    marker_detector = OpenCVArucoDetector(
        OpenCVArucoDetectorConfig(dictionary_id=calibration_params.dictionary_id, corner_refinement=True)
    )
    collector = LiveFrameCollector(
        camera=camera,
        detector=marker_detector,
        target=calibration_params.target,
        cfg=LiveFrameCollectorConfig(window_name="Camera Calibration LiveCapture"),
    )
    engine = OpenCVGridBoardCameraCalibrationEngine(
        calibration_params.board_specifications, calibration_params.board_calibration_config
    )
    return CameraCalibrationService(collector, engine, CalibrationRepository())


def build_tracking_service(
    camera: Camera,
    target: TargetedMarker,
    detector_config: OpenCVArucoDetectorConfig,
    calibration_data: CalibrationData,
    camera_mount: CameraMount = CameraMount(),
) -> TrackingService:
    return TrackingService(
        camera=camera,
        detector=OpenCVArucoDetector(detector_config),
        pose_estimator=OpenCVPoseEstimator(),
        target=target,
        calibration=calibration_data,
        annotator=OpenCVFrameAnnotator(),
        camera_mount=camera_mount,
    )


def build_landing_service(config: AutolanderConfiguration, use_simulated_cam: bool = False) -> DroneAutolandingService:
    """Assemble landing components; connection and startup remain explicit."""
    from infrastructure.communication.webrtc_content_streamer import WebRTCConfig, WebRTCContentStreamer

    calibration = (
        CalibrationRepository()
        .set_calibration_filepath(config.camera_config.calibration_filepath)
        .load_calibration_data()
    )
    camera = build_camera(config.camera_config, calibration, use_simulated_cam)
    drone = build_drone(config.drone_connection_config)
    tracker = build_tracking_service(
        camera=camera,
        target=config.targeted_marker,
        detector_config=OpenCVArucoDetectorConfig(dictionary_id=config.targeted_marker.dictionary),
        calibration_data=calibration,
        camera_mount=config.camera_config.mount,
    )
    frame_buffer = FrameBuffer()
    streamer = WebRTCContentStreamer(
        frame_buffer,
        WebRTCConfig(
            host=config.streaming_config.host,
            port=config.streaming_config.port,
            stream_fps=config.streaming_config.video.fps,
            password=os.getenv("AUTOLANDER_HTTP_PASSWORD"),
            tls_cert=os.getenv("AUTOLANDER_TLS_CERT"),
            tls_key=os.getenv("AUTOLANDER_TLS_KEY"),
        ),
    )
    return DroneAutolandingService(
        drone,
        tracker,
        content_streamer=streamer,
        frame_buffer=frame_buffer,
        pose_buffer=PoseBuffer(),
        drone_status_buffer=DroneStatusBuffer(),
        telemetry_dps=config.streaming_config.data.dps,
    )
