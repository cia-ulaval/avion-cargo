import os
from dataclasses import dataclass
from typing import Optional

from vtol.application.autolanding.camera_calibration_service import CameraCalibrationService
from vtol.application.autolanding.drone_autolanding_service import DroneAutolandingService
from vtol.application.autolanding.tracking_service import TrackingService
from vtol.application.flight_controller_runtime import FlightControllerObservationRuntime
from vtol.application.mission.observation_store import MissionObservationStore
from vtol.application.mission.orchestrator import MissionOrchestrator
from vtol.application.payload_coordination.runtime import VtolPayloadCommunicationRuntime
from vtol.application.runtime import VtolApplicationRuntime
from vtol.domain.camera import Camera
from vtol.domain.camera_mount import CameraMount
from vtol.domain.drone import Drone
from vtol.domain.markers import MarkerType
from vtol.domain.models import CalibrationData, TargetedMarker
from vtol.infrastructure.camera.frame_buffer import FrameBuffer
from vtol.infrastructure.camera.opencv_capture_adapter import OpenCVCamera
from vtol.infrastructure.communication.drone_status_buffer import DroneStatusBuffer
from vtol.infrastructure.d3kble.runtime import VtolD3kbleApplication, build_vtol_d3kble_application
from vtol.infrastructure.persistence.calibration_repository import CalibrationRepository
from vtol.infrastructure.persistence.configuration_models import (
    AutolanderConfiguration,
    CameraConfiguration,
    DroneConnectionConfiguration,
)
from vtol.infrastructure.persistence.vtol_application_configuration import VtolApplicationConfiguration
from vtol.infrastructure.vision.faruco_detector import FArUcoDetector
from vtol.infrastructure.vision.live_frame_collector import LiveFrameCollector, LiveFrameCollectorConfig
from vtol.infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetector, OpenCVArucoDetectorConfig
from vtol.infrastructure.vision.opencv_frame_annotator import OpenCVFrameAnnotator
from vtol.infrastructure.vision.opencv_gridboard_calibration_engine import (
    GridBoardCalibrationConfig,
    GridBoardSpec,
    OpenCVGridBoardCameraCalibrationEngine,
)
from vtol.infrastructure.vision.opencv_pose_estimator import OpenCVPoseEstimator
from vtol.infrastructure.vision.pose_buffer import PoseBuffer


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

        from vtol.simulation.gazebo_camera import GazeboCamera

        return GazeboCamera(camera_config.simulation_topic_name, fps=camera_config.fps)

    camera_height, camera_width = camera_config.height, camera_config.width
    if calibration_data:
        camera_height, camera_width = calibration_data.camera_height, calibration_data.camera_width

    if camera_config.use_picamera:
        from vtol.infrastructure.camera.picamera_adapter import PiCameraAdapter

        return PiCameraAdapter(width=camera_width, height=camera_height, fps=camera_config.fps)

    return OpenCVCamera(
        source=camera_config.id,
        width=camera_width,
        height=camera_height,
        fps=camera_config.fps,
    )


def build_drone(drone_connection_config: DroneConnectionConfiguration) -> Drone:
    from vtol.infrastructure.communication.mavlink import (
        DroneMavlinkSerialConnector,
        DroneMavlinkUDPConnector,
        MavlinkConnectionParams,
    )

    mavlink_params = MavlinkConnectionParams(
        address=drone_connection_config.address,
        port=drone_connection_config.port,
        baud_rate=drone_connection_config.baud_rate,
        timeout=drone_connection_config.timeout,
        fail_on_timeout=drone_connection_config.fail_on_timeout,
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
    detector_config: Optional[OpenCVArucoDetectorConfig],
    calibration_data: CalibrationData,
    camera_mount: CameraMount = CameraMount(),
) -> TrackingService:
    if target.marker_type is MarkerType.FARUCO:
        detector = FArUcoDetector(target, calibration_data)
        annotator = detector
    else:
        detector = OpenCVArucoDetector(detector_config or OpenCVArucoDetectorConfig(dictionary_id=target.dictionary))
        annotator = OpenCVFrameAnnotator()

    return TrackingService(
        camera=camera,
        detector=detector,
        pose_estimator=OpenCVPoseEstimator(),
        target=target,
        calibration=calibration_data,
        annotator=annotator,
        camera_mount=camera_mount,
    )


def build_landing_service(config: AutolanderConfiguration, use_simulated_cam: bool = False) -> DroneAutolandingService:
    """Assemble landing components; connection and startup remain explicit."""
    from vtol.infrastructure.communication.webrtc_content_streamer import WebRTCConfig, WebRTCContentStreamer

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
        detector_config=None,
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



@dataclass
class VtolApplication:
    """Own the VTOL Pi's flight-controller, D3K and mission application runtime."""

    d3kble: VtolD3kbleApplication
    drone: Drone
    observations: MissionObservationStore
    runtime: VtolApplicationRuntime
    _closed: bool = False

    @property
    def nodes(self):
        return self.d3kble.nodes

    @property
    def payload_peers(self):
        return self.d3kble.payload_peers

    async def run(self, stop_event) -> None:
        await self.runtime.run(stop_event)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.drone.close()
        finally:
            self.d3kble.close()


def build_vtol_application(
    config: VtolApplicationConfiguration,
    *,
    node_factory=None,
    drone_factory=None,
) -> VtolApplication:
    """Build the single VTOL application runtime from the combined Pi config."""
    d3kble_application = build_vtol_d3kble_application(config.d3kble, node_factory=node_factory)
    drone = None
    try:
        drone = (drone_factory or build_drone)(config.autolanding.drone_connection_config)
        observations = MissionObservationStore()
        flight_controller_runtime = FlightControllerObservationRuntime(drone, observations)
        payload_runtime = VtolPayloadCommunicationRuntime(d3kble_application.payload_peers, observations)
        mission_orchestrator = MissionOrchestrator(observations.snapshot)
        runtime = VtolApplicationRuntime(
            flight_controller=flight_controller_runtime,
            payload_communication=payload_runtime,
            mission_orchestrator=mission_orchestrator,
        )
        return VtolApplication(d3kble_application, drone, observations, runtime)
    except BaseException:
        if drone is not None:
            drone.close()
        d3kble_application.close()
        raise
