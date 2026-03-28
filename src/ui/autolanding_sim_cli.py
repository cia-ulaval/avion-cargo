from pathlib import Path

import click
from loguru import logger

from application.tracking_service import TrackingService
from domain.drone import Drone
from infrastructure.communication.webrtc_content_diffuser import WebRTCConfig
from infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader
from infrastructure.persistence.calibration_repo import CalibrationRepository
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetectorConfig
from simulation.drone_autolanding_service import DroneAutolandingService
from simulation.gazebo_camera import GazeboCamera

# from infrastructure.vision.threaded_pipeline import ThreadedPipeline
from ui.common_functions import build_camera, build_drone


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@logger.catch
def main():
    config_reader = AutolanderConfigurationReader(
        Path("/home/bertrand-awz/Documents/avionCargo/autolander/autolanding_config.json")
    )
    autolander_config = config_reader.read()

    # camera and vision
    calibration_data = CalibrationRepository().load_report(autolander_config.camera_config.calibration_filepath)
    camera = GazeboCamera(
        topic_name="/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
    )
    detector_config: OpenCVArucoDetectorConfig = OpenCVArucoDetectorConfig(
        dictionary_id=autolander_config.targeted_marker.dictionary
    )

    # drone communication
    drone: Drone = build_drone(autolander_config.drone_connection_config)
    drone.connect()

    tracker = TrackingService.create(
        target=autolander_config.targeted_marker,
        camera=camera,
        detector_config=detector_config,
        calibration=calibration_data,
    )

    # streaming
    streamer_config = WebRTCConfig(
        host="0.0.0.0",
        port=autolander_config.streaming_config.port,
        stream_fps=autolander_config.streaming_config.video.fps,
    )

    # landing operations

    landing_service = DroneAutolandingService(drone, tracker, streamer_config)
    landing_service.track_target()
    landing_service.stream_landing_video()
    landing_service.land()
    landing_service.stop()


if __name__ == "__main__":
    main()
