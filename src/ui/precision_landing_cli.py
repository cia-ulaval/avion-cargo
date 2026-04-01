from pathlib import Path

import click
from loguru import logger

from application.drone_autolanding_service import DroneAutolandingService
from application.tracking_service import TrackingService
from infrastructure.communication.webrtc_content_diffuser import WebRTCConfig
from infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader
from infrastructure.persistence.calibration_repository import CalibrationRepository
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetectorConfig
from ui.common_functions import build_camera, build_drone


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("config_file_path", type=click.Path(exists=True))
@click.option(
    "--gz-simulation", default=False, is_flag=True, help="Run simulation using Gazebo Camera", show_default=True
)
@logger.catch
def main(config_file_path, gz_simulation):
    config_reader = AutolanderConfigurationReader(Path(config_file_path))
    autolander_config = config_reader.read()

    # camera and vision
    calibration_data = (
        CalibrationRepository().set_calibration_filepath(config_file_path.calibration_filepath).load_calibration_data()
    )

    camera = build_camera(
        use_simulated_cam=gz_simulation,
        camera_config=autolander_config.camera_config,
        calibration_data=calibration_data,
    )
    detector_config = OpenCVArucoDetectorConfig(dictionary_id=autolander_config.targeted_marker.dictionary)

    # drone communication
    drone = build_drone(autolander_config.drone_connection_config)
    drone.connect()

    tracker = TrackingService.create(
        camera=camera,
        target=autolander_config.targeted_marker,
        detector_config=detector_config,
        calibration_data=calibration_data,
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
    landing_service.stream_video()
    landing_service.perform_precision_landing()
    landing_service.stop()


if __name__ == "__main__":
    main()
