import click
from loguru import logger

from application.tracking_service import TrackingService
from domain.camera import LastestFrameBuffer
from infrastructure.communication.webrtc_content_diffuser import WebRTCConfig, WebRTCContentDiffuser
from infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader
from infrastructure.persistence.calibration_repo import CalibrationRepository
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetectorConfig
from ui.common_functions import build_camera, build_drone


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("config_file_path", type=click.Path(exists=True))
@logger.catch
def main(config_file_path):
    config_reader = AutolanderConfigurationReader(config_file_path)
    autolander_config = config_reader.read()

    # camera and vision

    frame_buffer = LastestFrameBuffer()
    calibration_data = CalibrationRepository().load_report(autolander_config.camera_config.calibration_filepath)
    camera = build_camera(
        picam=autolander_config.camera_config.use_picamera,
        cam_id=autolander_config.camera_config.id,
        width=calibration_data.camera_width,
        height=calibration_data.camera_height,
        fps=autolander_config.camera_config.fps,
    )

    detector_config = OpenCVArucoDetectorConfig(dictionary_id=autolander_config.targeted_marker.dictionary)
    tracker = TrackingService.create(
        target=autolander_config.targeted_marker,
        camera=camera,
        detector_config=detector_config,
        calibration=calibration_data,
    )

    # drone communication
    drone = build_drone(autolander_config.drone_connection_config)
    drone.connect()

    # streaming
    streamer_config = WebRTCConfig(
        host="0.0.0.0",
        port=autolander_config.streaming_config.port,
        stream_fps=autolander_config.streaming_config.video.fps,
    )
    content_streamer = WebRTCContentDiffuser(frame_buffer, streamer_config)
