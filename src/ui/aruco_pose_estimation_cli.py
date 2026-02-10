from pathlib import Path

import click
from loguru import logger

from application.tracking_service import TrackingService
from domain.camera import Camera
from domain.models import TargetedMarker
from infrastructure.camera.opencv_capture_adapter import OpenCVCamera
from infrastructure.persistence.calibration_repo import CalibrationRepository
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetector, OpenCVArucoDetectorConfig
from infrastructure.vision.opencv_pose_estimator import OpenCVPoseEstimator


def build_camera(*, picam: bool, cam_id: int, width: int, height: int, fps: int) -> Camera:
    if picam:
        from infrastructure.camera.picamera_adapter import PiCameraAdapter

        return PiCameraAdapter(width=width, height=height, fps=fps, rgb=False)

    return OpenCVCamera(source=cam_id, width=width, height=height, fps=fps, rgb=False)


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("calibration_file", type=click.Path(exists=True))
@click.option("-l", "marker_length", required=True, type=float, help="Marker length (meters)")
@click.option(
    "-d", "dictionary_id", required=True, default=16, show_default=True, type=int, help="Dictionary id (0..16)"
)
@click.option("-mid", "marker_id", default=None, show_default=True, type=int, help="Marker id (0..16)")
@click.option("--picam", is_flag=True, help="Use PiCamera2 (Raspberry Pi)")
@click.option("--cam-id", default=0, show_default=True, type=int, help="Webcam id. Not necessary if using Picamera2.")
@click.option("--width", default=640, show_default=True, type=int)
@click.option("--height", default=480, show_default=True, type=int)
@click.option("--fps", default=30, show_default=True, type=int)
@logger.catch()
def main(calibration_file, marker_length, dictionary_id, marker_id, cam_id, width, height, fps, picam):

    target_marker = TargetedMarker(marker_id, marker_length)
    camera = build_camera(picam=picam, cam_id=cam_id, width=width, height=height, fps=fps)
    pose_estimator = OpenCVPoseEstimator()
    calibration_data = CalibrationRepository().load_report(Path(calibration_file))
    detection_params = OpenCVArucoDetectorConfig(dictionary_id=dictionary_id)
    detector = OpenCVArucoDetector(detection_params)
    tracker = TrackingService(
        pose_estimator=pose_estimator,
        camera=camera,
        target=target_marker,
        detector=detector,
        calibration=calibration_data,
    )

    while True:
        res = tracker.track_once()
        logger.info(f"res: {res}")


if __name__ == "__main__":
    main()
