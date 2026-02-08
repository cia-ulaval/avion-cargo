import click
from loguru import logger

from application.camera_calibrator import CameraCalibrationParameters, CameraCalibrator
from domain.camera import Camera
from domain.models import TargetedMarker
from infrastructure.camera.opencv_capture_adapter import OpenCVCaptureAdapter
from infrastructure.vision.opencv_gridboard_calibration_engine import GridBoardCalibrationConfig, GridBoardSpec


def build_camera(*, picam: bool, cam_id: int, width: int, height: int, fps: int) -> Camera:
    if picam:
        from infrastructure.camera.picamera_adapter import PiCameraAdapter

        return PiCameraAdapter(width=width, height=height, fps=fps, rgb=False)

    return OpenCVCaptureAdapter(source=cam_id, width=width, height=height, fps=fps, rgb=False)


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.option("-w", "markers_x", required=True, type=int, help="Markers X")
@click.option("-hm", "markers_y", required=True, type=int, help="Markers Y")
@click.option("-l", "marker_length", required=True, type=float, help="Marker length (meters)")
@click.option("-s", "marker_separation", required=True, type=float, help="Marker separation (meters)")
@click.option("-d", "dictionary_id", default=16, show_default=True, type=int, help="Dictionary id (0..16)")
@click.option("--rs", is_flag=True, help="Refine strategy (during calibration)")
@click.option("--zt", is_flag=True, help="Zero tangential distortion")
@click.option("--pc", is_flag=True, help="Fix principal point")
@click.option("-a", "aspect_ratio", default=None, type=float, help="Fix aspect ratio fx/fy")
@click.option("--picam", is_flag=True, help="Use PiCamera2 (Raspberry Pi)")
@click.option("--cam-id", default=0, show_default=True, type=int, help="Webcam id")
@click.option("--width", default=640, show_default=True, type=int)
@click.option("--height", default=480, show_default=True, type=int)
@click.option("--fps", default=30, show_default=True, type=int)
@logger.catch
def main(
    markers_x,
    markers_y,
    marker_length,
    marker_separation,
    dictionary_id,
    rs,
    zt,
    pc,
    aspect_ratio,
    picam,
    cam_id,
    width,
    height,
    fps,
):
    camera = build_camera(picam=picam, cam_id=cam_id, width=width, height=height, fps=fps)

    camera_calibration_params = CameraCalibrationParameters(
        board_specifications=GridBoardSpec(
            markers_x=markers_x,
            markers_y=markers_y,
            marker_length_m=marker_length,
            marker_separation_m=marker_separation,
            dictionary_id=dictionary_id,
        ),
        board_calibration_config=GridBoardCalibrationConfig(
            refine_strategy=rs,
            fix_aspect_ratio=aspect_ratio,
            zero_tangent_dist=zt,
            fix_principal_point=pc,
        ),
        target=TargetedMarker(
            marker_id=None,
            marker_length_m=marker_length,
        ),
        dictionary_id=dictionary_id,
    )

    camera_calibrator = CameraCalibrator.create(camera, camera_calibration_params)

    logger.info("Starting calibration...")
    calibration_report, calibration_filepath = camera_calibrator.calibrate()

    logger.success(f"Calibration finished: calibration report saved to {calibration_filepath}")


if __name__ == "__main__":
    main()
