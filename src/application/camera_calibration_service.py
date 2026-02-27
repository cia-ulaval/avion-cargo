from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

from domain.camera import Camera
from domain.camera_calibration_engine import CameraCalibrationEngine
from domain.frame_collector import FrameCollector
from domain.models import CalibrationReport, TargetedMarker
from infrastructure.persistence.calibration_repo import CalibrationRepository
from infrastructure.vision.live_frame_collector import LiveFrameCollector, LiveFrameCollectorConfig
from infrastructure.vision.opencv_aruco_detector import (
    OpenCVArucoDetector,
    OpenCVArucoDetectorConfig,
)
from infrastructure.vision.opencv_gridboard_calibration_engine import (
    GridBoardCalibrationConfig,
    GridBoardSpec,
    OpenCVGridBoardCameraCalibrationEngine,
)


@dataclass(frozen=True)
class CameraCalibrationParameters:
    board_specifications: GridBoardSpec
    board_calibration_config: GridBoardCalibrationConfig
    target: TargetedMarker
    dictionary_id: int


class CameraCalibrator:
    def __init__(
        self,
        frame_collector: FrameCollector,
        camera_calibration_engine: CameraCalibrationEngine,
        calibration_repository: CalibrationRepository,
    ):
        self.frame_collector = frame_collector
        self.calibration_engine = camera_calibration_engine
        self.calibration_repository = calibration_repository

    def calibrate(self) -> Tuple[CalibrationReport, Path]:
        collected_frames = self.frame_collector.collect()
        calibration_report = self.calibration_engine.calibrate_from_frames(collected_frames)
        saved_report_filepath = self.calibration_repository.save_report(calibration_report)
        return calibration_report, saved_report_filepath

    @staticmethod
    def create(camera: Camera, calibration_params: CameraCalibrationParameters) -> "CameraCalibrator":
        detector_config = OpenCVArucoDetectorConfig(
            dictionary_id=calibration_params.dictionary_id, corner_refinement=True
        )

        marker_detector = OpenCVArucoDetector(detector_config)

        live_frame_collector_config = LiveFrameCollectorConfig(
            window_name="Camera Calibration LiveCapture",
        )

        live_frame_collector = LiveFrameCollector(
            camera=camera,
            detector=marker_detector,
            target=calibration_params.target,
            cfg=live_frame_collector_config,
        )

        calibration_engine = OpenCVGridBoardCameraCalibrationEngine(
            calibration_params.board_specifications, calibration_params.board_calibration_config
        )

        calibration_repository = CalibrationRepository()

        return CameraCalibrator(live_frame_collector, calibration_engine, calibration_repository)
