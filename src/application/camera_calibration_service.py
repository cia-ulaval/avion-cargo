from pathlib import Path
from typing import Tuple

from domain.calibration_report_store import CalibrationReportStore
from domain.camera_calibration_engine import CameraCalibrationEngine
from domain.frame_collector import FrameCollector
from domain.models import CalibrationReport


class CameraCalibrationService:
    def __init__(
        self,
        frame_collector: FrameCollector,
        camera_calibration_engine: CameraCalibrationEngine,
        calibration_repository: CalibrationReportStore,
    ):
        self.frame_collector = frame_collector
        self.calibration_engine = camera_calibration_engine
        self.calibration_repository = calibration_repository

    def calibrate(self) -> Tuple[CalibrationReport, Path]:
        collected_frames = self.frame_collector.collect()
        calibration_report = self.calibration_engine.calibrate_from_frames(collected_frames)
        saved_report_filepath = self.calibration_repository.save_report(calibration_report)
        return calibration_report, saved_report_filepath
