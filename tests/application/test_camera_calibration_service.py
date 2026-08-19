from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from application.camera_calibration_service import CameraCalibrationService
from domain.models import CalibrationReport


@dataclass
class FakeFrameCollector:
    frames: list[np.ndarray]
    collect_calls: int = 0

    def collect(self) -> list[np.ndarray]:
        self.collect_calls += 1
        return self.frames


@dataclass
class FakeCalibrationEngine:
    report: CalibrationReport
    received_frames: list[np.ndarray] | None = None

    def calibrate_from_frames(self, frames: list[np.ndarray]) -> CalibrationReport:
        self.received_frames = frames
        return self.report


@dataclass
class FakeCalibrationRepository:
    saved_path: Path
    saved_reports: list[CalibrationReport] = field(default_factory=list)

    def save_report(self, report: CalibrationReport) -> Path:
        self.saved_reports.append(report)
        return self.saved_path


def test_calibration_service_collects_frames_calibrates_them_and_persists_the_report(
    sample_calibration_report: CalibrationReport,
) -> None:
    frames = [np.full((2, 2, 3), fill_value=value, dtype=np.uint8) for value in (10, 20)]
    collector = FakeFrameCollector(frames)
    engine = FakeCalibrationEngine(sample_calibration_report)
    repository = FakeCalibrationRepository(Path("calibration_results/calibration_2026-01-02_03-04-05.npz"))
    service = CameraCalibrationService(collector, engine, repository)

    report, saved_path = service.calibrate()

    assert report is sample_calibration_report
    assert saved_path == repository.saved_path
    assert collector.collect_calls == 1
    assert engine.received_frames is frames
    assert repository.saved_reports == [sample_calibration_report]
