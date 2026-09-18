from abc import ABC, abstractmethod
from pathlib import Path

from domain.models import CalibrationReport


class CalibrationReportStore(ABC):
    """Persist a calibration report and return its location."""

    @abstractmethod
    def save_report(self, calib: CalibrationReport) -> Path:
        raise NotImplementedError()
