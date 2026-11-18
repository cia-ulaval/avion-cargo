from abc import ABC, abstractmethod
from typing import Iterable

import numpy as np

from vtol.domain.models import CalibrationReport


class CameraCalibrationEngine(ABC):

    @abstractmethod
    def calibrate_from_frames(self, frames: Iterable[np.ndarray]) -> CalibrationReport:
        """Compute calibration from an iterable of frames (numpy arrays)."""
