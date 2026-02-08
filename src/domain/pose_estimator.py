
from abc import ABC, abstractmethod

import numpy as np

from domain.models import CalibrationData, Pose3D


class PoseEstimator(ABC):
    @abstractmethod
    def estimate_pose(self, corners: np.ndarray, marker_length_m: float, calib: CalibrationData) -> Pose3D:
        """Estimate pose from detected corners."""
