from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from domain.models import CalibrationData, Pose3D


class PoseEstimatorPort(ABC):
    @abstractmethod
    def estimate_pose(
        self, corners: np.ndarray, marker_length_m: float, calib: CalibrationData
    ) -> Pose3D:
        """Estimate pose from detected corners."""
