from abc import ABC, abstractmethod
from typing import Tuple

import numpy as np

from domain.models import CalibrationData, Pose3D


class PoseEstimator(ABC):
    @abstractmethod
    def estimate_pose(self, corners: np.ndarray, marker_length_m: float, calib: CalibrationData) -> Tuple[Pose3D, np.ndarray, np.ndarray]:
        """
        Estimate pose from detected corners.
        :param corners: detected corners (numpy array)
        :param marker_length_m: length of marker detected (in meters)
        :param calib: calibration data (CalibrationData object)
        :return: Tuple[Pose3D, rvecs, tvecs]
            - Estimated pose as Pose3D object
            - Translation vectors
            - Rotation vectors
        """
