from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np

from vtol.domain.models import CalibrationData
from vtol.domain.rigid_transform import RigidTransform


class PoseEstimator(ABC):
    @abstractmethod
    def estimate_pose(
        self, corners: np.ndarray, marker_length_m: float, calib: CalibrationData, center: Optional[bool] = False
    ) -> Tuple[RigidTransform, np.ndarray, np.ndarray]:
        """
        Estimate pose from detected corners.
        :param corners: detected corners (numpy array)
        :param marker_length_m: length of marker detected (in meters)
        :param calib: calibration data (CalibrationData object)
        :param center: center of pose estimation
        :return: Tuple[camera_from_marker, rotation_vectors, translation_vectors]
            - Full marker-to-camera homogeneous transform, with metric translation
            - Rotation vectors
            - Translation vectors
        """
