from typing import Tuple

import cv2
import numpy as np

from domain.models import CalibrationData, Pose3D
from domain.pose_estimator import PoseEstimator


class OpenCVPoseEstimator(PoseEstimator):
    def estimate_pose(
        self, corners: np.ndarray, marker_length_m: float, calib: CalibrationData
    ) -> Tuple[Pose3D, np.ndarray, np.ndarray]:
        if corners.ndim == 2:
            corners_in = corners.reshape(1, 1, 4, 2)
        elif corners.ndim == 3:
            corners_in = corners.reshape(1, *corners.shape)
        else:
            corners_in = corners

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners_in, marker_length_m, calib.camera_matrix, calib.dist_coeffs
        )

        t = tvecs[0][0]
        return Pose3D(x=float(t[0]), y=float(t[1]), z=float(t[2])), rvecs, tvecs
