from typing import Tuple, Optional

import cv2
import numpy as np

from domain.models import CalibrationData, Pose3D
from domain.pose_estimator import PoseEstimator


class OpenCVPoseEstimator(PoseEstimator):
    def estimate_pose(
        self, corners: np.ndarray, marker_length_m: float, calib: CalibrationData, center: Optional[bool] = False
    ) -> Tuple[Pose3D, np.ndarray, np.ndarray]:
        if corners.ndim == 2:
            corners_in = corners.reshape(1, 1, 4, 2)
        elif corners.ndim == 3:
            corners_in = corners.reshape(1, *corners.shape)
        else:
            corners_in = corners

        rotation_vecs, translation_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners_in, marker_length_m, calib.camera_matrix, calib.dist_coeffs
        )
        t = translation_vecs[0][0]
        x, y, z = t[0], t[1], t[2]

        if center:
            x_sum = corners_in[0][0][0][0] + corners_in[0][0][1][0] + corners_in[0][0][2][0] + corners_in[0][0][3][0]
            y_sum = corners_in[0][0][0][1] + corners_in[0][0][1][1] + corners_in[0][0][2][1] + corners_in[0][0][3][1]
            x_avg = x_sum / 4
            y_avg = y_sum / 4

            fx = calib.camera_matrix[0, 0]
            fy = calib.camera_matrix[1, 1]
            cx = calib.camera_matrix[0, 2]
            cy = calib.camera_matrix[1, 2]

            x = (x_avg - cx) * z/fx
            y = (y_avg - cy) * z/fy

        return Pose3D(x=float(x), y=float(y), z=float(z)), rotation_vecs, translation_vecs
