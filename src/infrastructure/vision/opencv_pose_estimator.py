from typing import Optional, Tuple

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

        # OpenCV already estimates the marker center using calibrated, undistorted rays.
        # Keep the legacy center argument for API compatibility. Averaging distorted
        # image corners here would overwrite the metric PnP solution.

        return Pose3D(x=float(x), y=float(y), z=float(z)), rotation_vecs, translation_vecs
