import cv2
import numpy as np

from domain.models import CalibrationData
from infrastructure.vision.opencv_pose_estimator import OpenCVPoseEstimator


def test_off_axis_tilted_distorted_marker_preserves_pnp_translation():
    calibration = CalibrationData(
        camera_matrix=np.float64([[600, 0, 320], [0, 610, 240], [0, 0, 1]]),
        dist_coeffs=np.float64([0.25, -0.08, 0.02, -0.01, 0]),
    )
    square = np.float64([[-.2, .2, 0], [.2, .2, 0], [.2, -.2, 0], [-.2, -.2, 0]])
    translation = np.float64([0.7, 0.3, 1.6])
    corners, _ = cv2.projectPoints(square, np.float64([2.8, .3, .1]), translation,
                                  calibration.camera_matrix, calibration.dist_coeffs)
    pose, _, tvec = OpenCVPoseEstimator().estimate_pose(corners.reshape(1, 4, 2), .4, calibration, center=True)
    np.testing.assert_allclose([pose.x, pose.y, pose.z], translation, atol=1e-5)
    np.testing.assert_allclose([pose.x, pose.y, pose.z], tvec[0, 0])
