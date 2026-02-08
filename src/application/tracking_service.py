from dataclasses import dataclass
from typing import Tuple

import numpy as np

from domain.camera import Camera
from domain.marker_detector import MarkerDetector
from domain.models import CalibrationData, TargetedMarker
from domain.tracking import TrackingResult
from domain.pose_estimator import PoseEstimator


@dataclass(slots=True)
class TrackingService:
    """
    Application service (use-case) that tracks an ArUco marker/target
    and estimates its pose in the camera frame.

    - No OpenCV here.
    - No PiCamera2 here.
    - Pure orchestration of ports/adapters.
    """

    camera: Camera
    detector: MarkerDetector
    pose_estimator: PoseEstimator
    target: TargetedMarker
    calibration: CalibrationData

    def track_once(self) -> Tuple[np.ndarray, TrackingResult]:
        """
        Capture one frame, detect markers, estimate pose for the first matching marker.
        Returns a TrackingResult (DETECTED / NOT_FOUND).

        Raises:
            - domain errors if calibration/target invalid (already validated on construction),
            - infra exceptions if camera/detector fails unexpectedly.
        """
        frame = self.camera.get_frame()

        detections = self.detector.detect(frame, self.target)
        if not detections:
            return frame, TrackingResult.not_found()

        marker_id, corners = detections[0]
        pose = self.pose_estimator.estimate_pose(
            corners=corners,
            marker_length_m=self.target.marker_length_m,
            calib=self.calibration,
        )
        return frame, TrackingResult.detected(pose=pose, marker_id=marker_id)
