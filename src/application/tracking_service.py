from dataclasses import dataclass

import numpy as np

from domain.camera import Camera
from domain.marker_detector import MarkerDetector
from domain.models import CalibrationData, Pose3D, TargetedMarker
from domain.tracking import TrackingResult, TrackingStatus
from infrastructure.vision.pose_estimator_port import PoseEstimatorPort


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
    pose_estimator: PoseEstimatorPort
    target: TargetedMarker
    calibration: CalibrationData

    def track_once(self) -> TrackingResult:
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
            return TrackingResult.not_found()

        marker_id, corners = detections[0]
        pose = self.pose_estimator.estimate_pose(
            corners=corners,
            marker_length_m=self.target.marker_length_m,
            calib=self.calibration,
        )
        return TrackingResult.detected(pose=pose, marker_id=marker_id)

    def track_with_frame(self) -> tuple[np.ndarray, TrackingResult]:
        """
        Same as track_once, but also returns the frame for UI/overlay downstream.
        (Keeps the service usable for debug/preview without mixing UI logic here.)
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

    def track_specific_id(self, marker_id: int) -> TrackingResult:
        """
        Convenience method when you want to override the configured target marker_id.
        """
        frame = self.camera.get_frame()

        overridden_target = TargetedMarker(marker_id=marker_id, marker_length_m=self.target.marker_length_m)
        detections = self.detector.detect(frame, overridden_target)
        if not detections:
            return TrackingResult.not_found()

        mid, corners = detections[0]
        pose = self.pose_estimator.estimate_pose(
            corners=corners,
            marker_length_m=overridden_target.marker_length_m,
            calib=self.calibration,
        )
        return TrackingResult.detected(pose=pose, marker_id=mid)
