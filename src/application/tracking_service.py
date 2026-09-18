import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from domain.camera import Camera
from domain.camera_mount import CameraMount
from domain.frame_annotator import FrameAnnotator
from domain.marker_detector import MarkerDetector
from domain.models import CalibrationData, Pose3D, TargetedMarker
from domain.pose_estimator import PoseEstimator
from domain.tracking import TrackingResult


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
    annotator: FrameAnnotator
    camera_mount: CameraMount = CameraMount()

    def _to_uav_pose(self, estimated_pose: Optional[Pose3D]) -> Optional[Pose3D]:
        if estimated_pose is None:
            return None

        return self.camera_mount.to_body(estimated_pose)

    def track_target(self) -> Tuple[np.ndarray, TrackingResult]:
        """
        Capture one frame, detect markers, estimate pose for the first matching marker.
        Returns a TrackingResult (DETECTED / NOT_FOUND).

        Raises:
            - domain errors if calibration/target invalid (already validated on construction),
            - infra exceptions if camera/detector fails unexpectedly.
        """
        frame = self.camera.get_frame()
        captured_at_s = getattr(self.camera, "last_capture_time_s", None)
        if not isinstance(captured_at_s, (int, float)):
            captured_at_s = time.monotonic()

        detections = self.detector.detect(frame, self.target)
        if not detections:
            return frame, TrackingResult.not_found(captured_at_s=captured_at_s)

        marker_id, corners = detections[0]
        pose, rotation_vectors, translation_vectors = self.pose_estimator.estimate_pose(
            corners=corners, marker_length_m=self.target.length, calib=self.calibration, center=True
        )

        frame = self.annotator.annotate(
            frame,
            marker_id=marker_id,
            corners=corners,
            calibration=self.calibration,
            rotation_vectors=rotation_vectors,
            translation_vectors=translation_vectors,
        )

        return frame, TrackingResult.detected(pose=pose, marker_id=marker_id, uav_pose=self._to_uav_pose(pose), captured_at_s=captured_at_s)

    def get_target(self) -> TargetedMarker:
        return self.target
