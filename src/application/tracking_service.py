import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from domain.camera import Camera
from domain.camera_mount import CameraMount
from domain.frame_annotator import FrameAnnotator
from domain.marker_detection import MarkerDetection
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
        expected_shape = (self.calibration.camera_height, self.calibration.camera_width)
        if frame.shape[:2] != expected_shape:
            raise ValueError(f"Camera frame {frame.shape[:2]} does not match calibration {expected_shape}")
        captured_at_s = getattr(self.camera, "last_capture_time_s", None)
        if not isinstance(captured_at_s, (int, float)):
            captured_at_s = time.monotonic()

        detections = self.detector.detect(frame, self.target)
        if not detections:
            return frame, TrackingResult.not_found(captured_at_s=captured_at_s)

        detection = detections[0]
        # Accept tuple detections from older/custom adapters during migration.
        if isinstance(detection, MarkerDetection):
            marker_id, corners = detection.marker_id, detection.corners
        else:
            marker_id, corners = detection
            detection = None

        pose = None
        rotation_vectors = None
        translation_vectors = None
        if detection is not None and detection.has_pose:
            # Use the detector's pose directly.
            rotation_vectors = detection.rotation_vectors
            translation_vectors = detection.translation_vectors
            x, y, z = np.asarray(translation_vectors).reshape(-1)[:3]
            pose = Pose3D(x=float(x), y=float(y), z=float(z))
        elif not self.target.requires_detector_pose:
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
        if pose is None or pose.z <= 0:
            return frame, TrackingResult.not_found(captured_at_s=captured_at_s)

        return frame, TrackingResult.detected(
            pose=pose, marker_id=marker_id, uav_pose=self._to_uav_pose(pose), captured_at_s=captured_at_s
        )

    def get_target(self) -> TargetedMarker:
        return self.target
