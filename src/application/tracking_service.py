from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from domain.camera import Camera
from domain.marker_detector import MarkerDetector
from domain.models import CalibrationData, Pose3D, TargetedMarker
from domain.pose_estimator import PoseEstimator
from domain.tracking import TrackingResult
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetector, OpenCVArucoDetectorConfig
from infrastructure.vision.opencv_frame_manipution_tool import FrameManipulationTool
from infrastructure.vision.opencv_pose_estimator import OpenCVPoseEstimator


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

    @staticmethod
    def _to_uav_pose(estimated_pose: Optional[Pose3D]) -> Optional[Pose3D]:
        if estimated_pose is None:
            return None

        return Pose3D(
            x=-estimated_pose.y,
            y=estimated_pose.x,
            z=estimated_pose.z,
        )

    def track_target(self) -> Tuple[np.ndarray, TrackingResult]:
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
        pose, rotation_vectors, translation_vectors = self.pose_estimator.estimate_pose(
            corners=corners, marker_length_m=self.target.length, calib=self.calibration, center=True
        )

        FrameManipulationTool.draw_detected_markers(frame, [corners], np.array([[marker_id]], dtype=np.int32))
        FrameManipulationTool.draw_axes_for_poses(
            frame, self.calibration.camera_matrix, self.calibration.dist_coeffs, rotation_vectors, translation_vectors
        )

        return frame, TrackingResult.detected(pose=pose, marker_id=marker_id, uav_pose=self._to_uav_pose(pose))

    def get_target(self) -> TargetedMarker:
        return self.target

    @staticmethod
    def create(
        camera: Camera,
        target: TargetedMarker,
        detector_config: OpenCVArucoDetectorConfig,
        calibration_data: CalibrationData,
    ) -> "TrackingService":
        detector = OpenCVArucoDetector(detector_config)
        pose_estimator = OpenCVPoseEstimator()

        return TrackingService(
            camera=camera,
            detector=detector,
            pose_estimator=pose_estimator,
            target=target,
            calibration=calibration_data,
        )
