from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Optional

from .models import Pose3D
from .rigid_transform import RigidTransform


class TrackingStatus(Enum):
    DETECTED = auto()
    NOT_FOUND = auto()


@dataclass(frozen=True, slots=True)
class TrackingResult:
    status: TrackingStatus
    pose: Optional[Pose3D] = None
    uav_pose: Optional[Pose3D] = None
    marker_id: Optional[int] = None
    confidence: Optional[float] = None
    captured_at_s: Optional[float] = None  # local monotonic capture time
    camera_from_marker: Optional[RigidTransform] = None
    body_from_marker: Optional[RigidTransform] = None
    target_identity: Optional[str] = None  # Physical target; marker_id may identify a visible fractal level.

    @staticmethod
    def not_found(captured_at_s: Optional[float] = None) -> "TrackingResult":
        return TrackingResult(status=TrackingStatus.NOT_FOUND, pose=None, marker_id=None, captured_at_s=captured_at_s)

    @staticmethod
    def detected(
        pose: Pose3D,
        marker_id: int,
        uav_pose: Optional[Pose3D] = None,
        confidence: Optional[float] = None,
        captured_at_s: Optional[float] = None,
        camera_from_marker: Optional[RigidTransform] = None,
        body_from_marker: Optional[RigidTransform] = None,
        target_identity: Optional[str] = None,
    ) -> "TrackingResult":
        return TrackingResult(
            status=TrackingStatus.DETECTED,
            pose=pose,
            uav_pose=uav_pose,
            marker_id=marker_id,
            confidence=confidence,
            captured_at_s=captured_at_s,
            camera_from_marker=camera_from_marker,
            body_from_marker=body_from_marker,
            target_identity=target_identity,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "poses": {
                "estimated_pose_from_camera": self.pose.to_dict() if self.pose is not None else None,
                "estimated_pose_to_uav": self.uav_pose.to_dict() if self.uav_pose is not None else None,
            },
            "marker_id": self.marker_id,
            "target_identity": self.target_identity,
            "confidence": self.confidence,
            "transforms": {
                "camera_from_marker": self.camera_from_marker.to_list() if self.camera_from_marker else None,
                "body_from_marker": self.body_from_marker.to_list() if self.body_from_marker else None,
            },
        }
