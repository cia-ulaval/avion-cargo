from dataclasses import asdict, dataclass
from enum import Enum, auto
from typing import Any, Optional

from .models import Pose3D


class TrackingStatus(Enum):
    DETECTED = auto()
    NOT_FOUND = auto()


@dataclass(frozen=True, slots=True)
class TrackingResult:
    status: TrackingStatus
    pose: Optional[Pose3D] = None
    marker_id: Optional[int] = None
    confidence: Optional[float] = None  # optional; can be filled later

    @staticmethod
    def not_found() -> "TrackingResult":
        return TrackingResult(status=TrackingStatus.NOT_FOUND, pose=None, marker_id=None)

    @staticmethod
    def detected(pose: Pose3D, marker_id: int, confidence: Optional[float] = None) -> "TrackingResult":
        return TrackingResult(
            status=TrackingStatus.DETECTED,
            pose=pose,
            marker_id=marker_id,
            confidence=confidence,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "pose": self.pose.to_dict() if self.pose is not None else None,
            "marker_id": self.marker_id,
        }
