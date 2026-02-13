from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

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
        return TrackingResult(status=TrackingStatus.NOT_FOUND)

    @staticmethod
    def detected(pose: Pose3D, marker_id: int, confidence: Optional[float] = None) -> "TrackingResult":
        return TrackingResult(
            status=TrackingStatus.DETECTED,
            pose=pose,
            marker_id=marker_id,
            confidence=confidence,
        )
