from abc import ABC, abstractmethod
from typing import Any, Protocol

import numpy as np

from domain.drone import DroneStatus
from domain.models import Pose3D


class Buffer(ABC):
    @abstractmethod
    def set_value(self, value: Any):
        raise NotImplementedError()

    @abstractmethod
    def get_value(self) -> Any:
        raise NotImplementedError()


class FrameBufferPort(Protocol):
    """Share the latest image and its tracking metadata."""

    def set_value(
        self, frame: np.ndarray | None, metadata: dict[str, Any] | None = None, *, timestamp: float | None = None
    ) -> None: ...

    def get_value(self) -> tuple[np.ndarray | None, dict[str, Any] | None]: ...


class PoseBufferPort(Protocol):
    """Share the latest poses in camera and vehicle coordinates."""

    def set_value(self, pose: Pose3D | None, *, timestamp: float | None = None) -> None: ...

    def get_value(self) -> Pose3D | None: ...

    def set_uav_pose_value(self, uav_pose: Pose3D | None, *, timestamp: float | None = None) -> None: ...

    def get_uav_pose_value(self) -> Pose3D | None: ...


class DroneStatusBufferPort(Protocol):
    """Share the latest drone status."""

    def set_value(self, status: DroneStatus | None) -> None: ...

    def get_value(self) -> DroneStatus | None: ...
