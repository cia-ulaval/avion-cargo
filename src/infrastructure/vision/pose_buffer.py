import time
from threading import Lock

from domain.buffer import Buffer
from domain.models import Pose3D


class PoseBuffer(Buffer):
    def __init__(self, max_age_s: float = 1.0) -> None:
        if not 0 < max_age_s < float('inf'):
            raise ValueError('max_age_s must be finite and positive')
        self.lock = Lock()
        self.max_age_s = max_age_s
        self._pose3D = None
        self._uav_pose3D = None
        self._pose_timestamp = self._uav_timestamp = 0.0

    def set_value(self, pose: Pose3D | None, *, timestamp: float | None = None) -> None:
        with self.lock:
            self._pose3D = pose
            self._pose_timestamp = time.monotonic() if timestamp is None else timestamp

    def set_uav_pose_value(self, uav_pose: Pose3D | None, *, timestamp: float | None = None) -> None:
        with self.lock:
            self._uav_pose3D = uav_pose
            self._uav_timestamp = time.monotonic() if timestamp is None else timestamp

    def get_value(self) -> Pose3D | None:
        with self.lock:
            return self._pose3D if 0 <= time.monotonic() - self._pose_timestamp < self.max_age_s else None

    def get_uav_pose_value(self) -> Pose3D | None:
        with self.lock:
            return self._uav_pose3D if 0 <= time.monotonic() - self._uav_timestamp < self.max_age_s else None
