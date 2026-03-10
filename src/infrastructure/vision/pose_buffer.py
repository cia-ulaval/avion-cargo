from threading import Lock
from typing import Optional


from domain.buffer import Buffer
from domain.models import Pose3D


class PoseBuffer(Buffer):
    def __init__(self) -> None:
        self.lock = Lock()
        self._pose3D: Optional[Pose3D] = None


    def set_value(self, pose: Pose3D) -> None:
        with self.lock:
            self._pose3D = pose

    def get_value(self) -> Pose3D:
        with self.lock:
            return self._pose3D