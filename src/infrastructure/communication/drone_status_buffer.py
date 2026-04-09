from dataclasses import replace
from threading import Lock
from typing import Optional

from domain.buffer import Buffer
from domain.drone import DroneStatus


class DroneStatusBuffer(Buffer):
    def __init__(self) -> None:
        self.lock = Lock()
        self._status: Optional[DroneStatus] = None

    def set_value(self, status: Optional[DroneStatus]) -> None:
        with self.lock:
            self._status = None if status is None else replace(status)

    def get_value(self) -> Optional[DroneStatus]:
        with self.lock:
            return None if self._status is None else replace(self._status)
