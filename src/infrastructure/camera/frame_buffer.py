import time
from copy import deepcopy
from threading import Lock
from typing import Any

import numpy as np

from domain.buffer import Buffer
from domain.tracking import TrackingResult


class FrameBuffer(Buffer):
    def __init__(self, max_age_s: float = 1.0) -> None:
        if not 0 < max_age_s < float('inf'):
            raise ValueError('max_age_s must be finite and positive')
        self.lock = Lock()
        self.max_age_s = max_age_s
        self._frame = None
        self._metadata = None
        self._timestamp = 0.0

    def set_value(self, frame: np.ndarray | None, metadata: dict[str, Any] | None = None,
                  *, timestamp: float | None = None) -> None:
        with self.lock:
            self._frame = None if frame is None else frame.copy()
            self._metadata = deepcopy(metadata)
            self._timestamp = time.monotonic() if timestamp is None else timestamp

    def get_value(self) -> tuple[np.ndarray | None, dict[str, Any] | None]:
        with self.lock:
            if self._frame is None:
                return None, None
            if not 0 <= time.monotonic() - self._timestamp < self.max_age_s:
                return None, {**TrackingResult.not_found().to_dict(), 'stale': True}
            return self._frame.copy(), deepcopy(self._metadata)
