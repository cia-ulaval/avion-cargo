from threading import Lock
from typing import Any, Dict, Optional, Tuple

import numpy as np

from domain.buffer import Buffer


class FrameBuffer(Buffer):
    def __init__(self) -> None:
        self.lock = Lock()
        self._frame: Optional[np.ndarray] = None
        self._metadata: Optional[Dict[str, Any]] = None

    def set_value(self, frame: np.ndarray, metadata: Optional[Dict[str, Any]] = None) -> None:
        with self.lock:
            self._frame = frame
            self._metadata = metadata

    def get_value(self) -> Tuple[Optional[np.ndarray], Optional[Dict[str, Any]]]:
        with self.lock:
            if self._frame is None:
                return None, None
            return self._frame, self._metadata
