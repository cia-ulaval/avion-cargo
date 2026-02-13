from abc import ABC, abstractmethod
from threading import Lock
from typing import Dict, Any, Optional, Tuple

import numpy as np


class Camera(ABC):
    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def get_frame(self) -> np.ndarray:
        """Return an RGB or BGR frame (document which one you choose)."""
        raise NotImplementedError()

    @abstractmethod
    def get_fps(self) -> int:
        raise NotImplementedError()


class LastestFrameBuffer:
    def __init__(self) -> None:
        self.lock = Lock()
        self._frame: Optional[np.ndarray] = None
        self._metadata: Optional[Dict[str, Any]] = None

    def set(self, frame: np.ndarray, metadata: Optional[Dict[str, Any]] = None) -> None:
        with self.lock:
            self._frame = frame
            self._metadata = metadata

    def get_copy(self) -> Tuple[Optional[np.ndarray], Optional[Dict[str, Any]]]:
        with self.lock:
            if self._frame is None:
                return None, None
            return self._frame, self._metadata
