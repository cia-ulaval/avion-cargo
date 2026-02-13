from abc import ABC, abstractmethod

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
