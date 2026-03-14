from abc import ABC, abstractmethod

import numpy as np


class FrameProcessor(ABC):
    @abstractmethod
    def apply(self, frame: np.ndarray) -> tuple[np.ndarray, dict]:
        raise NotImplementedError()
