from abc import ABC, abstractmethod
from typing import List

import numpy as np

from domain.models import TargetedMarker


class MarkerDetector(ABC):
    @abstractmethod
    def detect(self, frame: np.ndarray, target: TargetedMarker) -> List[tuple[int, np.ndarray]]:
        """
        Returns list of (marker_id, corners).
        corners typically shape: (1,4,2) float32
        """
