from abc import ABC, abstractmethod
from typing import List

import numpy as np

from domain.models import LandingTarget


class MarkerDetector(ABC):
    @abstractmethod
    def detect(
        self, frame: np.ndarray, target: LandingTarget
    ) -> List[tuple[int, np.ndarray]]:
        """
        Returns list of (marker_id, corners).
        corners typically shape: (1,4,2) float32
        """
