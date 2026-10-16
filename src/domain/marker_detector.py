from abc import ABC, abstractmethod
from typing import List

import numpy as np

from domain.marker_detection import MarkerDetection
from domain.models import TargetedMarker


class MarkerDetector(ABC):
    @abstractmethod
    def detect(self, frame: np.ndarray, target: TargetedMarker) -> List[MarkerDetection]:
        """Return detections for the configured target, including outer image corners."""

    def close(self) -> None:
        """Release detector resources. Stateless detectors need not override this."""
