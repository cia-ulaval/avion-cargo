from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from domain.models import CalibrationData


class FrameAnnotator(ABC):
    """Render a detected marker and its pose onto an image."""

    @abstractmethod
    def annotate(
        self,
        frame: np.ndarray,
        *,
        marker_id: int,
        corners: np.ndarray,
        calibration: CalibrationData,
        rotation_vectors: Optional[np.ndarray],
        translation_vectors: Optional[np.ndarray],
    ) -> np.ndarray:
        """Return the annotated image, which may be a copy of the input."""
        raise NotImplementedError()
