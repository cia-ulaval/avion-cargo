from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True, slots=True)
class MarkerDetection:
    """A detected marker, optionally including a pose estimated by its detector."""

    marker_id: int
    corners: np.ndarray
    rotation_vectors: Optional[np.ndarray] = None
    translation_vectors: Optional[np.ndarray] = None
    reprojection_error_px: Optional[float] = None

    def __post_init__(self) -> None:
        if (self.rotation_vectors is None) != (self.translation_vectors is None):
            raise ValueError("rotation_vectors and translation_vectors must be provided together")
