from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True, slots=True)
class MarkerDetection:
    """A visible marker or fractal level, with an optional pose in the common target frame.

    For fractals, marker_id and corners describe the visible level; the metric
    pose still describes the centre of the complete physical landing target.
    """

    marker_id: int
    corners: np.ndarray
    rotation_vectors: Optional[np.ndarray] = None
    translation_vectors: Optional[np.ndarray] = None
    reprojection_error_px: Optional[float] = None

    def __post_init__(self) -> None:
        if (self.rotation_vectors is None) != (self.translation_vectors is None):
            raise ValueError("rotation_vectors and translation_vectors must be provided together")

    @property
    def has_pose(self) -> bool:
        return self.translation_vectors is not None
