from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from vtol.domain.rigid_transform import RigidTransform


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
    camera_from_marker: Optional[RigidTransform] = field(default=None, init=False)

    def __post_init__(self) -> None:
        if (self.rotation_vectors is None) != (self.translation_vectors is None):
            raise ValueError("rotation_vectors and translation_vectors must be provided together")
        if self.translation_vectors is not None:
            object.__setattr__(
                self,
                "camera_from_marker",
                RigidTransform.from_vectors(self.rotation_vectors, self.translation_vectors),
            )

    @property
    def has_pose(self) -> bool:
        return self.camera_from_marker is not None
