# domain/models.py

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import numpy as np

from .errors import InvalidCalibrationError, InvalidMarkerLengthError, InvalidPoseError


@dataclass(frozen=True, slots=True)
class Pose3D:
    """3D translation (in meters) expressed in the camera coordinate system."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        for name, v in (("x", self.x), ("y", self.y), ("z", self.z)):
            if not np.isfinite(v):
                raise InvalidPoseError(f"Pose3D.{name} must be finite, got {v}")


@dataclass(frozen=True, slots=True)
class CalibrationData:
    """Camera intrinsics + distortion."""

    camera_matrix: np.ndarray  # expected (3, 3)
    dist_coeffs: np.ndarray  # common shapes: (1,5) (1,8) (5,) (8,) ...

    def __post_init__(self) -> None:
        k = np.asarray(self.camera_matrix)
        d = np.asarray(self.dist_coeffs)

        if k.shape != (3, 3):
            raise InvalidCalibrationError(
                f"camera_matrix must be shape (3,3), got {k.shape}"
            )
        if k.dtype.kind not in ("f", "i"):
            raise InvalidCalibrationError(
                f"camera_matrix must be numeric, got {k.dtype}"
            )

        if d.ndim == 2 and d.shape[0] == 1:
            pass  # ok: (1,N)
        elif d.ndim == 1:
            pass  # ok: (N,)
        else:
            raise InvalidCalibrationError(
                f"dist_coeffs must be shape (N,) or (1,N), got {d.shape}"
            )

        # Basic sanity checks
        if not np.all(np.isfinite(k)):
            raise InvalidCalibrationError("camera_matrix contains non-finite values")
        if not np.all(np.isfinite(d)):
            raise InvalidCalibrationError("dist_coeffs contains non-finite values")


@dataclass(frozen=True, slots=True)
class LandingTarget:
    """Defines which marker/target we want to track and its real size."""

    marker_id: Optional[int]  # None => accept any marker (first detected)
    marker_length_m: float  # side length in meters

    def __post_init__(self) -> None:
        if self.marker_id is not None and self.marker_id < 0:
            raise ValueError("marker_id must be >= 0 or None")

        if not np.isfinite(self.marker_length_m) or self.marker_length_m <= 0.0:
            raise InvalidMarkerLengthError(
                f"marker_length_m must be > 0, got {self.marker_length_m}"
            )


@dataclass(frozen=True, slots=True)
class CalibrationReport:
    calibration_date: datetime
    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    camera_distortion_matrix: np.ndarray
    avg_reprojection_error: float
    aspect_ratio: Optional[float]

    def get_camera_matrix(self) -> np.ndarray:
        return self.camera_matrix

    def get_camera_distortion_matrix(self) -> np.ndarray:
        return self.camera_distortion_matrix
