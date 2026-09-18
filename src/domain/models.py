import math
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple

import numpy as np

from domain.errors import InvalidCalibrationError, InvalidMarkerLengthError, InvalidPoseError


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

    def to_dict(self) -> dict[str, float | None]:
        return {"x": self.x, "y": self.y, "z": self.z}

    def to_angle(self) -> Tuple[float, float]:
        angle_x = math.atan2(self.y, self.z)
        angle_y = math.atan2(self.x, self.z)
        return angle_x, angle_y


@dataclass(frozen=True, slots=True)
class CalibrationData:
    """Camera intrinsics + distortion."""

    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    camera_width: Optional[int] = 640
    camera_height: Optional[int] = 480

    def __post_init__(self) -> None:
        k = np.asarray(self.camera_matrix)
        d = np.asarray(self.dist_coeffs)

        if k.shape != (3, 3):
            raise InvalidCalibrationError(f"camera_matrix must be shape (3,3), got {k.shape}")
        if k.dtype.kind not in ("f", "i"):
            raise InvalidCalibrationError(f"camera_matrix must be numeric, got {k.dtype}")

        if d.ndim == 2 and d.shape[0] == 1:
            pass  # ok: (1,N)
        elif d.ndim == 1:
            pass  # ok: (N,)
        else:
            raise InvalidCalibrationError(f"dist_coeffs must be shape (N,) or (1,N), got {d.shape}")

        # Basic sanity checks
        if not np.all(np.isfinite(k)):
            raise InvalidCalibrationError("camera_matrix contains non-finite values")
        if not np.all(np.isfinite(d)):
            raise InvalidCalibrationError("dist_coeffs contains non-finite values")


@dataclass(frozen=True, slots=True)
class TargetedMarker:
    """Defines which marker/target we want to track and its real size."""

    id: Optional[int]  # None => accept any marker (first detected)
    length: float  # side length in meters
    dictionary: int

    def __post_init__(self) -> None:
        if self.id is not None and self.id < 0:
            raise ValueError("id must be >= 0 or None")

        if not np.isfinite(self.length) or self.length <= 0.0:
            raise InvalidMarkerLengthError(f"marker_length_m must be > 0, got {self.length}")

        if not isinstance(self.dictionary, int) or isinstance(self.dictionary, bool):
            raise ValueError("dictionary must be an integer")

        if not 0 <= self.dictionary <= 16:
            raise ValueError("dictionary must be between 0 and 16")

        capacity = 1024 if self.dictionary == 16 else (50, 100, 250, 1000)[self.dictionary % 4]
        if self.id is not None and (isinstance(self.id, bool) or not isinstance(self.id, int) or self.id >= capacity):
            raise ValueError(f"id must be an integer below {capacity} for dictionary {self.dictionary}")


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
