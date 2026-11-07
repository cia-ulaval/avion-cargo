import math
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple

import numpy as np

from domain.errors import InvalidCalibrationError, InvalidMarkerLengthError, InvalidPoseError
from domain.markers import (
    DEFAULT_ARUCO_DICTIONARY,
    SUPPORTED_FRACTAL_TYPES,
    FractalType,
    MarkerType,
    aruco_dictionary_capacity,
)


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

        if d.dtype.kind not in ("f", "i"):
            raise InvalidCalibrationError("dist_coeffs must be numeric")

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
        if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
            raise InvalidCalibrationError("camera_matrix needs positive focal lengths and a [0,0,1] final row")
        if d.size not in (4, 5, 8, 12, 14):
            raise InvalidCalibrationError("dist_coeffs must contain 4, 5, 8, 12 or 14 coefficients")
        for dimension in (self.camera_width, self.camera_height):
            if isinstance(dimension, bool) or not isinstance(dimension, (int, np.integer)) or dimension <= 0:
                raise InvalidCalibrationError("Calibration image dimensions must be positive integers")


@dataclass(frozen=True, slots=True)
class TargetedMarker:
    """Defines the marker type, identity, and physical outer side length."""

    id: Optional[int]  # None => accept any detected marker
    length: float  # outer side length in meters
    dictionary: int = DEFAULT_ARUCO_DICTIONARY  # used only for conventional ArUco markers
    marker_type: MarkerType = MarkerType.ARUCO
    fractal_type: Optional[FractalType] = None

    def __post_init__(self) -> None:
        # JSON and existing callers may pass strings. Normalise them once so
        # the rest of the application only handles validated enum members.
        try:
            object.__setattr__(self, "marker_type", MarkerType(self.marker_type))
        except ValueError as error:
            raise ValueError(f"marker_type must be one of {tuple(kind.value for kind in MarkerType)}") from error

        if self.id is not None and (isinstance(self.id, bool) or not isinstance(self.id, int) or self.id < 0):
            raise ValueError("id must be an integer >= 0 or None")

        if not np.isfinite(self.length) or self.length <= 0.0:
            raise InvalidMarkerLengthError(f"marker_length_m must be > 0, got {self.length}")

        if self.marker_type is MarkerType.ARUCO:
            if self.fractal_type is not None:
                raise ValueError("fractal_type is only valid when marker_type is 'faruco'")
            capacity = aruco_dictionary_capacity(self.dictionary)
            if self.id is not None and self.id >= capacity:
                raise ValueError(f"id must be an integer below {capacity} for dictionary {self.dictionary}")
        else:
            try:
                object.__setattr__(self, "fractal_type", FractalType(self.fractal_type))
            except ValueError as error:
                raise ValueError(f"fractal_type must be one of {SUPPORTED_FRACTAL_TYPES}") from error

    @property
    def requires_detector_pose(self) -> bool:
        """Whether visible corners alone lack enough information for the generic square PnP."""
        return self.marker_type is MarkerType.FARUCO


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
