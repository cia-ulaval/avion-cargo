import math
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Optional, Tuple

import numpy as np
from tabulate import tabulate

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

    camera_matrix: np.ndarray  # expected (3, 3)
    dist_coeffs: np.ndarray  # common shapes: (1,5) (1,8) (5,) (8,) ...
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

        if not self.dictionary >= 1 and self.dictionary <= 16:
            raise ValueError("dictionary must be between 1 and 16")


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

    @staticmethod
    def _format_matrix(
        m: np.ndarray,
        float_fmt: str = ".6f",
        table_fmt: str = "rounded_outline",
    ) -> str:
        arr = np.asarray(m)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)

        rows: list[list[Any]] = []
        for r in arr:
            rows.append([float(x) if np.isfinite(x) else x for x in r])

        return tabulate(
            rows,
            tablefmt=table_fmt,
            floatfmt=float_fmt,
            colalign=("right",) * (arr.shape[1] if arr.ndim == 2 else 1),
        )

    @staticmethod
    def _fmt_float(v: Any, ndigits: int = 6) -> str:
        if v is None:
            return "—"
        try:
            fv = float(v)
        except ValueError:
            return str(v)
        if not np.isfinite(fv):
            return str(v)
        return f"{fv:.{ndigits}f}"

    @staticmethod
    def _fmt_int(v: Any) -> str:
        if v is None:
            return "—"
        try:
            return f"{int(v)}"
        except ValueError:
            return str(v)

    def show(self) -> None:
        date_str = self.calibration_date.strftime("%Y-%m-%d %H:%M:%S")
        resolution = f"{self._fmt_int(self.image_width)}×{self._fmt_int(self.image_height)}"
        reproj = self._fmt_float(self.avg_reprojection_error, ndigits=6)
        aspect = "—" if self.aspect_ratio is None else self._fmt_float(self.aspect_ratio, ndigits=6)

        summary_rows = [
            ("Calibration date", date_str),
            ("Image size", resolution),
            ("Average reprojection error", reproj),
            ("Aspect ratio", aspect),
        ]

        print("\n" + tabulate(summary_rows, tablefmt="rounded_outline"))
        print()

        print("Camera matrix (K)")
        print(self._format_matrix(self.camera_matrix, float_fmt=".6f"))
        print()

        d = np.asarray(self.camera_distortion_matrix)
        d_flat = d.reshape(-1)
        print("Distortion coefficients (D)")
        print(self._format_matrix(d_flat, float_fmt=".8f"))
        print()
