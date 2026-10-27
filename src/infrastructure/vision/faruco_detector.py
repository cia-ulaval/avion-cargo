"""Estimate one landing-target pose from visible levels of a fractal marker."""

from dataclasses import dataclass
from typing import Any, List, Optional

import cv2
import numpy as np

from domain.marker_detection import MarkerDetection
from domain.marker_detector import MarkerDetector
from domain.markers import FRACTAL_ROOT_ID, MarkerType
from domain.models import CalibrationData, TargetedMarker


@dataclass(frozen=True, slots=True)
class FArUcoDetectorConfig:
    min_marker_side_px: float = 24.0
    max_reprojection_error_px: float = 3.0
    transition_frames: int = 3
    max_position_jump_m: float = 0.10
    max_position_jump_ratio: float = 0.25
    lost_frames_before_reset: int = 5

    def __post_init__(self) -> None:
        for name in (
            "min_marker_side_px",
            "max_reprojection_error_px",
            "max_position_jump_m",
            "max_position_jump_ratio",
        ):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and > 0")
        for name in ("transition_frames", "lost_frames_before_reset"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be an integer >= 1")


class FArUcoDetector(MarkerDetector):
    """Fuse visible points in the outer marker's metric frame, including inner-only views."""

    def __init__(
        self,
        target: TargetedMarker,
        calibration: CalibrationData,
        bridge_detector: Optional[Any] = None,
        config: Optional[FArUcoDetectorConfig] = None,
    ) -> None:
        if target.marker_type is not MarkerType.FARUCO or target.fractal_type is None:
            raise ValueError("FArUcoDetector requires a TargetedMarker configured for 'faruco'")
        if target.id not in (None, FRACTAL_ROOT_ID):
            raise ValueError("FArUco targets cannot select an individual level; use id=None for automatic tracking")

        self._target = target
        self._calibration = calibration
        self._config = config or FArUcoDetectorConfig()
        self._ratios = target.fractal_type.side_ratios
        self._active_level: Optional[int] = None
        self._last_translation: Optional[np.ndarray] = None
        self._pending_level: Optional[int] = None
        self._pending_translation: Optional[np.ndarray] = None
        self._pending_count = 0
        self._lost_count = 0

        if bridge_detector is None:
            try:
                from faruco_bridge import Detector
            except ImportError as error:
                raise RuntimeError(
                    "FArUco support requires the optional 'faruco-bridge' dependency; "
                    "install it with 'poetry install --with faruco'."
                ) from error

            bridge_detector = Detector(
                fractal_type=target.fractal_type.value,
                marker_size_m=target.length,
                camera_matrix=calibration.camera_matrix,
                dist_coeffs=calibration.dist_coeffs,
                include_points=True,
            )

        self._bridge_detector = bridge_detector

    def detect(self, frame: np.ndarray, target: TargetedMarker) -> List[MarkerDetection]:
        if target.marker_type is not MarkerType.FARUCO:
            raise ValueError("FArUcoDetector cannot detect a conventional ArUco target")
        if target != self._target:
            raise ValueError("Create a new FArUcoDetector when changing the target")

        result = self._bridge_detector.detect(frame, include_points=True)
        candidates = []
        for marker in result.markers:
            if not 0 <= marker.id < len(self._ratios):
                continue
            if getattr(marker, "type", target.fractal_type) != target.fractal_type:
                continue
            candidate = self._estimate(marker, frame.shape[:2])
            if candidate is not None:
                candidates.append(candidate)

        # Prefer well-spread observations, then more inliers and lower residuals.
        candidates.sort(key=lambda item: item[1:], reverse=True)
        for detection, *_ in candidates:
            translation = detection.translation_vectors.reshape(3)
            if self._last_translation is None or self._consistent(translation, self._last_translation):
                confirmed = self._confirm(detection)
                if confirmed:
                    return confirmed
                # Continue publishing a CURRENT observation of the active level
                # while a better candidate is being confirmed. Never replay a
                # detection from a previous frame when the active level is absent.
                for active, *_ in candidates:
                    if active.marker_id == self._active_level and self._consistent(
                        active.translation_vectors.reshape(3), self._last_translation
                    ):
                        self._last_translation = active.translation_vectors.reshape(3).copy()
                        return [active]
                return []

        self._pending_level = None
        self._pending_translation = None
        self._pending_count = 0
        self._miss()
        return []

    def _object_corners(self, level: int) -> np.ndarray:
        half = self._target.length * self._ratios[level] / 2
        return np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)

    def _estimate(self, marker: Any, shape: tuple[int, int]) -> Optional[tuple]:
        corners = np.asarray(marker.corners, dtype=np.float64)
        if corners.size != 8 or not np.all(np.isfinite(corners)):
            return None
        corners = corners.reshape(4, 2)
        height, width = shape
        if np.any(corners < 2) or np.any(corners[:, 0] > width - 3) or np.any(corners[:, 1] > height - 3):
            return None
        if np.min(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)) < self._config.min_marker_side_px:
            return None
        if not cv2.isContourConvex(corners.astype(np.float32)):
            return None

        object_points = getattr(marker, "object_points", None)
        image_points = getattr(marker, "image_points", None)
        if object_points is None and image_points is None:
            object_points, image_points = self._object_corners(marker.id), corners
        elif object_points is None or image_points is None:
            return None
        else:
            object_points = np.asarray(object_points, dtype=np.float64)
            image_points = np.asarray(image_points, dtype=np.float64)
            if object_points.ndim != 2 or object_points.shape[1] != 3:
                return None
            if image_points.ndim != 2 or image_points.shape != (len(object_points), 2):
                return None
            # Older workers use the OUTER template when only an inner contour
            # supplies four correspondences. Repair that exact fallback only;
            # correctly scaled multi-level correspondences remain untouched.
            if (
                marker.id != FRACTAL_ROOT_ID
                and len(object_points) == 4
                and np.allclose(image_points, corners, atol=1e-3)
                and np.allclose(object_points, self._object_corners(FRACTAL_ROOT_ID), rtol=1e-5, atol=1e-7)
            ):
                object_points = self._object_corners(marker.id)

        if len(object_points) < 4 or not np.all(np.isfinite(object_points)) or not np.all(np.isfinite(image_points)):
            return None
        inside = (
            (image_points[:, 0] >= 0)
            & (image_points[:, 0] < width)
            & (image_points[:, 1] >= 0)
            & (image_points[:, 1] < height)
        )
        object_points, image_points = object_points[inside], image_points[inside]
        if len(object_points) < 4:
            return None
        if np.linalg.matrix_rank(object_points - object_points.mean(axis=0)) < 2:
            return None

        calibration = self._calibration
        try:
            if len(object_points) > 4:
                solved, rvec, tvec, inliers = cv2.solvePnPRansac(
                    object_points,
                    image_points,
                    calibration.camera_matrix,
                    calibration.dist_coeffs,
                    iterationsCount=100,
                    reprojectionError=self._config.max_reprojection_error_px,
                    confidence=0.99,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if not solved or inliers is None or len(inliers) < max(4, int(np.ceil(len(object_points) * 0.6))):
                    return None
                object_points, image_points = object_points[inliers.ravel()], image_points[inliers.ravel()]
                solved, rvec, tvec = cv2.solvePnP(
                    object_points, image_points, calibration.camera_matrix, calibration.dist_coeffs, rvec, tvec, True
                )
            else:
                solved, rvec, tvec = cv2.solvePnP(
                    object_points, image_points, calibration.camera_matrix, calibration.dist_coeffs
                )
            if not solved or not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)) or tvec.reshape(3)[2] <= 0:
                return None
            rotation, _ = cv2.Rodrigues(rvec)
            if np.any((object_points @ rotation.T + tvec.reshape(3))[:, 2] <= 0):
                return None
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec, calibration.camera_matrix, calibration.dist_coeffs
            )
        except cv2.error:
            return None

        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
        if not np.isfinite(rms) or rms > self._config.max_reprojection_error_px:
            return None
        area = cv2.contourArea(cv2.convexHull(image_points.astype(np.float32)))
        if area < self._config.min_marker_side_px**2:
            return None
        detection = MarkerDetection(
            marker_id=int(marker.id),
            corners=corners.astype(np.float32).reshape(1, 4, 2),
            rotation_vectors=rvec.reshape(1, 1, 3),
            translation_vectors=tvec.reshape(1, 1, 3),
            reprojection_error_px=rms,
        )
        return detection, area, len(object_points), -rms

    def _consistent(self, current: np.ndarray, previous: np.ndarray) -> bool:
        limit = self._config.max_position_jump_m + self._config.max_position_jump_ratio * previous[2]
        return bool(np.linalg.norm(current - previous) <= limit)

    def _confirm(self, detection: MarkerDetection) -> List[MarkerDetection]:
        translation = detection.translation_vectors.reshape(3)
        self._lost_count = 0
        if self._active_level != detection.marker_id:
            if (
                self._pending_level == detection.marker_id
                and self._pending_translation is not None
                and self._consistent(translation, self._pending_translation)
            ):
                self._pending_count += 1
            else:
                self._pending_count = 1
            self._pending_level = detection.marker_id
            self._pending_translation = translation.copy()
            if self._pending_count < self._config.transition_frames:
                return []

        self._active_level = detection.marker_id
        self._last_translation = translation.copy()
        self._pending_level = None
        self._pending_translation = None
        self._pending_count = 0
        return [detection]

    def _miss(self) -> None:
        self._lost_count += 1
        if self._lost_count >= self._config.lost_frames_before_reset:
            self._active_level = None
            self._last_translation = None

    def close(self) -> None:
        self._bridge_detector.close()
