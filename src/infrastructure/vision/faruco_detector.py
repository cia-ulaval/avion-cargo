"""Use FArUco Bridge directly for detection, pose estimation and annotation."""

from typing import Any, List, Optional

import numpy as np

from domain.frame_annotator import FrameAnnotator
from domain.marker_detection import MarkerDetection
from domain.marker_detector import MarkerDetector
from domain.markers import FRACTAL_ROOT_ID, MarkerType
from domain.models import CalibrationData, TargetedMarker


class FArUcoDetector(MarkerDetector, FrameAnnotator):
    """Adapt bridge results without filtering detections or recalculating poses."""

    def __init__(
        self,
        target: TargetedMarker,
        calibration: CalibrationData,
        bridge_detector: Optional[Any] = None,
    ) -> None:
        if target.marker_type is not MarkerType.FARUCO or target.fractal_type is None:
            raise ValueError("FArUcoDetector requires a TargetedMarker configured for 'faruco'")
        if target.id not in (None, FRACTAL_ROOT_ID):
            raise ValueError("FArUco targets cannot select an individual level; use id=None for automatic tracking")

        self._target = target
        self._last_result: Optional[Any] = None

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
            )

        self._bridge_detector = bridge_detector

    def detect(self, frame: np.ndarray, target: TargetedMarker) -> List[MarkerDetection]:
        if target.marker_type is not MarkerType.FARUCO:
            raise ValueError("FArUcoDetector cannot detect a conventional ArUco target")
        if target != self._target:
            raise ValueError("Create a new FArUcoDetector when changing the target")

        self._last_result = self._bridge_detector.detect(frame)
        return [
            MarkerDetection(
                marker_id=marker.id,
                corners=marker.corners.reshape(1, 4, 2),
                rotation_vectors=None if marker.rvec is None else marker.rvec.reshape(1, 1, 3),
                translation_vectors=None if marker.tvec is None else marker.tvec.reshape(1, 1, 3),
                reprojection_error_px=marker.reprojection_error_px,
            )
            for marker in self._last_result.markers
        ]

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
        """Render the current bridge result exactly as in its webcam example."""
        if self._last_result is None:
            return frame
        return self._last_result.annotate(frame)

    def close(self) -> None:
        self._bridge_detector.close()
