"""Adapter from FArUco Bridge detections to AvionCargo's marker model."""

from typing import Any, List, Optional

import numpy as np

from domain.marker_detection import MarkerDetection
from domain.marker_detector import MarkerDetector
from domain.models import CalibrationData, TargetedMarker


class FArUcoDetector(MarkerDetector):
    """Detect a configured fractal marker and reuse its multi-point pose estimate."""

    def __init__(
        self,
        target: TargetedMarker,
        calibration: CalibrationData,
        bridge_detector: Optional[Any] = None,
    ) -> None:
        if target.marker_type != "faruco" or target.fractal_type is None:
            raise ValueError("FArUcoDetector requires a TargetedMarker configured for 'faruco'")

        if bridge_detector is None:
            try:
                from faruco_bridge import Detector
            except ImportError as error:
                raise RuntimeError(
                    "FArUco support requires the optional 'faruco-bridge' dependency; "
                    "install it with 'poetry install --with faruco'."
                ) from error

            bridge_detector = Detector(
                fractal_type=target.fractal_type,
                marker_size_m=target.length,
                camera_matrix=calibration.camera_matrix,
                dist_coeffs=calibration.dist_coeffs,
                include_points=False,
            )

        self._bridge_detector = bridge_detector

    def detect(self, frame: np.ndarray, target: TargetedMarker) -> List[MarkerDetection]:
        if target.marker_type != "faruco":
            raise ValueError("FArUcoDetector cannot detect a conventional ArUco target")

        result = self._bridge_detector.detect(frame, include_points=False)
        detections: List[MarkerDetection] = []
        for marker in result.markers:
            if target.id is not None and marker.id != target.id:
                continue

            rotation_vectors = None
            translation_vectors = None
            if marker.rvec is not None and marker.tvec is not None:
                # Normalize bridge vectors to the (N, 1, 3) layout used by the
                # existing OpenCV annotator.
                rotation_vectors = np.asarray(marker.rvec, dtype=np.float64).reshape(1, 1, 3)
                translation_vectors = np.asarray(marker.tvec, dtype=np.float64).reshape(1, 1, 3)

            detections.append(
                MarkerDetection(
                    marker_id=int(marker.id),
                    corners=np.asarray(marker.corners, dtype=np.float32).reshape(1, 4, 2),
                    rotation_vectors=rotation_vectors,
                    translation_vectors=translation_vectors,
                    reprojection_error_px=marker.reprojection_error_px,
                )
            )

        return detections

    def close(self) -> None:
        self._bridge_detector.close()
