from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from domain.marker_detector import MarkerDetector
from domain.models import LandingTarget


@dataclass(frozen=True, slots=True)
class OpenCVArucoDetectorConfig:
    dictionary_id: int = cv2.aruco.DICT_4X4_250
    corner_refinement: bool = True


class OpenCVArucoDetector(MarkerDetector):
    def __init__(self, cfg: OpenCVArucoDetectorConfig = OpenCVArucoDetectorConfig()):
        self._cfg = cfg
        self._dict = cv2.aruco.getPredefinedDictionary(cfg.dictionary_id)

        params = cv2.aruco.DetectorParameters()
        if cfg.corner_refinement:
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(self._dict, params)

    def detect(
        self, frame: np.ndarray, target: LandingTarget
    ) -> List[Tuple[int, np.ndarray]]:
        corners, ids, _ = self._detector.detectMarkers(frame)
        if ids is None or len(ids) == 0:
            return []

        ids_flat = ids.flatten().tolist()
        results: List[Tuple[int, np.ndarray]] = []

        for marker_id, c in zip(ids_flat, corners):
            if target.marker_id is None or marker_id == target.marker_id:
                results.append((int(marker_id), c))

        return results
