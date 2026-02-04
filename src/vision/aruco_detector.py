from typing import NamedTuple

import cv2

class ArucoDetectorParams(NamedTuple):
    adaptiveThreshWinSizeMin: int
    adaptiveThreshWinSizeMax: int
    adaptiveThreshWinSizeStep: int
    cornerRefinementMethod: int
    minCornerDistanceRate: float
    minMarkerDistanceRate: float
    polygonalApproxAccuracyRate: float

class ArucoDetector:
    def __init__(self, detector_params: ArucoDetectorParams):
        self.detector_params = detector_params
        self.__detector: cv2.aruco.ArucoDetector = cv2.aruco.ArucoDetector()