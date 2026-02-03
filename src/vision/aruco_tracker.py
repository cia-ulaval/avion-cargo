from pathlib import Path
from typing import Tuple, NamedTuple
import cv2
import numpy as np
from camera import Camera

class Coordinates3D(NamedTuple):
    x: float
    y: float
    z: float

class ArucoTracker:
    def __init__(self, camera: Camera):
        self.detector_params = cv2.aruco.DetectorParameters()
        self.camera = camera
        self.detector = cv2.aruco.ArucoDetector()

    @staticmethod
    def _write_text_on_image(image, name:str, value: float, xy):
        txt = f"{name}: {value:8.4f}"
        cv2.putText(image, txt, xy, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 252, 124), 2, cv2.LINE_AA)

    def _load_camera_calibration(self, calibration_file: Path):
        pass

    def _configure_aruco_detector_params(self):
        pass

    def track(self, marker_length:float, axis_length: float) -> Tuple[np.ndarray, Coordinates3D]:
        rgb = self.camera.get_frame()
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        vis = frame.copy()

        corners, ids, rejected = self.detector.detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            rotation_vectors, translation_vectors, _obj = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_length, self.camera.matrix, self.camera.distortion_matrix
                )

        for i in range(len(ids)):
            cv2.drawFrameAxes(vis, self.camera.matrix, self.camera.distortion_matrix, rotation_vectors[i], translation_vectors[i], axis_length)

        self._write_text_on_image(vis, "x", float(translation_vectors[0][0][0]), (10, 30))
        self._write_text_on_image(vis, "y", float(translation_vectors[0][0][1]), (10, 55))
        self._write_text_on_image(vis, "z", float(translation_vectors[0][0][2]), (10, 80))

        return vis, Coordinates3D(translation_vectors[0][0][0], translation_vectors[0][0][1], translation_vectors[0][0][2])