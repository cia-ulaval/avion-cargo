from enum import Enum

from camera import Camera


class CalibrationMethod(Enum):
    ARUCO_GRID_BOARD = 0


class CameraCalibrator:
    def __init__(self, camera: Camera, calibration_method: CalibrationMethod):
        self.camera = camera
        pass

    def save_calibration_params(self):
        pass

    def calibrate(self):
        pass
